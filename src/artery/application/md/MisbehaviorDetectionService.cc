
#include "artery/application/md/MisbehaviorDetectionService.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
#include <vanetza/asn1/cam.hpp>
#include "artery/application/md/base64.h"
#include "artery/application/CaService.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/md/MisbehaviorTypes.h"
#include "artery/envmod/sensor/Sensor.h"
#include <boost/units/io.hpp>
#include <inet/common/ModuleAccess.h>
#include "artery/traci/Cast.h"
#include "artery/application/md/MisbehaviorCaService.h"

namespace artery {
    using namespace omnetpp;

    Define_Module(MisbehaviorDetectionService);

    static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");

    bool MisbehaviorDetectionService::staticInitializationComplete = false;
    std::map<uint32_t, misbehaviorTypes::MisbehaviorTypes> MisbehaviorDetectionService::mStationIdMisbehaviorTypeMap;
    std::shared_ptr<const traci::API> MisbehaviorDetectionService::traciAPI;

    traci::Boundary MisbehaviorDetectionService::simulationBoundary;

    MisbehaviorDetectionService::MisbehaviorDetectionService() {
        curl = curl_easy_init();
    }

    MisbehaviorDetectionService::~MisbehaviorDetectionService() {
        cancelAndDelete(m_self_msg);
        curl_easy_cleanup(curl);

//        while (!activePoIs.empty()) {
//            traciPoiScope->remove(activePoIs.front());
//            activePoIs.pop_front();
//        }
    }

    void MisbehaviorDetectionService::initialize() {
        ItsG5BaseService::initialize();
        m_self_msg = new cMessage("InitializeMessage");
        subscribe(scSignalCamReceived);
        scheduleAt(simTime() + 3.0, m_self_msg);
        mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
        mLocalEnvironmentModel = getFacilities().get_mutable_ptr<LocalEnvironmentModel>();
        mGlobalEnvironmentModel = mLocalEnvironmentModel->getGlobalEnvMod();
        mVehicleController = &getFacilities().get_const<traci::VehicleController>();

        if (!staticInitializationComplete) {
            staticInitializationComplete = true;
            traciAPI = getFacilities().get_const<traci::VehicleController>().getTraCI();
            traciPoiScope = &traciAPI->poi;
            TraCIAPI::PolygonScope polygon = traciAPI->polygon;
            simulationBoundary = traci::Boundary{traciAPI->simulation.getNetBoundary()};
            initializeParameters();
        }
    }


    void MisbehaviorDetectionService::initializeParameters() {
        F2MDParameters::detectionParameters.maxPlausibleSpeed = par("maxPlausibleSpeed");
        F2MDParameters::detectionParameters.maxPlausibleAcceleration = par("maxPlausibleAcceleration");
        F2MDParameters::detectionParameters.maxPlausibleDeceleration = par("maxPlausibleDeceleration");
        F2MDParameters::detectionParameters.maxPlausibleRange = par("maxPlausibleRange");

        F2MDParameters::detectionParameters.maxProximityRangeL = par("maxProximityRangeL");
        F2MDParameters::detectionParameters.maxProximityRangeW = par("maxProximityRangeW");
        F2MDParameters::detectionParameters.maxProximityDistance = par("maxProximityDistance");
        F2MDParameters::detectionParameters.maxTimeDelta = par("maxTimeDelta");
        F2MDParameters::detectionParameters.maxMgtRng = par("maxMgtRng");
        F2MDParameters::detectionParameters.maxMgtRngDown = par("maxMgtRngDown");
        F2MDParameters::detectionParameters.maxMgtRngUp = par("maxMgtRngUp");
        F2MDParameters::detectionParameters.maxSuddenAppearanceRange = par("maxSuddenAppearanceRange");
        F2MDParameters::detectionParameters.maxSuddenAppearanceTime = par("maxSuddenAppearanceTime");
        F2MDParameters::detectionParameters.maxCamFrequency = par("maxCamFrequency");
        F2MDParameters::detectionParameters.maxOffroadSpeed = par("maxOffroadSpeed");
        F2MDParameters::detectionParameters.positionHeadingTime = par("positionHeadingTime");
        F2MDParameters::detectionParameters.maxHeadingChange = par("maxHeadingChange");

        F2MDParameters::detectionParameters.maxKalmanTime = par("maxKalmanTime");
        F2MDParameters::detectionParameters.kalmanMinPosRange = par("kalmanMinPosRange");
        F2MDParameters::detectionParameters.kalmanMinSpeedRange = par("kalmanMinSpeedRange");
        F2MDParameters::detectionParameters.kalmanMinHeadingRange = par("kalmanMinHeadingRange");
        F2MDParameters::detectionParameters.kalmanPosRange = par("kalmanPosRange");
        F2MDParameters::detectionParameters.kalmanSpeedRange = par("kalmanSpeedRange");

        F2MDParameters::miscParameters.objectAccessHelperGridSize = par("objectAccessHelperGridSize");
    }


    void MisbehaviorDetectionService::trigger() {
        Enter_Method("trigger");
    }

    void MisbehaviorDetectionService::indicate(const vanetza::btp::DataIndication &ind, cPacket *packet,
                                               const NetworkInterface &net) {
        Enter_Method("indicate");

        EV_INFO << "packet indication on channel " << net.channel << "with byte length" << packet->getByteLength()
                << "\n";

        delete (packet);
    }

    void MisbehaviorDetectionService::handleMessage(cMessage *msg) {
        Enter_Method("handleMessage");

        if (msg == m_self_msg) {
            EV_INFO << "self message\n";
        }
    }

    void MisbehaviorDetectionService::visualizeCamPosition(vanetza::asn1::Cam cam, const libsumo::TraCIColor &color,
                                                           const std::string &idPrefix) {
        traci::TraCIGeoPosition traciGeoPosition = {
                (double) cam->cam.camParameters.basicContainer.referencePosition.longitude / 10000000.0,
                (double) cam->cam.camParameters.basicContainer.referencePosition.latitude / 10000000.0};
        traci::TraCIPosition traciPosition = mVehicleController->getTraCI()->convert2D(traciGeoPosition);
        std::string poiId = mVehicleController->getVehicleId();
        poiId += idPrefix;
        poiId += "_CAM_";
        poiId += std::to_string(cam->cam.generationDeltaTime);
        traciPoiScope->add(poiId, traciPosition.x, traciPosition.y, color,
                           poiId, 5, "", 0,
                           0, 0);
        activePoIs.push_back(poiId);
        if (activePoIs.size() > F2MDParameters::miscParameters.CamLocationVisualizerMaxLength) {
            traciPoiScope->remove(activePoIs.front());
            activePoIs.pop_front();
        }
        int alphaStep = 185 / F2MDParameters::miscParameters.CamLocationVisualizerMaxLength;
        int currentAlpha = 80;
//        for(const auto& poi : activePoIs){
//            traciPoiScope->setColor(poi,color);
//            currentAlpha += alphaStep;
//        }
    }

    void MisbehaviorDetectionService::receiveSignal(cComponent *source, simsignal_t signal, cObject *c_obj, cObject *) {
        Enter_Method("receiveSignal");
        if (signal == scSignalCamReceived) {
            auto *ca = dynamic_cast<CaObject *>(c_obj);
            vanetza::asn1::Cam message = ca->asn1();
            uint32_t senderStationId = message->header.stationID;
//            std::cout << mVehicleDataProvider->getStationId() << " <-- " << senderStationId << std::endl;

//            traci::TraCIGeoPosition traciGeoPositionSelf = {
//                    mVehicleDataProvider->longitude().value(),
//                    mVehicleDataProvider->latitude().value()};
//            traci::TraCIPosition traciPositionSelf = mTraciAPI->convert2D(traciGeoPositionSelf);
//            Position ownPosition = Position(traciPositionSelf.x, traciPositionSelf.y);
//            auto &allObjects = mLocalEnvironmentModel->allObjects();
//            TrackedObjectsFilterRange envModObjects = filterBySensorCategory(allObjects, "CA");
//            CheckResult *result;
//            if (detectedSenders.find(senderStationId) != detectedSenders.end()) {
//                result = detectedSenders[senderStationId]->addAndCheckCam(message, ownPosition,
//                                                                          envModObjects);
//            } else {
//                detectedSenders[senderStationId] = new DetectedSender(mTraciAPI, &F2MDParameters::detectionParameters,
//                                                                      message);
//                result = detectedSenders[senderStationId]->addAndCheckCam(message, ownPosition,
//                                                                          envModObjects);
//            }
//            std::cout << result->toString(0.5) << std::endl;

            misbehaviorTypes::MisbehaviorTypes senderMisbehaviorType = getMisbehaviorTypeOfStationId(senderStationId);
//            if (senderMisbehaviorType == misbehaviorTypes::LocalAttacker) {
//                EV_INFO << "Received manipulated CAM!";
//                EV_INFO << "Received DoS " << simTime().inUnit(SimTimeUnit::SIMTIME_MS) << " delta:  "
//                        << message->cam.generationDeltaTime << "\n";
//                vanetza::ByteBuffer byteBuffer = ca->asn1().encode();
//                std::string encoded(byteBuffer.begin(), byteBuffer.end());
//                std::string b64Encoded = base64_encode(reinterpret_cast<const unsigned char *>(encoded.c_str()),
//                                                       encoded.length(), false);
//                if (curl) {
//                    curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:9981/newCAM");
//                    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, b64Encoded.c_str());
//                    CURLcode curlResponse = curl_easy_perform(curl);
//                    if (curlResponse != CURLE_OK) {
//                        EV_ERROR << "curl_easy_perform() failed: %s\n"
//                                 << curl_easy_strerror(curlResponse);
//                    }
//                }
//            } else if (senderMisbehaviorType == misbehaviorTypes::Benign) {
//                EV_INFO << "Received benign CAM!";
//            } else {
//                EV_INFO << "Received weird misbehaviorType";
//            }


            ReferencePosition_t referencePosition = message->cam.camParameters.basicContainer.referencePosition;
            traci::TraCIGeoPosition traciGeoPositionSender = {
                    (double) referencePosition.longitude / 10000000.0,
                    (double) referencePosition.latitude / 10000000.0};
            Position senderPosition = position_cast(simulationBoundary, traciAPI->convert2D(traciGeoPositionSender));
            bool messageHasPoi = false;
            if (senderMisbehaviorType == misbehaviorTypes::LocalAttacker) {
                GridCell gridCell = mGlobalEnvironmentModel->getGridCellFromPosition(senderPosition);
                for (const auto &obstacleStruct : gridCell.obstacles) {
                    if (boost::geometry::within(senderPosition, *obstacleStruct->boundingBox)) {
                        if (boost::geometry::within(senderPosition, *obstacleStruct->outline)) {
                            if (!messageHasPoi) {
                                std::string prefix = "inside obstacle";
                                prefix += obstacleStruct->id;
                                visualizeCamPosition(message, libsumo::TraCIColor(0, 255, 255, 255),
                                                     prefix);
                                messageHasPoi = true;
                            }
                        }
                    }
                }
                for (const auto &laneStruct: gridCell.lanes) {
                    if (boost::geometry::within(senderPosition, *laneStruct->boundingBox)) {
                        if (boost::geometry::distance(senderPosition, *laneStruct->shape) <
                            laneStruct->width / 2) {
                            if (!messageHasPoi) {
                                std::string prefix = "inside lane";
                                prefix += laneStruct->id;
                                visualizeCamPosition(message, libsumo::TraCIColor(255, 255, 0, 255),
                                                     prefix);

                                messageHasPoi = true;
                            }
                        }
                    }
                }
                for (const auto &junctionStruct : gridCell.junctions) {
                    if (boost::geometry::within(senderPosition, *junctionStruct->boundingBox)) {
                        if (boost::geometry::within(senderPosition, *junctionStruct->outline)) {
                            if (!messageHasPoi) {
                                std::string prefix = "inside junction";
                                prefix += junctionStruct->id;
                                visualizeCamPosition(message, libsumo::TraCIColor(255, 0, 255, 255),
                                                     prefix);

                                messageHasPoi = true;
                            }
                        }
                    }
                }
            }
//            if (!messageHasPoi) {
            if (!messageHasPoi && senderMisbehaviorType == misbehaviorTypes::LocalAttacker) {
                visualizeCamPosition(message, libsumo::TraCIColor(0, 255, 0, 255),
                                     "outside");
            }
        }
    }


    misbehaviorTypes::MisbehaviorTypes MisbehaviorDetectionService::getMisbehaviorTypeOfStationId(uint32_t stationId) {
        auto search = mStationIdMisbehaviorTypeMap.find(stationId);
        if (search != mStationIdMisbehaviorTypeMap.end()) {
            return search->second;
        } else {
            return misbehaviorTypes::SIZE_OF_ENUM;
        }
    }

    void MisbehaviorDetectionService::addStationIdToVehicleList(uint32_t stationId,
                                                                misbehaviorTypes::MisbehaviorTypes misbehaviorType) {
        mStationIdMisbehaviorTypeMap[stationId] = misbehaviorType;
    }

    void MisbehaviorDetectionService::removeStationIdFromVehicleList(uint32_t stationId) {
        auto it = mStationIdMisbehaviorTypeMap.find(stationId);
        if (it != mStationIdMisbehaviorTypeMap.end()) {
            mStationIdMisbehaviorTypeMap.erase(it);
        }
    }

}
