
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
#include <artery/application/md/util/CustomRing.h>
#include <inet/common/ModuleAccess.h>
#include "artery/traci/Cast.h"

namespace artery {
    using namespace omnetpp;

    Define_Module(MisbehaviorDetectionService);

    static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");

    bool MisbehaviorDetectionService::staticInitializationComplete = false;
    std::map<uint32_t, misbehaviorTypes::MisbehaviorTypes> MisbehaviorDetectionService::mStationIdMisbehaviorTypeMap;
    std::shared_ptr<const traci::API> MisbehaviorDetectionService::traciAPI;
    std::vector<std::vector<traci::Boundary>> MisbehaviorDetectionService::gridCellBoundaries;
    std::vector<std::vector<std::vector<std::tuple<std::string, geometry::Box *, std::shared_ptr<EnvironmentModelObstacle>>>>> MisbehaviorDetectionService::gridCellObstacles;

    MisbehaviorDetectionService::MisbehaviorDetectionService() {
        curl = curl_easy_init();
    }

    MisbehaviorDetectionService::~MisbehaviorDetectionService() {
        cancelAndDelete(m_self_msg);
        curl_easy_cleanup(curl);
    }

    void MisbehaviorDetectionService::initialize() {
        ItsG5BaseService::initialize();
        m_self_msg = new cMessage("InitializeMessage");
        subscribe(scSignalCamReceived);
        scheduleAt(simTime() + 3.0, m_self_msg);
        mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
        mLocalEnvironmentModel = getFacilities().get_mutable_ptr<LocalEnvironmentModel>();
        mGlobalEnvironmentModel = mLocalEnvironmentModel->getGlobalEnvMod();
        if (!staticInitializationComplete) {
            staticInitializationComplete = true;
            traciAPI = getFacilities().get_const<traci::VehicleController>().getTraCI();
            initializeParameters();
            TraCIAPI::PolygonScope polygon = traciAPI->polygon;
            traci::Boundary outerBoundary{traciAPI->simulation.getNetBoundary()};
            int splitGridSize = F2MDParameters::miscParameters.objectAccessHelperGridSize;
            double gridLengthX =
                    (outerBoundary.upperRightPosition().x - outerBoundary.lowerLeftPosition().x) / splitGridSize;
            double gridLengthY =
                    (outerBoundary.upperRightPosition().y - outerBoundary.lowerLeftPosition().y) / splitGridSize;
            std::cout << "size: x: " << gridLengthX << " y: " << gridLengthY << std::endl;
            for (int i = 0; i < splitGridSize; i++) {
                std::vector<traci::Boundary> row;
                std::vector<std::vector<std::tuple<std::string, geometry::Box *, std::shared_ptr<EnvironmentModelObstacle>>>> rowObstacle;
                for (int j = 0; j < splitGridSize; j++) {
                    libsumo::TraCIPositionVector v;
                    v.value.emplace_back(Position(i * gridLengthX + outerBoundary.lowerLeftPosition().x,
                                                  j * gridLengthY +
                                                  outerBoundary.lowerLeftPosition().y).toTraCIPosition());
                    v.value.emplace_back(Position((i + 1) * gridLengthX + outerBoundary.lowerLeftPosition().x,
                                                  (j + 1) * gridLengthY +
                                                  outerBoundary.lowerLeftPosition().y).toTraCIPosition());
                    traci::Boundary currentBoundary{v};
                    row.emplace_back(currentBoundary);
                    std::vector<std::tuple<std::string, geometry::Box *, std::shared_ptr<EnvironmentModelObstacle>>> cellObstacle;
                    rowObstacle.emplace_back(cellObstacle);
                    std::string id = "outlinePoly-";
                    id += std::to_string(i);
                    id += " ";
                    id += std::to_string(j);
                    polygon.add(id, CustomRing(currentBoundary).toTraCiPositionVector(),
                                libsumo::TraCIColor(255, 0, 255, 255), false, "helper", 5);
                }
                gridCellBoundaries.emplace_back(row);
                gridCellObstacles.emplace_back(rowObstacle);
            }
            std::cout << "created grid" << std::endl;
            ObstacleRtree *obstacleRtree = mGlobalEnvironmentModel->getObstacleRTree();
            ObstacleDB *obstacles = mGlobalEnvironmentModel->getObstacleDB();
            for (auto treeObject : *obstacleRtree) {

                Position obstacleLowerLeft = Position(position_cast(outerBoundary,
                                                                    Position(treeObject.first.min_corner().get<0>(),
                                                                             treeObject.first.max_corner().get<1>())));
                Position obstacleUpperRight = Position(position_cast(outerBoundary,
                                                                     Position(treeObject.first.max_corner().get<0>(),
                                                                              treeObject.first.min_corner().get<1>())));
                for (int i = 0; i < splitGridSize; i++) {
                    if (obstacleLowerLeft.x.value() >= gridCellBoundaries[i][0].lowerLeftPosition().x &&
                        obstacleLowerLeft.x.value() <= gridCellBoundaries[i][0].upperRightPosition().x ||
                        obstacleUpperRight.x.value() >= gridCellBoundaries[i][0].lowerLeftPosition().x &&
                        obstacleUpperRight.x.value() <= gridCellBoundaries[i][0].upperRightPosition().x) {
                        for (int j = 0; j < splitGridSize; j++) {
                            if (obstacleLowerLeft.y.value() >= gridCellBoundaries[0][j].lowerLeftPosition().y &&
                                obstacleLowerLeft.y.value() <= gridCellBoundaries[0][j].upperRightPosition().y ||
                                obstacleUpperRight.y.value() >= gridCellBoundaries[0][j].lowerLeftPosition().y &&
                                obstacleUpperRight.y.value() <= gridCellBoundaries[0][j].upperRightPosition().y) {
                                if (i == 3 && j == 3) {
                                    std::cout << "NEW obstacle in box: " << treeObject.second << std::endl;
                                }
                                std::tuple<std::string, geometry::Box *, std::shared_ptr<EnvironmentModelObstacle>> tuple = std::make_tuple(
                                        treeObject.second, &treeObject.first, (*obstacles)[treeObject.second]);
                                gridCellObstacles[i][j].emplace_back(tuple);
                            }
                        }
                    }
                }
            }
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

    void MisbehaviorDetectionService::receiveSignal(cComponent *source, simsignal_t signal, cObject *c_obj, cObject *) {
        Enter_Method("receiveSignal");
        if (signal == scSignalCamReceived) {
            CaObject *ca = dynamic_cast<CaObject *>(c_obj);
            vanetza::asn1::Cam message = ca->asn1();
            uint32_t senderStationId = message->header.stationID;
            std::cout << mVehicleDataProvider->getStationId() << " <-- " << senderStationId << std::endl;

            traci::TraCIGeoPosition traciGeoPositionSelf = {
                    mVehicleDataProvider->longitude().value(),
                    mVehicleDataProvider->latitude().value()};
            traci::TraCIPosition traciPositionSelf = traciAPI->convert2D(traciGeoPositionSelf);
            Position ownPosition = Position(traciPositionSelf.x, traciPositionSelf.y);
            auto &allObjects = mLocalEnvironmentModel->allObjects();
            TrackedObjectsFilterRange envModObjects = filterBySensorCategory(allObjects, "CA");
            CheckResult *result;
            if (detectedSenders.find(senderStationId) != detectedSenders.end()) {
                result = detectedSenders[senderStationId]->addAndCheckCam(message, ownPosition,
                                                                          envModObjects);
            } else {
                detectedSenders[senderStationId] = new DetectedSender(traciAPI, &F2MDParameters::detectionParameters,
                                                                      message);
                result = detectedSenders[senderStationId]->addAndCheckCam(message, ownPosition,
                                                                          envModObjects);
            }
            std::cout << result->toString(0.5) << std::endl;
            ReferencePosition_t referencePosition = message->cam.camParameters.basicContainer.referencePosition;
            traci::TraCIGeoPosition traciGeoPositionSender = {
                    (double) referencePosition.longitude / 10000000.0,
                    (double) referencePosition.latitude / 10000000.0};
            traci::TraCIPosition traciPositionSender = traciAPI->convert2D(traciGeoPositionSender);
            Position senderPosition = Position(traciPositionSender.x, traciPositionSender.y);


            int splitGridSize = F2MDParameters::miscParameters.objectAccessHelperGridSize;
            for (int i = 0; i < splitGridSize; i++) {
                if (senderPosition.x.value() >= gridCellBoundaries[i][0].lowerLeftPosition().x &&
                    senderPosition.x.value() <= gridCellBoundaries[i][0].upperRightPosition().x)
                    for (int j = 0; j < splitGridSize; j++) {
                        if (senderPosition.y.value() >= gridCellBoundaries[0][j].lowerLeftPosition().y &&
                            senderPosition.y.value() <= gridCellBoundaries[0][j].upperRightPosition().y) {
                            std::string id = "outlinePoly-";
                            id += std::to_string(i);
                            id += " ";
                            id += std::to_string(j);
                            traciAPI->polygon.setColor(id, libsumo::TraCIColor(0, 255, 255, 255));
                            std::cout << "sender in " << id << std::endl;
                            for (auto obstacleTuple : gridCellObstacles[i][j]) {
                                std::cout << std::get<0>(obstacleTuple) << ", ";
                            }
                            std::cout << std::endl;
                        }
                    }
            }


            misbehaviorTypes::MisbehaviorTypes senderMisbehaviorType = getMisbehaviorTypeOfStationId(senderStationId);
            if (senderMisbehaviorType == misbehaviorTypes::LocalAttacker) {
                EV_INFO << "Received manipulated CAM!";
                EV_INFO << "Received DoS " << simTime().inUnit(SimTimeUnit::SIMTIME_MS) << " delta:  "
                        << message->cam.generationDeltaTime << "\n";
                vanetza::ByteBuffer byteBuffer = ca->asn1().encode();
                std::string encoded(byteBuffer.begin(), byteBuffer.end());
                std::string b64Encoded = base64_encode(reinterpret_cast<const unsigned char *>(encoded.c_str()),
                                                       encoded.length(), false);
                if (curl) {
                    curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:9981/newCAM");
                    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, b64Encoded.c_str());
                    CURLcode curlResponse = curl_easy_perform(curl);
                    if (curlResponse != CURLE_OK) {
                        EV_ERROR << "curl_easy_perform() failed: %s\n"
                                 << curl_easy_strerror(curlResponse);
                    }
                }
            } else if (senderMisbehaviorType == misbehaviorTypes::Benign) {
                EV_INFO << "Received benign CAM!";
            } else {
                EV_INFO << "Received weird misbehaviorType";
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
