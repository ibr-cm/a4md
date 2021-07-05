
#include "artery/application/md/MisbehaviorDetectionService.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
#include <vanetza/asn1/cam.hpp>
#include "artery/application/md/util/base64.h"
#include "artery/application/CaService.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/md/util/MisbehaviorTypes.h"
#include "artery/envmod/sensor/Sensor.h"
#include <inet/common/ModuleAccess.h>
#include "artery/traci/Cast.h"
#include "artery/application/md/MisbehaviorCaService.h"
#include "MisbehaviorReportObject.h"
#include <boost/math/constants/constants.hpp>
#include <boost/units/cmath.hpp>
#include <boost/math/constants/info.hpp>
#include <bitset>

namespace artery {
    using namespace omnetpp;

    Define_Module(MisbehaviorDetectionService);

    static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
    static const simsignal_t scSignalMisbehaviorAuthorityNewReport = cComponent::registerSignal("misbehaviorAuthority.newReport");

    bool MisbehaviorDetectionService::staticInitializationComplete = false;
    std::map<uint32_t, misbehaviorTypes::MisbehaviorTypes> MisbehaviorDetectionService::mStationIdMisbehaviorTypeMap;
    std::shared_ptr<const traci::API> MisbehaviorDetectionService::mTraciAPI;
    GlobalEnvironmentModel *MisbehaviorDetectionService::mGlobalEnvironmentModel;

    traci::Boundary MisbehaviorDetectionService::mSimulationBoundary;

    MisbehaviorDetectionService::MisbehaviorDetectionService() {
        curl = curl_easy_init();
    }

    MisbehaviorDetectionService::~MisbehaviorDetectionService() {
        curl_easy_cleanup(curl);

//        while (!activePoIs.empty()) {
//            traciPoiScope->remove(activePoIs.front());
//            activePoIs.pop_front();
//        }
    }

    void MisbehaviorDetectionService::initialize() {
        ItsG5BaseService::initialize();
        subscribe(scSignalCamReceived);
        mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
        mVehicleController = &getFacilities().get_const<traci::VehicleController>();
        mLocalEnvironmentModel = getFacilities().get_mutable_ptr<LocalEnvironmentModel>();
        mTimer = &getFacilities().get_const<Timer>();

        if (!staticInitializationComplete) {
            staticInitializationComplete = true;
            mGlobalEnvironmentModel = mLocalEnvironmentModel->getGlobalEnvMod();
            mTraciAPI = getFacilities().get_const<traci::VehicleController>().getTraCI();
            mSimulationBoundary = traci::Boundary{mTraciAPI->simulation.getNetBoundary()};
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
        F2MDParameters::detectionParameters.maxDistanceFromRoad = par("maxDistanceFromRoad");
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
    }

    void MisbehaviorDetectionService::visualizeCamPosition(vanetza::asn1::Cam cam, const libsumo::TraCIColor &color,
                                                           const std::string &idPrefix) {
        static int counter = 0;
        traci::TraCIGeoPosition traciGeoPosition = {
                (double) cam->cam.camParameters.basicContainer.referencePosition.longitude / 10000000.0,
                (double) cam->cam.camParameters.basicContainer.referencePosition.latitude / 10000000.0};
        traci::TraCIPosition traciPosition = mVehicleController->getTraCI()->convert2D(traciGeoPosition);
        std::string poiId = std::to_string(cam->header.stationID);
        poiId += idPrefix;
        poiId += "_CAM_";
        poiId += std::to_string(cam->header.messageID);
        poiId += "-";
        poiId += std::to_string(cam->cam.generationDeltaTime);
        poiId += "-";
        poiId += std::to_string(counter++);
        mTraciAPI->poi.add(poiId, traciPosition.x, traciPosition.y, color,
                           poiId, 5, "", 0,
                           0, 0);
        activePoIs.push_back(poiId);
        if (activePoIs.size() > F2MDParameters::miscParameters.CamLocationVisualizerMaxLength) {
            mTraciAPI->poi.remove(activePoIs.front());
            activePoIs.pop_front();
        }
//        int alphaStep = 185 / F2MDParameters::miscParameters.CamLocationVisualizerMaxLength;
//        int currentAlpha = 80;
//        for(const auto& poi : activePoIs){
//            traciPoiScope->setColor(poi,color);
//            currentAlpha += alphaStep;
//        }
    }


    boost::geometry::strategy::transform::matrix_transformer<double, 2, 2>
    transformVehicle(double length, double width, const Position &pos, Angle alpha) {
        using namespace boost::geometry::strategy::transform;

        // scale square to vehicle dimensions
        scale_transformer<double, 2, 2> scaling(length, width);
        // rotate into driving direction
        rotate_transformer<boost::geometry::radian, double, 2, 2> rotation(alpha.radian());
        // move to given front bumper position
        translate_transformer<double, 2, 2> translation(pos.x.value(), pos.y.value());

        return matrix_transformer<double, 2, 2>{translation.matrix() * rotation.matrix() * scaling.matrix()};
    }

    std::vector<Position> MisbehaviorDetectionService::getVehicleOutline() {
        Angle heading = -1.0 * (mVehicleDataProvider->heading() -
                                0.5 * boost::math::double_constants::pi * boost::units::si::radian);
        auto transformationMatrix = transformVehicle(mVehicleController->getVehicleType().getLength().value(),
                                                     mVehicleController->getVehicleType().getWidth().value(),
                                                     mVehicleDataProvider->position(),
                                                     heading);
        std::vector<Position> squareOutline = {
                Position(0.0, 0.5), // front left
                Position(0.0, -0.5), // front right
                Position(-1.0, -0.5), // back right
                Position(-1.0, 0.5) // back left
        };
        std::vector<Position> vehicleOutline;
        boost::geometry::transform(squareOutline, vehicleOutline, transformationMatrix);
        return vehicleOutline;

    }


    void MisbehaviorDetectionService::receiveSignal(cComponent *source, simsignal_t signal, cObject *c_obj, cObject *) {
        Enter_Method("receiveSignal");
        if (signal == scSignalCamReceived) {
//            return;
            auto *ca = dynamic_cast<CaObject *>(c_obj);
            vanetza::asn1::Cam message = ca->asn1();
            uint32_t senderStationId = message->header.stationID;

            misbehaviorTypes::MisbehaviorTypes senderMisbehaviorType = getMisbehaviorTypeOfStationId(senderStationId);
            if (senderMisbehaviorType == misbehaviorTypes::LocalAttacker) {
                std::vector<Position> vehicleOutline = getVehicleOutline();
                std::cout << mVehicleDataProvider->getStationId() << " <-- " << senderStationId << ": "
                          << message->cam.generationDeltaTime << std::endl;

                auto &allObjects = mLocalEnvironmentModel->allObjects();
                TrackedObjectsFilterRange envModObjects = filterBySensorCategory(allObjects, "CA");
                if (detectedSenders.find(senderStationId) == detectedSenders.end()) {
                    detectedSenders[senderStationId] = new DetectedSender(mTraciAPI, mGlobalEnvironmentModel,
                                                                          &F2MDParameters::detectionParameters,
                                                                          message);
                }
                CheckResult *result = detectedSenders[senderStationId]->addAndCheckCam(message,
                                                                                       mVehicleDataProvider,
                                                                                       vehicleOutline,
                                                                                       envModObjects);

                std::cout << result->toString(0.5) << std::endl;
                vanetza::asn1::MisbehaviorReport misbehaviorReport =  createMisbehaviorReport("123",message);

                MisbehaviorReportObject obj(std::move(misbehaviorReport));
                emit(scSignalMisbehaviorAuthorityNewReport, &obj);

//            if (senderMisbehaviorType == misbehaviorTypes::LocalAttacker) {
                EV_INFO << "Received manipulated CAM!";
                EV_INFO << "Received DoS " << simTime().inUnit(SimTimeUnit::SIMTIME_MS) << " delta:  "
                        << message->cam.generationDeltaTime << "\n";
                vanetza::ByteBuffer byteBuffer = message.encode();
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
//            } else if (senderMisbehaviorType == misbehaviorTypes::Benign) {
//                EV_INFO << "Received benign CAM!";
//            } else {
//                EV_INFO << "Received weird misbehaviorType";
//            }

            }
        }

    }

    vanetza::asn1::MisbehaviorReport MisbehaviorDetectionService::createMisbehaviorReport(const std::string &reportId,const vanetza::asn1::Cam& cam) {
        vanetza::asn1::MisbehaviorReport misbehaviorReport;

//        misbehaviorReport->version = 1;
        misbehaviorReport->version = cam->header.stationID;
        ReportMetadataContainer_t &reportMetadataContainer = misbehaviorReport->reportMetadataContainer;
        assert(asn_long2INTEGER(&reportMetadataContainer.generationTime,
                                countTaiMilliseconds(mTimer->getCurrentTime())) == 0);
        OCTET_STRING_fromBuf(&reportMetadataContainer.reportID, reportId.c_str(), (int) strlen(reportId.c_str()));
//        const char *bla = reportId.c_str();
//        std::cout << bla << std::endl;
//        bla = (char*) &reportMetadataContainer.reportID.buf;
//        std::cout << bla << std::endl;
        ReportContainer &reportContainer = misbehaviorReport->reportContainer;
        reportContainer.reportedMessageContainer.present = ReportedMessageContainer_PR_certificateIncludedContainer;
        EtsiTs103097Data_t reportedMessage = reportContainer.reportedMessageContainer.choice.certificateIncludedContainer.reportedMessage;
        reportedMessage.protocolVersion = 3;
        Ieee1609Dot2Content_t ieee1609Dot2Content;
        reportedMessage.content = &ieee1609Dot2Content;
        ieee1609Dot2Content.present = Ieee1609Dot2Content_PR_unsecuredData;
        vanetza::ByteBuffer byteBuffer = cam.encode();
        std::string encodedCam(byteBuffer.begin(), byteBuffer.end());
//        OCTET_STRING_fromBuf(&ieee1609Dot2Content.choice.unsecuredData, encodedCam.c_str(), (int) strlen(encodedCam.c_str()));

        reportContainer.misbehaviorTypeContainer.present = MisbehaviorTypeContainer_PR_semanticDetection;
        SemanticDetection_t semanticDetection= reportContainer.misbehaviorTypeContainer.choice.semanticDetection;
        semanticDetection.present = SemanticDetection_PR_semanticDetectionReferenceCAM;
        semanticDetection.choice.semanticDetectionReferenceCAM.detectionLevelCAM = 3;
        int semanticDetectionErrorCodeCAM = 0;
        semanticDetectionErrorCodeCAM |= 1; //set bit for wrong referencePosition
        std::string encoded = std::bitset<24>(semanticDetectionErrorCodeCAM).to_string();
        OCTET_STRING_fromBuf(&semanticDetection.choice.semanticDetectionReferenceCAM.semanticDetectionErrorCodeCAM,encoded.c_str(),(int) strlen(encoded.c_str()));

        return misbehaviorReport;
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
