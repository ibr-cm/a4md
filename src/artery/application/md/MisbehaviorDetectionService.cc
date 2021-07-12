
#include "artery/application/md/MisbehaviorDetectionService.h"
#include "artery/application/md/util/HelperFunctions.h"
#include "artery/application/md/fusion/ThresholdFusion.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
#include <vanetza/asn1/cam.hpp>
#include "artery/application/CaService.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/md/util/MisbehaviorTypes.h"
#include <inet/common/ModuleAccess.h>
#include "artery/traci/Cast.h"
#include "artery/application/md/MisbehaviorCaService.h"
#include "MisbehaviorReportObject.h"
#include <bitset>
#include <boost/units/systems/cgs.hpp>
#include <boost/units/make_scaled_unit.hpp>

namespace artery {
    using namespace omnetpp;

    Define_Module(MisbehaviorDetectionService);

    namespace {
        auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;

        template<typename T, typename U>
        long round(const boost::units::quantity<T> &q, const U &u) {
            boost::units::quantity<U> v{q};
            return std::round(v.value());
        }
    }

    static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
    static const simsignal_t scSignalMisbehaviorAuthorityNewReport = cComponent::registerSignal(
            "misbehaviorAuthority.newReport");

    bool MisbehaviorDetectionService::staticInitializationComplete = false;
    std::map<uint32_t, misbehaviorTypes::MisbehaviorTypes> MisbehaviorDetectionService::mStationIdMisbehaviorTypeMap;
    std::shared_ptr<const traci::API> MisbehaviorDetectionService::mTraciAPI;
    GlobalEnvironmentModel *MisbehaviorDetectionService::mGlobalEnvironmentModel;


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
//            mSimulationBoundary = traci::Boundary{mTraciAPI->simulation.getNetBoundary()};
            initializeParameters();
        }

        fusionApplication = new ThresholdFusion(0.5);
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


    void MisbehaviorDetectionService::receiveSignal(cComponent *source, simsignal_t signal, cObject *c_obj, cObject *) {
        Enter_Method("receiveSignal");
        if (signal == scSignalCamReceived) {
            auto *ca = dynamic_cast<CaObject *>(c_obj);
            vanetza::asn1::Cam message = ca->asn1();
            uint32_t senderStationId = message->header.stationID;

            misbehaviorTypes::MisbehaviorTypes senderMisbehaviorType = getMisbehaviorTypeOfStationId(senderStationId);
            if (senderMisbehaviorType == misbehaviorTypes::LocalAttacker) {
                std::vector<Position> mVehicleOutline = getVehicleOutline(mVehicleDataProvider, mVehicleController);

                auto &allObjects = mLocalEnvironmentModel->allObjects();
                TrackedObjectsFilterRange envModObjects = filterBySensorCategory(allObjects, "CA");
                if (detectedSenders.find(senderStationId) == detectedSenders.end()) {
                    detectedSenders[senderStationId] = new DetectedSender(mTraciAPI, mGlobalEnvironmentModel,
                                                                          &F2MDParameters::detectionParameters,
                                                                          message);
                }
                CheckResult *result = detectedSenders[senderStationId]->addAndCheckCam(message,
                                                                                       mVehicleDataProvider,
                                                                                       mVehicleOutline,
                                                                                       envModObjects);

                if (result->intersection == 0) {
                    std::cout << std::endl << mVehicleDataProvider->getStationId() << " <-- " << senderStationId << ": "
                              << message->cam.generationDeltaTime << std::endl;
                    std::cout << result->toString(0.5) << std::endl;
                    DetectionReferenceCAM_t *semanticDetectionReferenceCam = fusionApplication->checkForReport(*result);
                    if (semanticDetectionReferenceCam->detectionLevelCAM != 0) {
                        std::string reportId = {std::to_string(message->header.stationID) + "_" +
                                                std::to_string(simTime().inUnit(SimTimeUnit::SIMTIME_MS))};
                        vanetza::asn1::MisbehaviorReport misbehaviorReport = createMisbehaviorReport(reportId, message,
                                                                                                     semanticDetectionReferenceCam);

                        switch (semanticDetectionReferenceCam->detectionLevelCAM) {
                            case 1: {
                                break;
                            }
                            case 2: {
                                std::vector<CheckResult *> results = detectedSenders[senderStationId]->getResults();
                                if (results.size() > 1) {
                                    misbehaviorReport->reportContainer.evidenceContainer = new EvidenceContainer_t();
                                    auto *reportedMessageContainer = new MessageEvidenceContainer_t();
                                    misbehaviorReport->reportContainer.evidenceContainer->reportedMessageContainer = reportedMessageContainer;
                                    std::cout << results.size() - 1 << std::endl;
                                    for (auto it = results.rbegin(); it != results.rend(); it++) {
                                        if (it == results.rbegin()) {
                                            continue;
                                        }
                                        auto *singleMessageContainer = new EtsiTs103097Data_t();
                                        singleMessageContainer->content = new Ieee1609Dot2Content_t();
                                        singleMessageContainer->content->present = Ieee1609Dot2Content_PR_unsecuredData;
                                        OCTET_STRING_fromBuf(&singleMessageContainer->content->choice.unsecuredData,
                                                             (const char *) &(*it)->cam,
                                                             (int) (*it)->cam.size());
                                        ASN_SEQUENCE_ADD(reportedMessageContainer, singleMessageContainer);
                                    }
                                }
                                break;
                            }
                            case 3: {
                                std::vector<Position> senderOutline;
                                std::vector<StationID_t> stationIdsWithOverlap;
                                for (const auto &object : envModObjects) {
                                    std::weak_ptr<EnvironmentModelObject> obj_ptr = object.first;
                                    if (obj_ptr.expired()) {
                                        continue;
                                    }
                                    std::shared_ptr<EnvironmentModelObject> envModObject = obj_ptr.lock();
                                    if (envModObject->getVehicleData().getStationId() == senderStationId) {
                                        senderOutline = envModObject->getOutline();
                                        break;
                                    }
                                }
                                if (senderOutline.empty()) {
                                    throw cRuntimeError("senderOutline empty");
                                } else {
                                    if (boost::geometry::intersects(senderOutline, mVehicleOutline)) {
                                        return;
                                    } else {
                                        for (const auto &object : envModObjects) {
                                            std::weak_ptr<EnvironmentModelObject> obj_ptr = object.first;
                                            if (obj_ptr.expired()) {
                                                continue;
                                            } else {
                                                std::shared_ptr<EnvironmentModelObject> envModObject = obj_ptr.lock();
                                                if (envModObject->getVehicleData().getStationId() != senderStationId &&
                                                    boost::geometry::intersects(senderOutline,
                                                                                envModObject->getOutline())) {
                                                    stationIdsWithOverlap.emplace_back(
                                                            envModObject->getVehicleData().getStationId());
                                                }
                                            }
                                        }
                                    }
                                }
                                misbehaviorReport->reportContainer.evidenceContainer = new EvidenceContainer_t();
                                auto *neighbourMessageContainer = new MessageEvidenceContainer_t();
                                misbehaviorReport->reportContainer.evidenceContainer->neighbourMessageContainer = neighbourMessageContainer;
                                for (auto stationId : stationIdsWithOverlap) {
                                    auto *singleMessageContainer = new EtsiTs103097Data_t();
                                    singleMessageContainer->content = new Ieee1609Dot2Content_t();
                                    singleMessageContainer->content->present = Ieee1609Dot2Content_PR_unsecuredData;
                                    std::vector<CheckResult *> results = detectedSenders[stationId]->getResults();
                                    vanetza::asn1::Cam &cam = (*results.rbegin())->cam;
                                    OCTET_STRING_fromBuf(&singleMessageContainer->content->choice.unsecuredData,
                                                         (const char *) &cam,
                                                         (int) cam.size());
                                    ASN_SEQUENCE_ADD(neighbourMessageContainer, singleMessageContainer);
                                }
                                break;
                            }
                            case 4: {
                                auto *senderInfoContainer = new SenderInfoContainer_t();
                                senderInfoContainer->stationType = static_cast<StationType_t>(mVehicleDataProvider->getStationType());
                                senderInfoContainer->referencePosition = mVehicleDataProvider->approximateReferencePosition();
                                senderInfoContainer->heading = mVehicleDataProvider->approximateHeading();
                                senderInfoContainer->speed = mVehicleDataProvider->approximateSpeed();
                                senderInfoContainer->driveDirection = mVehicleDataProvider->speed().value() >= 0.0 ?
                                                                      DriveDirection_forward : DriveDirection_backward;
                                senderInfoContainer->vehicleLength.vehicleLengthValue = (long) (
                                        mVehicleController->getLength().value() * 10);
                                senderInfoContainer->vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_noTrailerPresent;
                                senderInfoContainer->vehicleWidth = (long) (mVehicleController->getWidth().value() *
                                                                            10);
                                senderInfoContainer->longitudinalAcceleration = mVehicleDataProvider->approximateAcceleration();

                                senderInfoContainer->curvature.curvatureConfidence = CurvatureConfidence_unavailable;
                                senderInfoContainer->curvature.curvatureValue = (long) (
                                        abs(mVehicleDataProvider->curvature() / vanetza::units::reciprocal_metre) *
                                        10000.0);
                                if (senderInfoContainer->curvature.curvatureValue >= 1023) {
                                    senderInfoContainer->curvature.curvatureValue = 1023;
                                }

                                senderInfoContainer->yawRate.yawRateValue = (long)
                                        ((double) round(mVehicleDataProvider->yaw_rate(), degree_per_second) *
                                         YawRateValue_degSec_000_01ToLeft * 100.0);
                                if (senderInfoContainer->yawRate.yawRateValue < -32766 ||
                                    senderInfoContainer->yawRate.yawRateValue > 32766) {
                                    senderInfoContainer->yawRate.yawRateValue = YawRateValue_unavailable;
                                }
                                senderInfoContainer->yawRate.yawRateConfidence = YawRateConfidence_unavailable;
                                auto *evidenceContainer = new EvidenceContainer_t();
                                evidenceContainer->senderInfoContainer = senderInfoContainer;
                                misbehaviorReport->reportContainer.evidenceContainer = evidenceContainer;
                                break;
                            }
                        }
                        std::cout << "Sending Report..." << std::endl;
                        MisbehaviorReportObject obj(std::move(misbehaviorReport));
                        emit(scSignalMisbehaviorAuthorityNewReport, &obj);
                    }
                }
            }
        }

    }

    vanetza::asn1::MisbehaviorReport
    MisbehaviorDetectionService::createMisbehaviorReport(const std::string &reportId, const vanetza::asn1::Cam &cam,
                                                         DetectionReferenceCAM_t *semanticDetectionReferenceCam) {
        vanetza::asn1::MisbehaviorReport misbehaviorReport;

        misbehaviorReport->version = 1;
        ReportMetadataContainer_t &reportMetadataContainer = misbehaviorReport->reportMetadataContainer;
        assert(asn_long2INTEGER(&reportMetadataContainer.generationTime,
                                countTaiMilliseconds(mTimer->getCurrentTime())) == 0);

        OCTET_STRING_fromBuf(&reportMetadataContainer.reportID, reportId.c_str(), (int) strlen(reportId.c_str()));

        ReportContainer &reportContainer = misbehaviorReport->reportContainer;
        reportContainer.reportedMessageContainer.present = ReportedMessageContainer_PR_certificateIncludedContainer;
        EtsiTs103097Data_t &reportedMessage = reportContainer.reportedMessageContainer.choice.certificateIncludedContainer.reportedMessage;
        reportedMessage.protocolVersion = 3;
        auto *ieee1609Dot2Content = new Ieee1609Dot2Content_t();
        reportedMessage.content = ieee1609Dot2Content;
        ieee1609Dot2Content->present = Ieee1609Dot2Content_PR_unsecuredData;
        vanetza::ByteBuffer byteBuffer = cam.encode();
        std::string encodedCam(byteBuffer.begin(), byteBuffer.end());
        Opaque_t &unsecuredData = ieee1609Dot2Content->choice.unsecuredData;

        OCTET_STRING_fromBuf(&unsecuredData, (const char *) &cam, (int) cam.size());

        reportContainer.misbehaviorTypeContainer.present = MisbehaviorTypeContainer_PR_semanticDetection;
        SemanticDetection_t &semanticDetection = reportContainer.misbehaviorTypeContainer.choice.semanticDetection;
        semanticDetection.present = SemanticDetection_PR_semanticDetectionReferenceCAM;
        semanticDetection.choice.semanticDetectionReferenceCAM = *semanticDetectionReferenceCam;

        if (!misbehaviorReport.validate()) {
            std::cout << "failed validation" << std::endl;
        }
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

} // namespace artery
