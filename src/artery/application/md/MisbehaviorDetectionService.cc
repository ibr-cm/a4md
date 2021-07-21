
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

        fusionApplication = new ThresholdFusion(0.5);
    }

    void MisbehaviorDetectionService::initializeParameters() {

        F2MDParameters::detectionParameters.checkType = par("checkType");
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
        F2MDParameters::detectionParameters.maxIntersectionDeltaTime = par("maxIntersectionDeltaTime");

        F2MDParameters::detectionParameters.maxKalmanTime = par("maxKalmanTime");
        F2MDParameters::detectionParameters.kalmanMinPosRange = par("kalmanMinPosRange");
        F2MDParameters::detectionParameters.kalmanMinSpeedRange = par("kalmanMinSpeedRange");
        F2MDParameters::detectionParameters.kalmanMinHeadingRange = par("kalmanMinHeadingRange");
        F2MDParameters::detectionParameters.kalmanPosRange = par("kalmanPosRange");
        F2MDParameters::detectionParameters.kalmanSpeedRange = par("kalmanSpeedRange");

        F2MDParameters::reportParameters.evidenceContainerMaxCamCount = par("evidenceContainerMaxCamCount");

    }


    void MisbehaviorDetectionService::trigger() {
        Enter_Method("trigger");
    }

    void MisbehaviorDetectionService::indicate(const vanetza::btp::DataIndication &ind, cPacket *packet,
                                               const NetworkInterface &net) {
        Enter_Method("indicate");
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
        std::string poiId = {
                std::to_string(cam->header.stationID) + idPrefix + "_CAM_" + std::to_string(cam->header.messageID) +
                "-" + std::to_string(cam->cam.generationDeltaTime) + "-" + std::to_string(counter++)};
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


    std::vector<vanetza::asn1::Cam *>
    MisbehaviorDetectionService::getSurroundingCamObjects(StationID_t senderStationId) {
        std::vector<vanetza::asn1::Cam *> surroundingCamObjects;
        for (auto it : detectedSenders) {
            auto detectedSender = it.second;
            if (detectedSender->getStationId() != senderStationId) {
                vanetza::asn1::Cam &latestCam = detectedSender->getResults().back()->cam;
                uint16_t oldTime = latestCam->cam.generationDeltaTime;
                uint16_t currentTime = countTaiMilliseconds(mTimer->getCurrentTime());
                if ((uint16_t) (currentTime - oldTime) <
                    (long) (F2MDParameters::detectionParameters.maxCamFrequency * 1000)) {
                    surroundingCamObjects.emplace_back(&latestCam);
                }
            }
        }
        return surroundingCamObjects;
    }

    std::vector<std::bitset<16>> MisbehaviorDetectionService::checkCam(const vanetza::asn1::Cam &message) {

        uint32_t senderStationId = message->header.stationID;
        std::vector<Position> mVehicleOutline = getVehicleOutline(mVehicleDataProvider, mVehicleController);

        std::vector<vanetza::asn1::Cam *> surroundingCamObjects = getSurroundingCamObjects(
                senderStationId);
        if (detectedSenders.find(senderStationId) == detectedSenders.end()) {
            detectedSenders[senderStationId] = new DetectedSender(mTraciAPI, mGlobalEnvironmentModel,
                                                                  &F2MDParameters::detectionParameters,
                                                                  message);
        }
        CheckResult *result = detectedSenders[senderStationId]->addAndCheckCam(message,
                                                                               mVehicleDataProvider,
                                                                               mVehicleOutline,
                                                                               surroundingCamObjects);

//        std::cout << result->toString(0.5) << std::endl;
        std::vector<std::bitset<16>> detectionLevelErrorCodes = fusionApplication->checkForReport(*result);
        return detectionLevelErrorCodes;
    }

    void MisbehaviorDetectionService::sendLevel1Report(const std::string &reportId, const std::string *relatedReportId,
                                                       const vanetza::asn1::Cam *reportedMessage,
                                                       std::bitset<16> semanticDetectionErrorCodeCAM) {
        DetectionReferenceCAM_t semanticDetectionReferenceCam = fillSemanticDetectionReferenceCam(
                detectionLevels::Level1, semanticDetectionErrorCodeCAM);
        vanetza::asn1::MisbehaviorReport misbehaviorReport = createMisbehaviorReport(reportId, reportedMessage,
                                                                                     &semanticDetectionReferenceCam);
        if (relatedReportId != nullptr) {
            misbehaviorReport->reportMetadataContainer.relatedReportContainer = new RelatedReportContainer_t();
            fillRelatedReportContainer(misbehaviorReport->reportMetadataContainer.relatedReportContainer,
                                       *relatedReportId, 0);
        }

        MisbehaviorReportObject obj(std::move(misbehaviorReport));
        emit(scSignalMisbehaviorAuthorityNewReport, &obj);
    }

    void MisbehaviorDetectionService::sendLevel2Report(const std::string &reportId, const std::string *relatedReportId,
                                                       const vanetza::asn1::Cam *reportedMessage,
                                                       std::bitset<16> semanticDetectionErrorCodeCAM,
                                                       StationID_t senderStationId) {
        DetectionReferenceCAM_t semanticDetectionReferenceCam = fillSemanticDetectionReferenceCam(
                detectionLevels::Level2, semanticDetectionErrorCodeCAM);
        vanetza::asn1::MisbehaviorReport misbehaviorReport = createMisbehaviorReport(reportId, reportedMessage,
                                                                                     &semanticDetectionReferenceCam);
        if (relatedReportId != nullptr) {
            misbehaviorReport->reportMetadataContainer.relatedReportContainer = new RelatedReportContainer_t();
            fillRelatedReportContainer(misbehaviorReport->reportMetadataContainer.relatedReportContainer,
                                       *relatedReportId, 0);
        }
        std::vector<CheckResult *> results = detectedSenders[senderStationId]->getResults();
        if (results.size() > 1) {
            misbehaviorReport->reportContainer.evidenceContainer = new EvidenceContainer_t();
            auto *reportedMessageContainer = new MessageEvidenceContainer_t();
            misbehaviorReport->reportContainer.evidenceContainer->reportedMessageContainer = reportedMessageContainer;
            int limit = std::min((int) results.size() - 1,
                                 F2MDParameters::reportParameters.evidenceContainerMaxCamCount);
            for (int i = 1; i <= limit; i++) {
                auto *singleMessageContainer = new EtsiTs103097Data_t();
                singleMessageContainer->content = new Ieee1609Dot2Content_t();
                singleMessageContainer->content->present = Ieee1609Dot2Content_PR_unsecuredData;
                OCTET_STRING_fromBuf(&singleMessageContainer->content->choice.unsecuredData,
                                     (const char *) &results[i]->cam,
                                     (int) results[i]->cam.size());
                ASN_SEQUENCE_ADD(reportedMessageContainer, singleMessageContainer);
            }

        }

        MisbehaviorReportObject obj(std::move(misbehaviorReport));
        emit(scSignalMisbehaviorAuthorityNewReport, &obj);
    }

    void MisbehaviorDetectionService::sendLevel3Report(const std::string &reportId, const std::string *relatedReportId,
                                                       const vanetza::asn1::Cam *reportedMessage,
                                                       std::bitset<16> semanticDetectionErrorCodeCAM,
                                                       StationID_t senderStationId) {
        DetectionReferenceCAM_t semanticDetectionReferenceCam = fillSemanticDetectionReferenceCam(
                detectionLevels::Level3, semanticDetectionErrorCodeCAM);
        vanetza::asn1::MisbehaviorReport misbehaviorReport = createMisbehaviorReport(reportId, reportedMessage,
                                                                                     &semanticDetectionReferenceCam);
        if (relatedReportId != nullptr) {
            misbehaviorReport->reportMetadataContainer.relatedReportContainer = new RelatedReportContainer_t();
            fillRelatedReportContainer(misbehaviorReport->reportMetadataContainer.relatedReportContainer,
                                       *relatedReportId, 0);
        }

        auto *evidenceContainer = new EvidenceContainer_t();
        misbehaviorReport->reportContainer.evidenceContainer = evidenceContainer;
        std::vector<Position> senderOutline = getVehicleOutline(*reportedMessage, mSimulationBoundary,
                                                                mTraciAPI);
        std::vector<StationID_t> stationIdsWithOverlap;
        if (boost::geometry::intersects(senderOutline, getVehicleOutline(mVehicleDataProvider,
                                                                         mVehicleController))) {
            auto *senderInfoContainer = new SenderInfoContainer_t();
            evidenceContainer->senderInfoContainer = senderInfoContainer;
            fillSenderInfoContainer(*senderInfoContainer);
        }
        for (auto cam : getSurroundingCamObjects(senderStationId)) {
            std::vector<Position> outline = getVehicleOutline((*cam), mSimulationBoundary,
                                                              mTraciAPI);
            if (boost::geometry::intersects(senderOutline, outline)) {
                stationIdsWithOverlap.emplace_back((*cam)->header.stationID);
            }
        }
        if (!stationIdsWithOverlap.empty()) {
            auto *neighbourMessageContainer = new MessageEvidenceContainer_t();
            evidenceContainer->neighbourMessageContainer = neighbourMessageContainer;
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
        }

        MisbehaviorReportObject obj(std::move(misbehaviorReport));
        emit(scSignalMisbehaviorAuthorityNewReport, &obj);
    }

    std::string MisbehaviorDetectionService::generateReportId(StationID_t senderStationId) {
        return {std::to_string(senderStationId) + "_" +
                std::to_string(countTaiMilliseconds(mTimer->getCurrentTime())) + "_" +
                std::to_string(intrand(UINT32_MAX))};
    }

    void MisbehaviorDetectionService::receiveSignal(cComponent *source, simsignal_t signal, cObject *c_obj, cObject *) {
        Enter_Method("receiveSignal");
        if (signal == scSignalCamReceived) {
            auto *ca = dynamic_cast<CaObject *>(c_obj);
            vanetza::asn1::Cam message = ca->asn1();
            uint32_t senderStationId = message->header.stationID;
            misbehaviorTypes::MisbehaviorTypes senderMisbehaviorType = getMisbehaviorTypeOfStationId(
                    senderStationId);
            if (senderMisbehaviorType == misbehaviorTypes::LocalAttacker && senderStationId == 2971333630) {
                std::cout << std::endl << mVehicleDataProvider->getStationId() << " <-- " << senderStationId << ": "
                          << message->cam.generationDeltaTime << std::endl;

                std::vector<std::bitset<16>> detectionLevelErrorCodes = checkCam(message);
                bool messageIsReported = false;
                DetectedSender &detectedSender = *detectedSenders[senderStationId];
                std::shared_ptr<std::string> relatedReportId = nullptr;
                if (detectedSender.hasBeenReported()) {
                    relatedReportId = std::make_shared<std::string>(detectedSender.getPreviousReportId());
                }

                if (detectionLevelErrorCodes[detectionLevels::Level1] != 0) {
                    std::shared_ptr<std::string> reportId =
                            std::make_shared<std::string>(generateReportId(senderStationId));
                    sendLevel1Report(*reportId, relatedReportId.get(),
                                     messageIsReported ? &message : nullptr,
                                     detectionLevelErrorCodes[detectionLevels::Level1]);
                    if (relatedReportId == nullptr) {
                        relatedReportId = reportId;
                    }
                    messageIsReported = true;
                }
                if (detectionLevelErrorCodes[detectionLevels::Level2] != 0) {
                    std::shared_ptr<std::string> reportId =
                            std::make_shared<std::string>(generateReportId(senderStationId));
                    sendLevel2Report(*reportId, relatedReportId.get(),
                                     messageIsReported ? &message : nullptr,
                                     detectionLevelErrorCodes[detectionLevels::Level2], senderStationId);
                    if (relatedReportId == nullptr) {
                        relatedReportId = reportId;
                    }
                    messageIsReported = true;
                }
                if (detectionLevelErrorCodes[detectionLevels::Level3] != 0) {
                    std::shared_ptr<std::string> reportId =
                            std::make_shared<std::string>(generateReportId(senderStationId));
                    sendLevel3Report(*reportId, relatedReportId.get(),
                                     messageIsReported ? &message : nullptr,
                                     detectionLevelErrorCodes[detectionLevels::Level3], senderStationId);
                    if (relatedReportId == nullptr) {
                        relatedReportId = reportId;
                    }
                    messageIsReported = true;
                }
                if (detectionLevelErrorCodes[detectionLevels::Level4] != 0) {

                }
                if (detectedSender.getPreviousReportId().empty()) {
                    detectedSender.setReportId(*relatedReportId);
                }
            }
        }

    }

    vanetza::asn1::MisbehaviorReport
    MisbehaviorDetectionService::createMisbehaviorReport(const string &reportId,
                                                         const vanetza::asn1::Cam *cam,
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
        if (cam != nullptr) {
            auto *ieee1609Dot2Content = new Ieee1609Dot2Content_t();
            reportedMessage.content = ieee1609Dot2Content;
            ieee1609Dot2Content->present = Ieee1609Dot2Content_PR_unsecuredData;
            Opaque_t &unsecuredData = ieee1609Dot2Content->choice.unsecuredData;
            OCTET_STRING_fromBuf(&unsecuredData, (const char *) cam, (int) cam->size());
        }

        reportContainer.misbehaviorTypeContainer.present = MisbehaviorTypeContainer_PR_semanticDetection;
        SemanticDetection_t &semanticDetection = reportContainer.misbehaviorTypeContainer.choice.semanticDetection;
        semanticDetection.present = SemanticDetection_PR_semanticDetectionReferenceCAM;
        semanticDetection.choice.semanticDetectionReferenceCAM = *semanticDetectionReferenceCam;

        std::string error;
        if (!misbehaviorReport.validate(error)) {
            throw cRuntimeError("Invalid Misbehavior Report: %s", error.c_str());
        }
        return misbehaviorReport;
    }

    void MisbehaviorDetectionService::fillSenderInfoContainer(SenderInfoContainer_t &senderInfoContainer) {
        senderInfoContainer.stationType = static_cast<StationType_t>(mVehicleDataProvider->getStationType());
        senderInfoContainer.referencePosition = mVehicleDataProvider->approximateReferencePosition();
        senderInfoContainer.heading = mVehicleDataProvider->approximateHeading();
        senderInfoContainer.speed = mVehicleDataProvider->approximateSpeed();
        senderInfoContainer.driveDirection = mVehicleDataProvider->speed().value() >= 0.0 ?
                                             DriveDirection_forward : DriveDirection_backward;
        senderInfoContainer.vehicleLength.vehicleLengthValue = (long) (
                mVehicleController->getLength().value() * 10);
        senderInfoContainer.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_noTrailerPresent;
        senderInfoContainer.vehicleWidth = (long) (mVehicleController->getWidth().value() *
                                                   10);
        senderInfoContainer.longitudinalAcceleration = mVehicleDataProvider->approximateAcceleration();

        senderInfoContainer.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
        senderInfoContainer.curvature.curvatureValue = (long) (
                abs(mVehicleDataProvider->curvature() / vanetza::units::reciprocal_metre) *
                10000.0);
        if (senderInfoContainer.curvature.curvatureValue >= 1023) {
            senderInfoContainer.curvature.curvatureValue = 1023;
        }

        senderInfoContainer.yawRate.yawRateValue = (long)
                ((double) round(mVehicleDataProvider->yaw_rate(), degree_per_second) *
                 YawRateValue_degSec_000_01ToLeft * 100.0);
        if (senderInfoContainer.yawRate.yawRateValue < -32766 ||
            senderInfoContainer.yawRate.yawRateValue > 32766) {
            senderInfoContainer.yawRate.yawRateValue = YawRateValue_unavailable;
        }
        senderInfoContainer.yawRate.yawRateConfidence = YawRateConfidence_unavailable;
    }

    DetectionReferenceCAM_t MisbehaviorDetectionService::fillSemanticDetectionReferenceCam(
            detectionLevels::DetectionLevels detectionLevelCam,
            std::bitset<16> semanticDetectionErrorCodeCAM) {
        auto *semanticDetectionReferenceCam = new DetectionReferenceCAM_t();
        semanticDetectionReferenceCam->detectionLevelCAM = detectionLevelCam + 1;
        std::string encoded = semanticDetectionErrorCodeCAM.to_string();
        OCTET_STRING_fromBuf(&semanticDetectionReferenceCam->semanticDetectionErrorCodeCAM, encoded.c_str(),
                             (int) strlen(encoded.c_str()));
        return *semanticDetectionReferenceCam;
    }

    void MisbehaviorDetectionService::fillRelatedReportContainer(RelatedReportContainer_t *relatedReportContainer,
                                                                 const std::string &relatedReportId,
                                                                 int omittedReportsNumber) {
//        relatedReportContainer = new RelatedReportContainer_t();
        OCTET_STRING_fromBuf(&relatedReportContainer->relatedReportID, relatedReportId.c_str(),
                             (int) strlen(relatedReportId.c_str()));
        relatedReportContainer->omittedReportsNumber = omittedReportsNumber;
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
