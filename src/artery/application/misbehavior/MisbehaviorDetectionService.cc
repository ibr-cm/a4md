
#include "artery/application/misbehavior/MisbehaviorDetectionService.h"
#include "artery/application/misbehavior/util/HelperFunctions.h"
#include "artery/application/misbehavior/fusion/ThresholdFusion.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
#include <vanetza/asn1/cam.hpp>
#include "artery/application/CaService.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/misbehavior/util/MisbehaviorTypes.h"
#include <inet/common/ModuleAccess.h>
#include "artery/traci/Cast.h"
#include "artery/application/misbehavior/MisbehaviorCaService.h"
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
    static const simsignal_t scSignalCamSent = cComponent::registerSignal("CamSent");
    static const simsignal_t scSignalMisbehaviorAuthorityNewReport = cComponent::registerSignal(
            "misbehaviorAuthority.newReport");

    bool MisbehaviorDetectionService::staticInitializationComplete = false;
    std::shared_ptr<const traci::API> MisbehaviorDetectionService::mTraciAPI;
    GlobalEnvironmentModel *MisbehaviorDetectionService::mGlobalEnvironmentModel;
    traci::Boundary MisbehaviorDetectionService::mSimulationBoundary;


    MisbehaviorDetectionService::MisbehaviorDetectionService() {
    }

    MisbehaviorDetectionService::~MisbehaviorDetectionService() {
        while (!activePoIs.empty()) {
            mTraciAPI->poi.remove(activePoIs.front());
            activePoIs.pop_front();
        }
    }

    void MisbehaviorDetectionService::initialize() {
        ItsG5BaseService::initialize();
        subscribe(scSignalCamReceived);
        subscribe(scSignalCamSent);
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


        mFusionApplication = new ThresholdFusion(F2MDParameters::detectionParameters.misbehaviorThreshold);
    }

    void MisbehaviorDetectionService::initializeParameters() {

        F2MDParameters::detectionParameters.checkType = par("checkType");
        F2MDParameters::detectionParameters.misbehaviorThreshold = par("misbehaviorThreshold");
        F2MDParameters::detectionParameters.resultArrayMaxSize = par("resultArrayMaxSize");
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
        F2MDParameters::reportParameters.omittedReportsCount = par("omittedReportsCount");
        F2MDParameters::reportParameters.omittedReportsCountPerErrorCode = par("omittedReportsCountPerErrorCode");
        F2MDParameters::reportParameters.broadcastReport = par("broadcastReport");

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

    void MisbehaviorDetectionService::receiveSignal(cComponent *source, simsignal_t signal, cObject *c_obj, cObject *) {
        Enter_Method("receiveSignal");
        if (signal == scSignalCamReceived) {
            auto *ca = dynamic_cast<CaObject *>(c_obj);
            vanetza::asn1::Cam message = ca->asn1();
            detectMisbehavior(message);
        } else if (signal == scSignalCamSent) {
            auto *ca = dynamic_cast<CaObject *>(c_obj);
            mLastSentCam = std::make_shared<vanetza::asn1::Cam>(ca->asn1());
        }
    }

    void MisbehaviorDetectionService::detectMisbehavior(vanetza::asn1::Cam &message) {
        uint32_t senderStationId = message->header.stationID;
//            std::cout << std::endl << mVehicleDataProvider->getStationId() << " <-- " << senderStationId << ": "
//                      << message->cam.generationDeltaTime << std::endl;
//

        std::vector<std::bitset<16>> detectionLevelErrorCodes = checkCam(message);
        auto *camPtr = &message;
        DetectedSender &detectedSender = *detectedSenders[senderStationId];
        std::string relatedReportId = detectedSender.getPreviousReportId();
        std::bitset<16> reportedErrorCodes = 0;

        for (detectionLevels::DetectionLevels detectionLevel : detectionLevels::DetectionLevelVector) {
            std::bitset<16> errorCode = detectionLevelErrorCodes[(int) detectionLevel];
            if (errorCode.any() && detectedSender.checkOmittedReportsLimit(errorCode)) {
                std::string reportId(generateReportId(senderStationId));
                vanetza::asn1::MisbehaviorReport misbehaviorReport =
                        createReport(detectionLevel, reportId, relatedReportId, camPtr,
                                     detectionLevelErrorCodes[(int) detectionLevel], detectedSender);
                fillMisbehaviorTypeContainer(misbehaviorReport->reportContainer.misbehaviorTypeContainer,
                                             detectionLevel, errorCode);
                if (relatedReportId.empty()) {
                    relatedReportId = reportId;
                } else {
                    fillRelatedReportContainer(
                            misbehaviorReport->reportMetadataContainer.relatedReportContainer,
                            relatedReportId, 0);
                }
                camPtr = nullptr;
                reportedErrorCodes |= errorCode;
                MisbehaviorReportObject obj(std::move(misbehaviorReport));
                emit(scSignalMisbehaviorAuthorityNewReport, &obj);
                relatedReportId = reportId;
                if (detectedSender.getPreviousReportId().empty()) {
                    detectedSender.setReportId(relatedReportId);
                }
            }
        }
        if (reportedErrorCodes.any()) {
            detectedSender.resetOmittedReports(reportedErrorCodes);
        } else {
            if (camPtr == nullptr) {
                std::cout << "omitted report" << std::endl;
            }
        }
        detectedSender.incrementOmittedReports(detectionLevelErrorCodes, reportedErrorCodes);
    }

    std::vector<std::bitset<16>> MisbehaviorDetectionService::checkCam(const vanetza::asn1::Cam &message) {

//        std::cout << "Result MisbehaviorDetectionService" << std::endl;
        uint32_t senderStationId = message->header.stationID;
        std::vector<Position> mVehicleOutline = getVehicleOutline(mVehicleDataProvider, mVehicleController);

        std::vector<std::shared_ptr<vanetza::asn1::Cam>> surroundingCamObjects = getSurroundingCamObjects(
                senderStationId);
        if (detectedSenders.find(senderStationId) == detectedSenders.end()) {
            detectedSenders[senderStationId] = new DetectedSender(mTraciAPI, mGlobalEnvironmentModel,
                                                                  &F2MDParameters::detectionParameters,
                                                                  mTimer, message);
        }
        std::shared_ptr<CheckResult> result = detectedSenders[senderStationId]->addAndCheckCam(message,
                                                                                               mVehicleDataProvider,
                                                                                               mVehicleOutline,
                                                                                               surroundingCamObjects);
//        std::cout << result->toString(0.5) << std::endl;
        std::vector<std::bitset<16>> detectionLevelErrorCodes = mFusionApplication->checkForReport(*result);
        return detectionLevelErrorCodes;
    }

    std::vector<std::shared_ptr<vanetza::asn1::Cam>>
    MisbehaviorDetectionService::getSurroundingCamObjects(StationID_t senderStationId) {
        std::vector<std::shared_ptr<vanetza::asn1::Cam>> surroundingCamObjects;
        for (auto it : detectedSenders) {
            auto detectedSender = it.second;
            if (detectedSender->getStationId() != senderStationId) {
                vanetza::asn1::Cam &latestCam = detectedSender->getResults().back()->cam;
                uint16_t oldTime = latestCam->cam.generationDeltaTime;
                uint16_t currentTime = countTaiMilliseconds(mTimer->getCurrentTime());
                if ((uint16_t) (currentTime - oldTime) <
                    (long) (F2MDParameters::detectionParameters.maxCamFrequency * 1000)) {
                    surroundingCamObjects.emplace_back(std::make_shared<vanetza::asn1::Cam>(latestCam));
                }
            }
        }
        return surroundingCamObjects;
    }

    vanetza::asn1::MisbehaviorReport
    MisbehaviorDetectionService::createLevel1Report(const std::string &reportId,
                                                    const vanetza::asn1::Cam *reportedMessage,
                                                    const bitset<16> &semanticDetectionErrorCodeCAM) {
        vanetza::asn1::MisbehaviorReport misbehaviorReport = createBasicMisbehaviorReport(reportId, reportedMessage);
        return misbehaviorReport;
    }

    vanetza::asn1::MisbehaviorReport
    MisbehaviorDetectionService::createLevel2Report(const std::string &reportId,
                                                    const vanetza::asn1::Cam *reportedMessage,
                                                    const bitset<16> &semanticDetectionErrorCodeCAM,
                                                    DetectedSender &detectedSender) {
        vanetza::asn1::MisbehaviorReport misbehaviorReport = createBasicMisbehaviorReport(reportId, reportedMessage);
        EvidenceContainer_t *&evidenceContainer = misbehaviorReport->reportContainer.evidenceContainer;
        misbehaviorReport->reportContainer.evidenceContainer = vanetza::asn1::allocate<EvidenceContainer_t>();
        std::list<std::shared_ptr<CheckResult>> resultsList = detectedSender.getResults();
        std::vector<std::shared_ptr<CheckResult>> results{resultsList.begin(), resultsList.end()};
        if (results.size() > 1) {
            MessageEvidenceContainer_t *&reportedMessageContainer = evidenceContainer->reportedMessageContainer;
            reportedMessageContainer = vanetza::asn1::allocate<MessageEvidenceContainer_t>();
            int limit =
                    std::min((int) results.size() - 1, F2MDParameters::reportParameters.evidenceContainerMaxCamCount);
//            for (int i = (int) results.size() - 2; i >= (int) results.size() - limit - 1; i--) {
            for (int i = (int) results.size() - limit - 1; i < results.size() - 1; i++) {
                auto *singleMessageContainer = vanetza::asn1::allocate<EtsiTs103097Data_t>();
                singleMessageContainer->content = vanetza::asn1::allocate<Ieee1609Dot2Content_t>();
                singleMessageContainer->content->present = Ieee1609Dot2Content_PR_unsecuredData;
                auto *encodedMessage = new vanetza::ByteBuffer(results[i]->cam.encode());
                OCTET_STRING_fromBuf(&singleMessageContainer->content->choice.unsecuredData,
                                     (const char *) encodedMessage->data(),
                                     (int) encodedMessage->size());
                ASN_SEQUENCE_ADD(reportedMessageContainer, singleMessageContainer);
            }

        }
        return misbehaviorReport;
    }

    vanetza::asn1::MisbehaviorReport
    MisbehaviorDetectionService::createLevel3Report(const std::string &reportId,
                                                    const vanetza::asn1::Cam *reportedMessage,
                                                    const bitset<16> &semanticDetectionErrorCodeCAM,
                                                    DetectedSender &detectedSender) {
        vanetza::asn1::MisbehaviorReport misbehaviorReport = createBasicMisbehaviorReport(reportId, reportedMessage);
        EvidenceContainer_t *&evidenceContainer = misbehaviorReport->reportContainer.evidenceContainer;
        evidenceContainer = vanetza::asn1::allocate<EvidenceContainer_t>();

        std::vector<Position> senderOutline = getVehicleOutline(detectedSender.getResults().back()->cam,
                                                                mSimulationBoundary,
                                                                mTraciAPI);
        std::vector<std::shared_ptr<vanetza::asn1::Cam>> overlappingCams;
        if (boost::geometry::intersects(senderOutline, getVehicleOutline(mVehicleDataProvider,
                                                                         mVehicleController))) {
            overlappingCams.emplace_back(mLastSentCam);
        }
        for (const auto &cam : getSurroundingCamObjects(detectedSender.getStationId())) {
            std::vector<Position> outline = getVehicleOutline((*cam), mSimulationBoundary,
                                                              mTraciAPI);
            if (boost::geometry::intersects(senderOutline, outline)) {
                overlappingCams.emplace_back(cam);
            }
        }
        if (!overlappingCams.empty()) {
            MessageEvidenceContainer_t *&neighbourMessageContainer = evidenceContainer->neighbourMessageContainer;
            neighbourMessageContainer = vanetza::asn1::allocate<MessageEvidenceContainer_t>();
            for (const auto &cam : overlappingCams) {
                auto *singleMessageContainer = vanetza::asn1::allocate<EtsiTs103097Data_t>();
                singleMessageContainer->content = vanetza::asn1::allocate<Ieee1609Dot2Content_t>();
                singleMessageContainer->content->present = Ieee1609Dot2Content_PR_unsecuredData;
                auto *encodedMessage = new vanetza::ByteBuffer(cam->encode());
                OCTET_STRING_fromBuf(&singleMessageContainer->content->choice.unsecuredData,
                                     (const char *) encodedMessage->data(),
                                     (int) encodedMessage->size());
                ASN_SEQUENCE_ADD(neighbourMessageContainer, singleMessageContainer);
            }
        }
        return misbehaviorReport;
    }

    vanetza::asn1::MisbehaviorReport
    MisbehaviorDetectionService::createLevel4Report(const std::string &reportId,
                                                    const vanetza::asn1::Cam *reportedMessage,
                                                    const bitset<16> &semanticDetectionErrorCodeCAM,
                                                    DetectedSender &detectedSender) {
        vanetza::asn1::MisbehaviorReport misbehaviorReport = createBasicMisbehaviorReport(reportId, reportedMessage);
        EvidenceContainer_t *&evidenceContainer = misbehaviorReport->reportContainer.evidenceContainer;
        evidenceContainer = vanetza::asn1::allocate<EvidenceContainer_t>();
        SenderInfoContainer_t *&senderInfoContainer = evidenceContainer->senderInfoContainer;
        senderInfoContainer = vanetza::asn1::allocate<SenderInfoContainer_t>();
        fillSenderInfoContainer(*senderInfoContainer);
        return misbehaviorReport;
    }

    vanetza::asn1::MisbehaviorReport
    MisbehaviorDetectionService::createReport(detectionLevels::DetectionLevels detectionLevel,
                                              const std::string &reportId, std::string &relatedReportId,
                                              const vanetza::asn1::Cam *reportedMessage, std::bitset<16> errorCode,
                                              DetectedSender &detectedSender) {
        vanetza::asn1::MisbehaviorReport misbehaviorReport;
        switch (detectionLevel) {
            case detectionLevels::Level1:
                misbehaviorReport = createLevel1Report(reportId, reportedMessage, errorCode);
                break;
            case detectionLevels::Level2:
                misbehaviorReport = createLevel2Report(reportId, reportedMessage, errorCode,
                                                       detectedSender);
                break;
            case detectionLevels::Level3:
                misbehaviorReport = createLevel3Report(reportId, reportedMessage, errorCode,
                                                       detectedSender);
                break;
            case detectionLevels::Level4:
                misbehaviorReport = createLevel4Report(reportId, reportedMessage, errorCode,
                                                       detectedSender);
                break;
            default:
                break;
        }
        return misbehaviorReport;

    }

    vanetza::asn1::MisbehaviorReport
    MisbehaviorDetectionService::createBasicMisbehaviorReport(const string &reportId, const vanetza::asn1::Cam *cam) {
        vanetza::asn1::MisbehaviorReport misbehaviorReport;

        misbehaviorReport->version = 1;
        ReportMetadataContainer_t &reportMetadataContainer = misbehaviorReport->reportMetadataContainer;
        uint64_t currentTime = countTaiMilliseconds(mTimer->getTimeFor(simTime()));
        assert(asn_long2INTEGER(&reportMetadataContainer.generationTime, currentTime) == 0);
        OCTET_STRING_fromBuf(&reportMetadataContainer.reportID, reportId.c_str(), (int) strlen(reportId.c_str()));

        ReportContainer &reportContainer = misbehaviorReport->reportContainer;
        reportContainer.reportedMessageContainer.present = ReportedMessageContainer_PR_certificateIncludedContainer;
        EtsiTs103097Data_t &reportedMessage =
                reportContainer.reportedMessageContainer.choice.certificateIncludedContainer.reportedMessage;
        reportedMessage.protocolVersion = 3;
        if (cam != nullptr) {
            reportedMessage.content = vanetza::asn1::allocate<Ieee1609Dot2Content_t>();
            reportedMessage.content->present = Ieee1609Dot2Content_PR_unsecuredData;
            auto *encodedMessage = new vanetza::ByteBuffer(cam->encode());
            OCTET_STRING_fromBuf(&reportedMessage.content->choice.unsecuredData,
                                 reinterpret_cast<const char *>(encodedMessage->data()), (int) encodedMessage->size());
        }

        std::string error;
        if (!misbehaviorReport.validate(error)) {
            throw cRuntimeError("Invalid Misbehavior Report: %s", error.c_str());
        }
        return misbehaviorReport;
    }

    std::string MisbehaviorDetectionService::generateReportId(const StationID_t &senderStationId) {
        return {std::to_string(senderStationId) + "_" +
                std::to_string(countTaiMilliseconds(mTimer->getCurrentTime())) + "_" +
                std::to_string(intrand(UINT32_MAX))};
    }

    void MisbehaviorDetectionService::fillMisbehaviorTypeContainer(MisbehaviorTypeContainer_t &misbehaviorTypeContainer,
                                                                   const detectionLevels::DetectionLevels &detectionLevel,
                                                                   const std::bitset<16> &errorCode) {
        auto *semanticDetectionReferenceCam = new DetectionReferenceCAM_t();
        semanticDetectionReferenceCam->detectionLevelCAM = detectionLevel;
        std::string encoded = errorCode.to_string();
        OCTET_STRING_fromBuf(&semanticDetectionReferenceCam->semanticDetectionErrorCodeCAM, encoded.c_str(),
                             (int) strlen(encoded.c_str()));

        misbehaviorTypeContainer.present = MisbehaviorTypeContainer_PR_semanticDetection;
        SemanticDetection_t &semanticDetection = misbehaviorTypeContainer.choice.semanticDetection;
        semanticDetection.present = SemanticDetection_PR_semanticDetectionReferenceCAM;
        semanticDetection.choice.semanticDetectionReferenceCAM = *semanticDetectionReferenceCam;
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

    void MisbehaviorDetectionService::fillRelatedReportContainer(RelatedReportContainer_t *&relatedReportContainer,
                                                                 const std::string &relatedReportId,
                                                                 const int &omittedReportsNumber) {
        relatedReportContainer = vanetza::asn1::allocate<RelatedReportContainer_t>();
        OCTET_STRING_fromBuf(&relatedReportContainer->relatedReportID, relatedReportId.c_str(),
                             (int) strlen(relatedReportId.c_str()));
        relatedReportContainer->omittedReportsNumber = omittedReportsNumber;
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
        int alphaStep = 185 / F2MDParameters::miscParameters.CamLocationVisualizerMaxLength;
        int currentAlpha = 80;
        for (const auto &poi : activePoIs) {
            mTraciAPI->poi.setColor(poi, color);
            currentAlpha += alphaStep;
        }
    }


} // namespace artery
