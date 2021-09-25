
#include "artery/application/misbehavior/detection/MisbehaviorDetectionService.h"
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
#include "artery/application/misbehavior/report/MisbehaviorReportObject.h"
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
            "newMisbehaviorReport");

    bool MisbehaviorDetectionService::staticInitializationComplete = false;
    std::shared_ptr<const traci::API> MisbehaviorDetectionService::mTraciAPI;
    GlobalEnvironmentModel *MisbehaviorDetectionService::mGlobalEnvironmentModel;
    traci::Boundary MisbehaviorDetectionService::mSimulationBoundary;


    MisbehaviorDetectionService::~MisbehaviorDetectionService() {
        while (!activePoIs.empty()) {
            mTraciAPI->poi.remove(activePoIs.front());
            activePoIs.pop_front();
        }
        detectedSenders.clear();
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

        mCheckableDetectionLevels[detectionLevels::Level1] = true;
        mCheckableDetectionLevels[detectionLevels::Level2] = true;
        mCheckableDetectionLevels[detectionLevels::Level3] = true;
        mCheckableDetectionLevels[detectionLevels::Level4] = true;

        if(uniform(0,1) > F2MDParameters::detectionParameters.generateLevel4Probability){
            mCheckableDetectionLevels[detectionLevels::Level4] = false;
        }
        if(uniform(0,1) > F2MDParameters::detectionParameters.generateLevel3Probability){
            mCheckableDetectionLevels[detectionLevels::Level3] = false;
        }
        if(uniform(0,1) > F2MDParameters::detectionParameters.generateLevel2Probability){
            mCheckableDetectionLevels[detectionLevels::Level2] = false;
        }
        if(uniform(0,1) > F2MDParameters::detectionParameters.generateLevel1Probability){
            mCheckableDetectionLevels[detectionLevels::Level1] = false;
        }

        mFusionApplication = new ThresholdFusion(F2MDParameters::detectionParameters.misbehaviorThreshold);
    }

    void MisbehaviorDetectionService::initializeParameters() {

        F2MDParameters::detectionParameters.checkType = par("checkType");
        F2MDParameters::detectionParameters.misbehaviorThreshold = par("misbehaviorThreshold");
        F2MDParameters::detectionParameters.detectedSenderCamArrayMaxSize = par("detectedSenderCamArrayMaxSize");

        F2MDParameters::detectionParameters.generateLevel4Probability = par("generateLevel4Probability");
        F2MDParameters::detectionParameters.generateLevel3Probability = par("generateLevel3Probability");
        F2MDParameters::detectionParameters.generateLevel2Probability = par("generateLevel2Probability");
        F2MDParameters::detectionParameters.generateLevel1Probability = par("generateLevel1Probability");

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

    void MisbehaviorDetectionService::indicate(const vanetza::btp::DataIndication &ind, cPacket *packet,
                                               const NetworkInterface &net) {
        Enter_Method("indicate");
        delete (packet);
    }


    void MisbehaviorDetectionService::receiveSignal(cComponent *source, simsignal_t signal, cObject *c_obj, cObject *) {
        Enter_Method("receiveSignal");
        if (signal == scSignalCamReceived) {
            auto *ca = dynamic_cast<CaObject *>(c_obj);
            std::shared_ptr<vanetza::asn1::Cam> message = std::make_shared<vanetza::asn1::Cam>(ca->asn1());
            detectMisbehavior(message);
        } else if (signal == scSignalCamSent) {
            auto *ca = dynamic_cast<CaObject *>(c_obj);
            mLastSentCam = std::make_shared<vanetza::asn1::Cam>(ca->asn1());
        }
    }

    void MisbehaviorDetectionService::detectMisbehavior(const shared_ptr<vanetza::asn1::Cam> &message) {
        uint32_t senderStationId = (*message)->header.stationID;
        std::shared_ptr<CheckResult> checkResult = checkCam(message);
        std::vector<std::bitset<16>> detectionLevelErrorCodes = mFusionApplication->checkForReport(*checkResult);
        DetectedSender &detectedSender = *detectedSenders[senderStationId];
        std::string relatedReportId = detectedSender.getPreviousReportId();
        std::bitset<16> reportedErrorCodes = 0;

        for (detectionLevels::DetectionLevels detectionLevel: detectionLevels::DetectionLevelVector) {
            std::bitset<16> errorCode = detectionLevelErrorCodes[(int) detectionLevel];
            if (errorCode.any() && detectedSender.checkOmittedReportsLimit(errorCode)) {
                std::string reportId(
                        generateReportId(senderStationId, mVehicleDataProvider->getStationId(), getRNG(0)));
                Report report(reportId, message, countTaiMilliseconds(mTimer->getTimeFor(simTime())));
                report.setSemanticDetection(detectionLevel, errorCode);
                SenderInfoContainer_t *senderInfoContainer = (vanetza::asn1::allocate<SenderInfoContainer_t>());
                switch (detectionLevel) {
                    case detectionLevels::Level1:
                        break;
                    case detectionLevels::Level2: {
                        int camCount = F2MDParameters::reportParameters.evidenceContainerMaxCamCount >
                                       F2MDParameters::reportParameters.omittedReportsCount
                                       ? F2MDParameters::reportParameters.omittedReportsCount
                                       : F2MDParameters::reportParameters.evidenceContainerMaxCamCount;
                        report.setReportedMessages(detectedSender.getCamVector(), camCount);
                        break;
                    }
                    case detectionLevels::Level3:
                        if (checkResult->intersection < F2MDParameters::detectionParameters.misbehaviorThreshold) {
                            report.evidence.neighbourMessages = getOverlappingCams(detectedSender);
                        }
                        break;
                    case detectionLevels::Level4: {
                        report.fillSenderInfoContainer(mVehicleDataProvider, mVehicleController);
                        break;
                    }
                    default:
                        break;
                }
                if (!relatedReportId.empty()) {
                    report.setRelatedReport(relatedReportId, F2MDParameters::reportParameters.omittedReportsCount);
                }
                reportedErrorCodes |= errorCode;
                vanetza::asn1::MisbehaviorReport misbehaviorReport = report.encode();
                MisbehaviorReportObject obj(std::move(misbehaviorReport));
                emit(scSignalMisbehaviorAuthorityNewReport, &obj);
                vanetza::asn1::free(asn_DEF_SenderInfoContainer, senderInfoContainer);
                relatedReportId = reportId;
                if (detectedSender.getPreviousReportId().empty()) {
                    detectedSender.setReportId(relatedReportId);
                }
            }
        }
        if (reportedErrorCodes.any()) {
            detectedSender.resetOmittedReports(reportedErrorCodes);
        }
        detectedSender.incrementOmittedReports(detectionLevelErrorCodes, reportedErrorCodes);
    }


    std::shared_ptr<CheckResult> MisbehaviorDetectionService::checkCam(const shared_ptr<vanetza::asn1::Cam> &message) {

        uint32_t senderStationId = (*message)->header.stationID;
        std::vector<Position> mVehicleOutline = getVehicleOutline(mVehicleDataProvider, mVehicleController);

        std::vector<std::shared_ptr<vanetza::asn1::Cam>> surroundingCamObjects = getSurroundingCams(
                senderStationId);
        if (detectedSenders.find(senderStationId) == detectedSenders.end()) {
            detectedSenders[senderStationId] = std::make_shared<DetectedSender>(mTraciAPI, mGlobalEnvironmentModel,
                                                                                &F2MDParameters::detectionParameters,
                                                                                mTimer, message, mCheckableDetectionLevels);
        }
        std::shared_ptr<CheckResult> result = detectedSenders[senderStationId]->addAndCheckCam(message,
                                                                                               mVehicleDataProvider,
                                                                                               mVehicleOutline,
                                                                                               surroundingCamObjects);
        return result;
    }

    std::vector<std::shared_ptr<vanetza::asn1::Cam>>
    MisbehaviorDetectionService::getSurroundingCams(StationID_t senderStationId) {
        std::vector<std::shared_ptr<vanetza::asn1::Cam>> surroundingCamObjects;
        for (const auto &it: detectedSenders) {
            auto detectedSender = it.second;
            if (detectedSender->getStationId() != senderStationId) {
                std::shared_ptr<vanetza::asn1::Cam> latestCam = detectedSender->getCams().back();
                uint16_t oldTime = (*latestCam)->cam.generationDeltaTime;
                uint16_t currentTime = countTaiMilliseconds(mTimer->getCurrentTime());
                if ((uint16_t) (currentTime - oldTime) <
                    (long) (F2MDParameters::detectionParameters.maxCamFrequency * 1000)) {
                    surroundingCamObjects.emplace_back(latestCam);
                }
            }
        }
        return surroundingCamObjects;
    }

    std::vector<std::shared_ptr<vanetza::asn1::Cam>>
    MisbehaviorDetectionService::getOverlappingCams(const DetectedSender &detectedSender) {
        std::vector<Position> senderOutline = getVehicleOutline(*detectedSender.getCams().back(),
                                                                mSimulationBoundary,
                                                                mTraciAPI);
        std::vector<std::shared_ptr<vanetza::asn1::Cam>> overlappingCams;
        if (boost::geometry::intersects(senderOutline, getVehicleOutline(mVehicleDataProvider,
                                                                         mVehicleController))) {
            overlappingCams.emplace_back(mLastSentCam);
        }
        for (const auto &cam: getSurroundingCams(detectedSender.getStationId())) {
            std::vector<Position> outline = getVehicleOutline((*cam), mSimulationBoundary,
                                                              mTraciAPI);
            if (boost::geometry::intersects(senderOutline, outline)) {
                overlappingCams.emplace_back(cam);
            }
        }
        return overlappingCams;
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
        for (const auto &poi: activePoIs) {
            mTraciAPI->poi.setColor(poi, color);
            currentAlpha += alphaStep;
        }
    }
} // namespace artery
