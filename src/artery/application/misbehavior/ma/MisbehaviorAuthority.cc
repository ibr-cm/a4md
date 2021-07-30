//
// Created by bastian on 05.07.21.
//

#include <artery/application/CaObject.h>
#include "MisbehaviorAuthority.h"
#include "traci/Core.h"
#include "artery/traci/Cast.h"
#include "artery/application/misbehavior/MisbehaviorReportObject.h"
#include "artery/application/misbehavior/MisbehaviorCaService.h"
#include "artery/application/misbehavior/util/HelperFunctions.h"
#include "artery/application/misbehavior/util/DetectionLevels.h"
#include <bitset>


namespace artery {

    using namespace omnetpp;

    Define_Module(MisbehaviorAuthority)

    MisbehaviorAuthority::MisbehaviorAuthority() {
        traciInitSignal = cComponent::registerSignal("traci.init");
        traciCloseSignal = cComponent::registerSignal("traci.close");
        maNewReport = cComponent::registerSignal("misbehaviorAuthority.newReport");
        maMisbehaviorAnnouncement = cComponent::registerSignal("misbehaviorAuthority.MisbehaviorAnnouncement");
    };

    MisbehaviorAuthority::~MisbehaviorAuthority() {
        this->clear();
    }

    void MisbehaviorAuthority::clear() {
    }

    void MisbehaviorAuthority::initialize() {
        cModule *traci = this->getParentModule()->getSubmodule("traci");
        traci->subscribe(traciInitSignal, this);
        traci->subscribe(traciCloseSignal, this);
        getSimulation()->getSystemModule()->subscribe(maNewReport, this);
        getSimulation()->getSystemModule()->subscribe(maMisbehaviorAnnouncement, this);

        mTimer.setTimebase(par("datetime"));
        cModule *globalEnvMod = this->getParentModule()->getSubmodule("environmentModel");
        if (globalEnvMod == nullptr) {
            throw cRuntimeError("globalEnvMod not found");
        }
        mGlobalEnvironmentModel = dynamic_cast<GlobalEnvironmentModel *>(globalEnvMod);

        F2MDParameters::misbehaviorAuthorityParameters.maxReportAge = par("maxReportAge");
    }

    void MisbehaviorAuthority::handleMessage(omnetpp::cMessage *msg) {
        Enter_Method("handleMessage");
    }

    void MisbehaviorAuthority::receiveSignal(cComponent *source, simsignal_t signal, const SimTime &,
                                             cObject *) {
        if (signal == traciInitSignal) {
            auto core = check_and_cast<traci::Core *>(source);
            mTraciAPI = core->getAPI();
        } else if (signal == traciCloseSignal) {
            clear();
        } else if (signal == maMisbehaviorAnnouncement) {
            auto misbehaviorCaService = check_and_cast<MisbehaviorCaService *>(source);
            StationID_t stationId = misbehaviorCaService->getStationId();
            mMisbehavingPseudonyms[stationId] =
                    new MisbehavingPseudonym(stationId, misbehaviorCaService->getMisbehaviorType(),
                                             misbehaviorCaService->getAttackType());
        }
    }

    void MisbehaviorAuthority::receiveSignal(cComponent *source, omnetpp::simsignal_t signal, cObject *obj,
                                             cObject *) {
        if (signal == maNewReport) {
            auto *reportObject = dynamic_cast<MisbehaviorReportObject *>(obj);
            const vanetza::asn1::MisbehaviorReport &misbehaviorReport = reportObject->shared_ptr().operator*();
            ma::Report *parsedReport = parseReport(misbehaviorReport);
            if (parsedReport != nullptr) {
                std::shared_ptr<ma::Report> reportPtr(parsedReport);
                mReports.emplace(parsedReport->reportId, reportPtr);
            }
        }
    }

    bool camsAreEqual(const vanetza::asn1::Cam &message1, const vanetza::asn1::Cam &message2) {
        if (message1->cam.generationDeltaTime == 37476 && message2->cam.generationDeltaTime == 37476) {
            std::cout << "";
        }
        {
            ItsPduHeader header1 = message1->header;
            ItsPduHeader header2 = message2->header;
            if (header1.protocolVersion != header2.protocolVersion ||
                header1.messageID != header2.messageID ||
                header1.stationID != header2.stationID) {
                return false;
            }
        }
        {
            CoopAwareness cam1 = message1->cam;
            CoopAwareness cam2 = message2->cam;
            if (cam1.generationDeltaTime != cam2.generationDeltaTime) {
                return false;
            }
            {
                BasicContainer bc1 = cam1.camParameters.basicContainer;
                BasicContainer bc2 = cam2.camParameters.basicContainer;
                if (bc1.stationType != bc2.stationType) {
                    return false;
                }
                {
                    ReferencePosition referencePosition1 = bc1.referencePosition;
                    ReferencePosition referencePosition2 = bc2.referencePosition;
                    if (referencePosition1.longitude != referencePosition2.longitude ||
                        referencePosition1.latitude != referencePosition2.latitude) {
                        return false;
                    }
                    {
                        Altitude altitude1 = referencePosition1.altitude;
                        Altitude altitude2 = referencePosition2.altitude;
                        if (altitude1.altitudeConfidence != altitude2.altitudeConfidence ||
                            altitude1.altitudeValue != altitude2.altitudeValue) {
                            return false;
                        }
                    }
                    {
                        PosConfidenceEllipse posConfidenceEllipse1 = referencePosition1.positionConfidenceEllipse;
                        PosConfidenceEllipse posConfidenceEllipse2 = referencePosition2.positionConfidenceEllipse;
                        if (posConfidenceEllipse1.semiMajorConfidence != posConfidenceEllipse2.semiMajorConfidence ||
                            posConfidenceEllipse1.semiMinorConfidence != posConfidenceEllipse2.semiMinorConfidence ||
                            posConfidenceEllipse1.semiMajorOrientation != posConfidenceEllipse2.semiMajorOrientation) {
                            return false;
                        }
                    }
                }
            }
            HighFrequencyContainer highFrequencyContainer1 = cam1.camParameters.highFrequencyContainer;
            HighFrequencyContainer highFrequencyContainer2 = cam2.camParameters.highFrequencyContainer;
            if (highFrequencyContainer1.present != highFrequencyContainer2.present) {
                return false;
            }
            if (cam1.camParameters.highFrequencyContainer.present ==
                HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
                BasicVehicleContainerHighFrequency hfc1 = cam1.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
                BasicVehicleContainerHighFrequency hfc2 = cam2.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
                if (hfc1.vehicleWidth != hfc2.vehicleWidth ||
                    hfc1.driveDirection != hfc2.driveDirection ||
                    hfc1.curvatureCalculationMode != hfc2.curvatureCalculationMode) {
                    return false;
                }
                {
                    Speed speed1 = hfc1.speed;
                    Speed speed2 = hfc2.speed;
                    if (speed1.speedValue != speed2.speedValue ||
                        speed1.speedConfidence != speed2.speedConfidence) {
                        return false;
                    }
                }
                {
                    Heading heading1 = hfc1.heading;
                    Heading heading2 = hfc2.heading;
                    if (heading1.headingValue != heading2.headingValue ||
                        heading1.headingConfidence != heading2.headingConfidence) {
                        return false;
                    }
                }
                {
                    VehicleLength vehicleLength1 = hfc1.vehicleLength;
                    VehicleLength vehicleLength2 = hfc2.vehicleLength;
                    if (vehicleLength1.vehicleLengthValue != vehicleLength2.vehicleLengthValue ||
                        vehicleLength1.vehicleLengthConfidenceIndication !=
                        vehicleLength2.vehicleLengthConfidenceIndication) {
                        return false;
                    }
                }
                {
                    LongitudinalAcceleration la1 = hfc1.longitudinalAcceleration;
                    LongitudinalAcceleration la2 = hfc2.longitudinalAcceleration;
                    if (la1.longitudinalAccelerationValue != la2.longitudinalAccelerationValue ||
                        la1.longitudinalAccelerationConfidence != la2.longitudinalAccelerationConfidence) {
                        return false;
                    }
                }
                {
                    Curvature curvature1 = hfc1.curvature;
                    Curvature curvature2 = hfc2.curvature;
                    if (curvature1.curvatureValue != curvature2.curvatureValue ||
                        curvature1.curvatureConfidence != curvature2.curvatureConfidence) {
                        return false;
                    }
                }
                {
                    YawRate yawRate1 = hfc1.yawRate;
                    YawRate yawRate2 = hfc2.yawRate;
                    if (yawRate1.yawRateValue != yawRate2.yawRateValue ||
                        yawRate1.yawRateConfidence != yawRate2.yawRateConfidence) {
                        return false;
                    }
                }
                {
                    AccelerationControl_t *ac1 = hfc1.accelerationControl;
                    AccelerationControl_t *ac2 = hfc2.accelerationControl;
                    if (ac1 == nullptr && ac2 != nullptr ||
                        ac1 != nullptr && ac2 == nullptr) {
                        return false;
                    }
                    if (ac1 != nullptr) {
                        if (ac1->buf != ac2->buf) {
                            return false;
                        }
                    }
                }
                {
                    LanePosition_t *lanePosition1 = hfc1.lanePosition;
                    LanePosition_t *lanePosition2 = hfc2.lanePosition;
                    if (lanePosition1 == nullptr && lanePosition2 != nullptr ||
                        lanePosition1 != nullptr && lanePosition2 == nullptr) {
                        return false;
                    }
                    if (lanePosition1 != nullptr) {
                        if (*lanePosition1 != *lanePosition2) {
                            return false;
                        }
                    }
                }
                {
                    SteeringWheelAngle *swa1 = hfc1.steeringWheelAngle;
                    SteeringWheelAngle *swa2 = hfc2.steeringWheelAngle;
                    if (swa1 == nullptr && swa2 != nullptr ||
                        swa1 != nullptr && swa2 == nullptr) {
                        return false;
                    }
                    if (swa1 != nullptr) {
                        if (swa1->steeringWheelAngleConfidence != swa2->steeringWheelAngleConfidence ||
                            swa1->steeringWheelAngleValue != swa2->steeringWheelAngleValue) {
                            return false;
                        }
                    }
                }
                {
                    LateralAcceleration *la1 = hfc1.lateralAcceleration;
                    LateralAcceleration *la2 = hfc2.lateralAcceleration;
                    if (la1 == nullptr && la2 != nullptr ||
                        la1 != nullptr && la2 == nullptr) {
                        return false;
                    }
                    if (la1 != nullptr) {
                        if (la1->lateralAccelerationConfidence != la2->lateralAccelerationConfidence ||
                            la1->lateralAccelerationValue != la2->lateralAccelerationValue) {
                            return false;
                        }
                    }
                }
                {
                    VerticalAcceleration *va1 = hfc1.verticalAcceleration;
                    VerticalAcceleration *va2 = hfc2.verticalAcceleration;
                    if (va1 == nullptr && va2 != nullptr ||
                        va1 != nullptr && va2 == nullptr) {
                        return false;
                    }
                    if (va1 != nullptr) {
                        if (va1->verticalAccelerationConfidence != va2->verticalAccelerationConfidence ||
                            va1->verticalAccelerationValue != va2->verticalAccelerationValue) {
                            return false;
                        }
                    }
                }
                {
                    PerformanceClass_t *pc1 = hfc1.performanceClass;
                    PerformanceClass_t *pc2 = hfc2.performanceClass;
                    if (pc1 == nullptr && pc2 != nullptr ||
                        pc1 != nullptr && pc2 == nullptr) {
                        return false;
                    }
                    if (pc1 != nullptr) {
                        if (*pc1 != *pc2) {
                            return false;
                        }
                    }
                }
                {
                    CenDsrcTollingZone *cdtz1 = hfc1.cenDsrcTollingZone;
                    CenDsrcTollingZone *cdtz2 = hfc2.cenDsrcTollingZone;
                    if (cdtz1 == nullptr && cdtz2 != nullptr ||
                        cdtz1 != nullptr && cdtz2 == nullptr) {
                        return false;
                    }
                    if (cdtz1 != nullptr) {
                        if (cdtz1->protectedZoneLatitude != cdtz2->protectedZoneLatitude ||
                            cdtz1->protectedZoneLongitude != cdtz2->protectedZoneLongitude) {
                            return false;
                        }
                        {
                            CenDsrcTollingZoneID_t *cid1 = cdtz1->cenDsrcTollingZoneID;
                            CenDsrcTollingZoneID_t *cid2 = cdtz2->cenDsrcTollingZoneID;
                            if (cid1 == nullptr && cid2 != nullptr ||
                                cid1 != nullptr && cid2 == nullptr) {
                                return false;
                            }
                            if (*cid1 != *cid2) {
                                return false;
                            }
                        }
                    }
                }
            }
            {
                LowFrequencyContainer *lowFrequencyContainer1 = cam1.camParameters.lowFrequencyContainer;
                LowFrequencyContainer *lowFrequencyContainer2 = cam2.camParameters.lowFrequencyContainer;
                if (lowFrequencyContainer1 == nullptr && lowFrequencyContainer2 != nullptr ||
                    lowFrequencyContainer1 != nullptr && lowFrequencyContainer2 == nullptr) {
                    return false;
                }
                if (lowFrequencyContainer1 != nullptr) {
                    if (lowFrequencyContainer1->present != lowFrequencyContainer2->present) {
                        return false;
                    }
                    if (lowFrequencyContainer1->present == LowFrequencyContainer_PR_basicVehicleContainerLowFrequency) {
                        BasicVehicleContainerLowFrequency_t lfc1 = lowFrequencyContainer1->choice.basicVehicleContainerLowFrequency;
                        BasicVehicleContainerLowFrequency_t lfc2 = lowFrequencyContainer2->choice.basicVehicleContainerLowFrequency;
                        if (lfc1.vehicleRole != lfc2.vehicleRole ||
                            *lfc1.exteriorLights.buf != *lfc2.exteriorLights.buf) {
                            return false;
                        }
                        if (lfc1.pathHistory.list.count != lfc2.pathHistory.list.count) {
                            return false;
                        }
                        for (int i = 0; i < lfc1.pathHistory.list.count; i++) {
                            PathPoint *pathPoint1 = lfc1.pathHistory.list.array[i];
                            PathPoint *pathPoint2 = lfc1.pathHistory.list.array[i];
                            {
                                DeltaReferencePosition drp1 = pathPoint1->pathPosition;
                                DeltaReferencePosition drp2 = pathPoint2->pathPosition;
                                if (drp1.deltaAltitude != drp2.deltaAltitude ||
                                    drp1.deltaLatitude != drp2.deltaLatitude ||
                                    drp1.deltaLongitude != drp2.deltaLongitude) {
                                    return false;
                                }
                            }
                            {
                                PathDeltaTime_t *pathDeltaTime1 = pathPoint1->pathDeltaTime;
                                PathDeltaTime_t *pathDeltaTime2 = pathPoint2->pathDeltaTime;
                                if (pathDeltaTime1 == nullptr && pathDeltaTime2 != nullptr ||
                                    pathDeltaTime1 != nullptr && pathDeltaTime2 == nullptr) {
                                    return false;
                                }
                                if (*pathDeltaTime1 != *pathDeltaTime2) {
                                    return false;
                                }
                            }
                        }
                    }
                }
            }
            {
                SpecialVehicleContainer *specialVehicleContainer1 = cam1.camParameters.specialVehicleContainer;
                SpecialVehicleContainer *specialVehicleContainer2 = cam2.camParameters.specialVehicleContainer;
                if (specialVehicleContainer1 == nullptr && specialVehicleContainer2 != nullptr ||
                    specialVehicleContainer1 != nullptr && specialVehicleContainer2 == nullptr) {
                    return false;
                }
                if (specialVehicleContainer1 != nullptr) {
                    if (specialVehicleContainer1->present != specialVehicleContainer2->present) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    ma::Report *MisbehaviorAuthority::parseReport(const vanetza::asn1::MisbehaviorReport &misbehaviorReport) {
        auto *report = new ma::Report();
        ReportMetadataContainer reportMetadataContainer = misbehaviorReport->reportMetadataContainer;
        long generationTime;
        asn_INTEGER2long(&reportMetadataContainer.generationTime, &generationTime);
        uint64_t currentTime = countTaiMilliseconds(mTimer.getTimeFor(simTime()));
        if (currentTime - generationTime >
            (uint64_t) F2MDParameters::misbehaviorAuthorityParameters.maxReportAge * 1000) {
            return nullptr;
        }
        std::string reportId = ia5stringToString(reportMetadataContainer.reportID);
        std::cout << "received report: " << reportId << " " << generationTime << std::endl;
        if (reportId.empty()) {
            return nullptr;
        }
        report->reportId = reportId;
        report->generationTime = generationTime;

        if (reportMetadataContainer.relatedReportContainer != nullptr) {
            RelatedReportContainer_t relatedReportContainer = *reportMetadataContainer.relatedReportContainer;
            std::string relatedReportId = ia5stringToString(relatedReportContainer.relatedReportID);
            std::cout << "  relatedReportId: " << relatedReportId << " " << std::endl;
            auto it = mReports.find(relatedReportId);
            if (it != mReports.end()) {
                report->relatedReport = new ma::RelatedReport;
                report->relatedReport->referencedReport = it->second;
                report->relatedReport->omittedReportsNumber = relatedReportContainer.omittedReportsNumber;
            } else {
                return nullptr;
            }
        }

        ReportContainer reportContainer = misbehaviorReport->reportContainer;
        if (reportContainer.reportedMessageContainer.present ==
            ReportedMessageContainer_PR_certificateIncludedContainer) {
            EtsiTs103097Data_t reportedMessage = reportContainer.reportedMessageContainer.choice.certificateIncludedContainer.reportedMessage;
            if (reportedMessage.content != nullptr) {
                Ieee1609Dot2Content ieee1609Dot2Content = *reportedMessage.content;
                if (ieee1609Dot2Content.present == Ieee1609Dot2Content_PR_unsecuredData) {
                    auto *cam = (vanetza::asn1::Cam *) ieee1609Dot2Content.choice.unsecuredData.buf;
                    std::cout << "  reported StationID: " << (*cam)->header.stationID << std::endl;
                    std::cout << "  genDeltaTime: " << (*cam)->cam.generationDeltaTime << std::endl;
                    report->reportedMessage = *cam;
                } else if (report->relatedReport == nullptr) {
                    return nullptr;
                }
            }
        } else {
            std::cout << "ignoring report, only CertificateIncludedContainer is implemented" << std::endl;
            return nullptr;
        }
        if (reportContainer.misbehaviorTypeContainer.present == MisbehaviorTypeContainer_PR_semanticDetection) {
            SemanticDetection_t semanticDetection = reportContainer.misbehaviorTypeContainer.choice.semanticDetection;
            if (semanticDetection.present == SemanticDetection_PR_semanticDetectionReferenceCAM) {
                report->detectionType.present = ma::detectionTypes::SemanticType;
                auto semantic = new ma::SemanticDetection;
                report->detectionType.semantic = semantic;
                semantic->detectionLevel = (int) semanticDetection.choice.semanticDetectionReferenceCAM.detectionLevelCAM;
                semantic->errorCode = (std::bitset<16>) semanticDetection.choice.semanticDetectionReferenceCAM.semanticDetectionErrorCodeCAM.buf;

                std::cout << "  detection level: "
                          << detectionLevels::DetectionLevelStrings[semantic->detectionLevel]
                          << std::endl;
                std::cout << "  " << semantic->errorCode.to_string() << std::endl;

                switch (semantic->detectionLevel) {
                    case detectionLevels::Level1: {
                        break;
                    }
                    case detectionLevels::Level2: {
                        if (reportContainer.evidenceContainer == nullptr) {
                            std::cout << "invalid report, evidenceContainer missing!" << std::endl;
                            return nullptr;
                        }
                        if (reportContainer.evidenceContainer->reportedMessageContainer == nullptr) {
                            std::cout
                                    << "invalid report, reportedMessageContainer (messageEvidenceContainer) missing!"
                                    << std::endl;
                            return nullptr;
                        }
                        auto *reportedMessageContainer = reportContainer.evidenceContainer->reportedMessageContainer;
                        if (reportedMessageContainer->list.count == 0) {
                            std::cout << "invalid report, reportedMessageContainer is empty" << std::endl;
                            return nullptr;
                        }
                        std::cout << "  evidenceCams: " << std::endl;
                        parseMessageEvidenceContainer(*reportedMessageContainer, report->evidence.reportedMessages);
                        break;
                    }
                    case detectionLevels::Level3: {
                        if (reportContainer.evidenceContainer == nullptr) {
                            std::cout << "invalid report, evidenceContainer missing!" << std::endl;
                            return nullptr;
                        }
                        if (reportContainer.evidenceContainer->neighbourMessageContainer != nullptr) {
                            auto *neighbourMessageContainer = reportContainer.evidenceContainer->neighbourMessageContainer;
                            if (neighbourMessageContainer->list.count == 0) {
                                std::cout << "neighbourMessageContainer is empty" << std::endl;
                            } else {
                                std::cout << "  neighbourCams: " << std::endl;
                                parseMessageEvidenceContainer(*neighbourMessageContainer,
                                                              report->evidence.neighbourMessages);
                            }
//                            std::cout
//                                    << "invalid report, neighbourMessageContainer (messageEvidenceContainer) missing!"
//                                    << std::endl;
//                            return nullptr;
                        }
                    }
                    case detectionLevels::Level4: {
                        std::cout << "Nothing to do, DetectionLevel 4 not implemented" << std::endl;
//                        return nullptr;
                    }
                }
            }
        } else {
            std::cout << "Nothing to do, only SemanticDetection is implemented" << std::endl;
//            return nullptr;

        }
        return report;
    }

    void MisbehaviorAuthority::parseMessageEvidenceContainer(const MessageEvidenceContainer &messageEvidenceContainer,
                                                             std::vector<std::shared_ptr<vanetza::asn1::Cam>> &messages) {
        for (int i = 0; i < messageEvidenceContainer.list.count; i++) {
            auto *ieee1609Dot2Content = ((EtsiTs103097Data_t *) messageEvidenceContainer.list.array[i])->content;
            if (ieee1609Dot2Content->present == Ieee1609Dot2Content_PR_unsecuredData) {
                auto *evidenceCam = (vanetza::asn1::Cam *) ieee1609Dot2Content->choice.unsecuredData.buf;
                std::cout << "    previous genDeltaTime: " << (*evidenceCam)->cam.generationDeltaTime
                          << std::endl;
                bool camFound = false;
                for (auto &mCam : mCams) {
                    if (camsAreEqual((*evidenceCam), *mCam)) {
                        camFound = true;
                        messages.emplace_back(mCam);
                        break;
                    }
                }
                if (!camFound) {
                    auto camPtr = std::make_shared<vanetza::asn1::Cam>(*evidenceCam);
                    messages.emplace_back(camPtr);
                    mCams.emplace_back(camPtr);
                }
            } else {
                std::cout << "ignoring CAM, only unsigned CAMs can be processed" << std::endl;
            }
        }
    }

} // namespace artery