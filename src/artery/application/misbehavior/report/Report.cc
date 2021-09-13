//
// Created by bastian on 30.08.21.
//

#include "Report.h"

#include <utility>
#include "artery/application/misbehavior/util/HelperFunctions.h"
#include <vanetza/units/time.hpp>
#include <omnetpp.h>

namespace artery {
    using namespace omnetpp;

    namespace {
        auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;

        template<typename T, typename U>
        long round(const boost::units::quantity<T> &q, const U &u) {
            boost::units::quantity<U> v{q};
            return std::round(v.value());
        }
    }

    Report::~Report() {
        delete detectionType.semantic;
        delete detectionType.security;
        vanetza::asn1::free(asn_DEF_SenderInfoContainer, evidence.senderInfo);
        vanetza::asn1::free(asn_DEF_SenderSensorContainer, evidence.senderSensors);
    }

    Report::Report(const vanetza::asn1::MisbehaviorReport &misbehaviorReport,
                   std::map<std::string, std::shared_ptr<ma::ReportSummary>> &reportSummaryList,
                   std::map<std::string, std::shared_ptr<Report>> &currentReportList,
                   std::set<std::shared_ptr<vanetza::asn1::Cam>, CamCompare> &camList) {
        generationTime = 0;
        isValid = false;
        score = 1;
        successfullyParsed = true;
        ReportMetadataContainer reportMetadataContainer = misbehaviorReport->reportMetadataContainer;
        asn_INTEGER2umax(&reportMetadataContainer.generationTime, &generationTime);
        reportId = ia5stringToString(reportMetadataContainer.reportID);

        if (reportMetadataContainer.relatedReportContainer != nullptr) {
            RelatedReportContainer_t relatedReportContainer = *reportMetadataContainer.relatedReportContainer;
            relatedReport = std::make_shared<RelatedReport>();
            relatedReport->referencedReportId = ia5stringToString(relatedReportContainer.relatedReportID);
            relatedReport->omittedReportsNumber = relatedReportContainer.omittedReportsNumber;
        }

        ReportContainer reportContainer = misbehaviorReport->reportContainer;
        if (reportContainer.reportedMessageContainer.present ==
            ReportedMessageContainer_PR_certificateIncludedContainer) {
            EtsiTs103097Data_t reportedMessageData = reportContainer.reportedMessageContainer.choice.certificateIncludedContainer.reportedMessage;
            if (reportedMessageData.content != nullptr) {
                Ieee1609Dot2Content ieee1609Dot2Content = *reportedMessageData.content;
                if (ieee1609Dot2Content.present == Ieee1609Dot2Content_PR_unsecuredData) {
                    this->reportedMessage = getCamFromOpaque(ieee1609Dot2Content.choice.unsecuredData, camList);
                }
            } else {
                successfullyParsed = false;
                return;
            }
        } else {
            std::cout << "ignoring report, only CertificateIncludedContainer is implemented" <<
                      std::endl;
            successfullyParsed = false;
            return;
        }

        if (reportContainer.misbehaviorTypeContainer.present == MisbehaviorTypeContainer_PR_semanticDetection) {
            SemanticDetection_t semanticDetection = reportContainer.misbehaviorTypeContainer.choice.semanticDetection;
            if (semanticDetection.present == SemanticDetection_PR_semanticDetectionReferenceCAM) {
                detectionType.
                        present = detectionTypes::SemanticType;
                detectionType.semantic = new SemanticDetection;
                SemanticDetection *&semantic = detectionType.semantic;
                semantic->detectionLevel = static_cast<detectionLevels::DetectionLevels>(
                        semanticDetection.choice.semanticDetectionReferenceCAM.detectionLevelCAM);
                semantic->errorCode = std::bitset<16>(ia5stringToString(
                        semanticDetection.choice.semanticDetectionReferenceCAM.semanticDetectionErrorCodeCAM));

                switch (semantic->detectionLevel) {
                    case detectionLevels::Level1: {
                        break;
                    }
                    case detectionLevels::Level2: {
                        if (reportContainer.evidenceContainer == nullptr) {
                            std::cout << "invalid report, evidenceContainer missing!" <<
                                      std::endl;
                            successfullyParsed = false;
                            return;
                        }
                        if (reportContainer.evidenceContainer->reportedMessageContainer == nullptr) {
                            std::cout
                                    << "invalid report, reportedMessageContainer (messageEvidenceContainer) missing!"
                                    <<
                                    std::endl;
                            successfullyParsed = false;
                            return;
                        }
                        auto *&reportedMessageContainer = reportContainer.evidenceContainer->reportedMessageContainer;
                        if (reportedMessageContainer->list.count == 0) {
                            std::cout << "invalid report, reportedMessageContainer is empty" <<
                                      std::endl;
                            successfullyParsed = false;
                            return;
                        }
                        decodeMessageEvidenceContainer(*reportedMessageContainer, evidence.reportedMessages, camList);
                        break;
                    }
                    case detectionLevels::Level3: {
                        if (reportContainer.evidenceContainer != nullptr &&
                            reportContainer.evidenceContainer->neighbourMessageContainer != nullptr) {
                            auto *&neighbourMessageContainer = reportContainer.evidenceContainer->neighbourMessageContainer;
                            if (neighbourMessageContainer->list.count == 0) {
                                std::cout << "neighbourMessageContainer is empty" <<
                                          std::endl;
                            } else {
                                decodeMessageEvidenceContainer(*neighbourMessageContainer, evidence.neighbourMessages,
                                                               camList);
                            }
                        }
                        break;
                    }
                    case detectionLevels::Level4: {
                        if (reportContainer.evidenceContainer == nullptr) {
                            std::cout << "invalid report, evidenceContainer missing!" << std::endl;
                            successfullyParsed = false;
                            return;
                        } else if (reportContainer.evidenceContainer->senderInfoContainer == nullptr &&
                                   reportContainer.evidenceContainer->senderSensorContainer == nullptr) {
                            std::cout << "invalid report, senderInfo and senderSensor missing!" << std::endl;
                            successfullyParsed = false;
                            return;
                        } else {
                            if (reportContainer.evidenceContainer->senderInfoContainer != nullptr) {
                                evidence.senderInfo = vanetza::asn1::allocate<SenderInfoContainer_t>();
                                *evidence.senderInfo = *reportContainer.evidenceContainer->senderInfoContainer;
                            }
                            if (reportContainer.evidenceContainer->senderSensorContainer != nullptr) {
                                evidence.senderSensors = vanetza::asn1::allocate<SenderSensorContainer_t>();
                                *evidence.senderSensors = *reportContainer.evidenceContainer->senderSensorContainer;
                            }
                        }
                        break;
                    }
                    default:
                        std::cout << "invalid report, invalid detectionLevel" << std::endl;
                        break;
                }
            }
        } else {
            std::cout << "Nothing to do, only SemanticDetection is implemented" << std::endl;
            successfullyParsed = false;
            return;

        }
    }

    void Report::decodeMessageEvidenceContainer(const MessageEvidenceContainer &messageEvidenceContainer,
                                                std::vector<std::shared_ptr<vanetza::asn1::Cam>> &messages,
                                                std::set<std::shared_ptr<vanetza::asn1::Cam>, CamCompare> &camList) {
        for (int i = 0; i < messageEvidenceContainer.list.count; i++) {
            auto *ieee1609Dot2Content = ((EtsiTs103097Data_t *) messageEvidenceContainer.list.array[i])->content;
            if (ieee1609Dot2Content->present == Ieee1609Dot2Content_PR_unsecuredData) {
                messages.emplace_back(getCamFromOpaque(ieee1609Dot2Content->choice.unsecuredData, camList));
            } else {
                std::cout << "ignoring CAM, only unsigned CAMs can be processed" << std::endl;
            }
        }
    }

    std::shared_ptr<vanetza::asn1::Cam> Report::getCamFromOpaque(const Opaque_t &data,
                                                                 std::set<std::shared_ptr<vanetza::asn1::Cam>, CamCompare> &camList) {
        std::shared_ptr<vanetza::asn1::Cam> cam(new vanetza::asn1::Cam());
        cam->decode(vanetza::ByteBuffer(data.buf, data.buf + data.size));

//        auto it = camList.find(cam);
//        if (it == camList.end()) {
//            camList.insert(cam);
//        } else {
//            cam = (*it);
//        }
        return cam;
    }

    Report::Report(std::string reportId, std::shared_ptr<vanetza::asn1::Cam> cam,
                   const uint64_t &generationTime) : reportId(std::move(reportId)), generationTime(generationTime),
                                                     reportedMessage(std::move(cam)) {
        isValid = false;
        successfullyParsed = false;
        score = 0;
        return;
    }

    void Report::setSemanticDetection(const detectionLevels::DetectionLevels &detectionLevel,
                                      const std::bitset<16> &errorCode) {
        detectionType.present = detectionTypes::SemanticType;
        detectionType.semantic = new SemanticDetection;
        detectionType.semantic->detectionLevel = detectionLevel;
        detectionType.semantic->errorCode = errorCode;
    }


    void Report::setReportedMessages(const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &cams,
                                     const int &maxCamCount) {
        int camCount = (int) cams.size();
        if (camCount >= 1) {
            int limit = std::min((int) camCount - 1, maxCamCount);
            for (int i = (int) camCount - limit - 1; i < camCount; i++) {
                evidence.reportedMessages.emplace_back(cams[i]);
            }
        }
    }


    void Report::setRelatedReport(const std::string &relatedReportId, const long &omittedReportsNumber) {
        relatedReport = std::make_shared<RelatedReport>();
        relatedReport->referencedReportId = relatedReportId;
        relatedReport->omittedReportsNumber = omittedReportsNumber;
    }


    void Report::fillSenderInfoContainer(const VehicleDataProvider *vehicleDataProvider,
                                         const traci::VehicleController *vehicleController) {
        evidence.senderInfo = vanetza::asn1::allocate<SenderInfoContainer_t>();
        SenderInfoContainer_t *&senderInfoContainer = evidence.senderInfo;
        senderInfoContainer->stationType = static_cast<StationType_t>(vehicleDataProvider->getStationType());
        senderInfoContainer->referencePosition = vehicleDataProvider->approximateReferencePosition();
        senderInfoContainer->heading = vehicleDataProvider->approximateHeading();
        senderInfoContainer->speed = vehicleDataProvider->approximateSpeed();
        senderInfoContainer->driveDirection = vehicleDataProvider->speed().value() >= 0.0 ?
                                              DriveDirection_forward : DriveDirection_backward;
        senderInfoContainer->vehicleLength.vehicleLengthValue = (long) (
                vehicleController->getLength().value() * 10);
        senderInfoContainer->vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_noTrailerPresent;
        senderInfoContainer->vehicleWidth = (long) (vehicleController->getWidth().value() *
                                                    10);
        senderInfoContainer->longitudinalAcceleration = vehicleDataProvider->approximateAcceleration();

        senderInfoContainer->curvature.curvatureConfidence = CurvatureConfidence_unavailable;
        senderInfoContainer->curvature.curvatureValue = (long) (
                abs(vehicleDataProvider->curvature() / vanetza::units::reciprocal_metre) *
                10000.0);
        if (senderInfoContainer->curvature.curvatureValue >= 1023) {
            senderInfoContainer->curvature.curvatureValue = 1023;
        }

        senderInfoContainer->yawRate.yawRateValue = (long)
                ((double) round(vehicleDataProvider->yaw_rate(), degree_per_second) *
                 YawRateValue_degSec_000_01ToLeft * 100.0);
        if (senderInfoContainer->yawRate.yawRateValue < -32766 ||
            senderInfoContainer->yawRate.yawRateValue > 32766) {
            senderInfoContainer->yawRate.yawRateValue = YawRateValue_unavailable;
        }
        senderInfoContainer->yawRate.yawRateConfidence = YawRateConfidence_unavailable;
    }

    vanetza::asn1::MisbehaviorReport Report::encode() {

        vanetza::asn1::MisbehaviorReport misbehaviorReport;
        misbehaviorReport->version = 1;
        ReportMetadataContainer_t &reportMetadataContainer = misbehaviorReport->reportMetadataContainer;
        asn_uint642INTEGER(&reportMetadataContainer.generationTime, generationTime);
        OCTET_STRING_fromBuf(&reportMetadataContainer.reportID, reportId.c_str(), (int) strlen(reportId.c_str()));

        ReportContainer &reportContainer = misbehaviorReport->reportContainer;
        reportContainer.reportedMessageContainer.present = ReportedMessageContainer_PR_certificateIncludedContainer;
        EtsiTs103097Data_t &reportedMessageEncoded =
                reportContainer.reportedMessageContainer.choice.certificateIncludedContainer.reportedMessage;
        reportedMessageEncoded.protocolVersion = 3;
        if (reportedMessage != nullptr) {
            reportedMessageEncoded.content = vanetza::asn1::allocate<Ieee1609Dot2Content_t>();
            reportedMessageEncoded.content->present = Ieee1609Dot2Content_PR_unsecuredData;
            auto *encodedMessage = new vanetza::ByteBuffer(reportedMessage->encode());
            OCTET_STRING_fromBuf(&reportedMessageEncoded.content->choice.unsecuredData,
                                 reinterpret_cast<const char *>(encodedMessage->data()), (int) encodedMessage->size());
            delete encodedMessage;
        }

        MisbehaviorTypeContainer_t &misbehaviorTypeContainer = misbehaviorReport->reportContainer.misbehaviorTypeContainer;
        switch (detectionType.present) {
            case detectionTypes::none:
                break;
            case detectionTypes::SemanticType: {
                misbehaviorTypeContainer.present = MisbehaviorTypeContainer_PR_semanticDetection;
                SemanticDetection_t &semanticDetection = misbehaviorTypeContainer.choice.semanticDetection;
                semanticDetection.present = SemanticDetection_PR_semanticDetectionReferenceCAM;

                DetectionReferenceCAM_t &semanticDetectionReferenceCam = semanticDetection.choice.semanticDetectionReferenceCAM;
                semanticDetectionReferenceCam.detectionLevelCAM = detectionType.semantic->detectionLevel;
                std::string encoded = detectionType.semantic->errorCode.to_string();
                OCTET_STRING_fromBuf(&semanticDetectionReferenceCam.semanticDetectionErrorCodeCAM, encoded.c_str(),
                                     (int) strlen(encoded.c_str()));
                break;
            }
            case detectionTypes::SecurityType:
                break;
        }
        if (detectionType.semantic->detectionLevel == detectionLevels::Level4) {
            std::cout << "";
        }

        if (!evidence.reportedMessages.empty() || !evidence.reportedMessages.empty() ||
            evidence.senderInfo != nullptr || evidence.senderSensors != nullptr) {
            EvidenceContainer_t *&evidenceContainer = misbehaviorReport->reportContainer.evidenceContainer;
            evidenceContainer = vanetza::asn1::allocate<EvidenceContainer_t>();

            if (!evidence.reportedMessages.empty()) {
                MessageEvidenceContainer_t *&reportedMessageContainer = evidenceContainer->reportedMessageContainer;
                reportedMessageContainer = vanetza::asn1::allocate<MessageEvidenceContainer_t>();
                for (const auto &cam: evidence.reportedMessages) {
                    auto *singleMessageContainer = vanetza::asn1::allocate<EtsiTs103097Data_t>();
                    singleMessageContainer->content = vanetza::asn1::allocate<Ieee1609Dot2Content_t>();
                    singleMessageContainer->content->present = Ieee1609Dot2Content_PR_unsecuredData;
                    auto *encodedMessage = new vanetza::ByteBuffer(cam->encode());
                    OCTET_STRING_fromBuf(&singleMessageContainer->content->choice.unsecuredData,
                                         (const char *) encodedMessage->data(),
                                         (int) encodedMessage->size());
                    ASN_SEQUENCE_ADD(reportedMessageContainer, singleMessageContainer);
                    delete encodedMessage;
                }
            }

            if (!evidence.neighbourMessages.empty()) {
                MessageEvidenceContainer_t *&neighbourMessageContainer = evidenceContainer->neighbourMessageContainer;
                neighbourMessageContainer = vanetza::asn1::allocate<MessageEvidenceContainer_t>();
                for (const auto &cam: evidence.neighbourMessages) {
                    auto *singleMessageContainer = vanetza::asn1::allocate<EtsiTs103097Data_t>();
                    singleMessageContainer->content = vanetza::asn1::allocate<Ieee1609Dot2Content_t>();
                    singleMessageContainer->content->present = Ieee1609Dot2Content_PR_unsecuredData;
                    auto *encodedMessage = new vanetza::ByteBuffer(cam->encode());
                    OCTET_STRING_fromBuf(&singleMessageContainer->content->choice.unsecuredData,
                                         (const char *) encodedMessage->data(),
                                         (int) encodedMessage->size());
                    ASN_SEQUENCE_ADD(neighbourMessageContainer, singleMessageContainer);
                    delete encodedMessage;
                }
            }

            if (evidence.senderInfo != nullptr) {
                evidenceContainer->senderInfoContainer = vanetza::asn1::allocate<SenderInfoContainer_t>();
                *evidenceContainer->senderInfoContainer = *evidence.senderInfo;
            }
        }


        std::string error;
        if (!misbehaviorReport.validate(error)) {
            throw cRuntimeError("Invalid Misbehavior Report: %s", error.c_str());
        }
        return misbehaviorReport;
    }
}
