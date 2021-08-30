//
// Created by bastian on 30.08.21.
//

#include "Report.h"
#include "artery/application/misbehavior/util/HelperFunctions.h"

namespace artery {

    Report::Report(const vanetza::asn1::MisbehaviorReport &misbehaviorReport,
                   std::map<std::string, std::shared_ptr<ma::ReportSummary>> &reportSummaryList,
                   std::map<std::string, std::shared_ptr<Report>> &currentReportList,
                   std::set<std::shared_ptr<vanetza::asn1::Cam>, CamCompare> &camList) {
        generationTime = 0;
        isValid = false;
        score = 1;
        successfullyParsed = true;
        ReportMetadataContainer reportMetadataContainer = misbehaviorReport->reportMetadataContainer;
        asn_INTEGER2long(&reportMetadataContainer.generationTime, (long *) &generationTime);
        reportId = ia5stringToString(reportMetadataContainer.reportID);

        if (reportMetadataContainer.relatedReportContainer != nullptr) {
            RelatedReportContainer_t relatedReportContainer = *reportMetadataContainer.relatedReportContainer;
            std::string relatedReportId = ia5stringToString(relatedReportContainer.relatedReportID);
            auto it = reportSummaryList.find(relatedReportId);
            if (it != reportSummaryList.end()) {
                relatedReport = new RelatedReport;
                relatedReport->referencedReportId = relatedReportId;
                relatedReport->omittedReportsNumber = relatedReportContainer.omittedReportsNumber;
            } else {
                successfullyParsed = false;
            }
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
            } else if (relatedReport == nullptr ||
                       reportSummaryList[relatedReport->referencedReportId]->reportedPseudonym->getPreviousReportGenerationTime() !=
                       generationTime) {
                successfullyParsed = false;
            } else {
                auto it = currentReportList.find(relatedReport->referencedReportId);
                if (it != currentReportList.end()) {
                    this->reportedMessage = it->second->reportedMessage;
                } else {
                    successfullyParsed = false;
                }
            }
        } else {
            std::cout << "ignoring report, only CertificateIncludedContainer is implemented" <<
                      std::endl;
            successfullyParsed = false;
        }

        if (reportContainer.misbehaviorTypeContainer.present == MisbehaviorTypeContainer_PR_semanticDetection) {
            SemanticDetection_t semanticDetection = reportContainer.misbehaviorTypeContainer.choice.semanticDetection;
            if (semanticDetection.present == SemanticDetection_PR_semanticDetectionReferenceCAM) {
                detectionType.
                        present = detectionTypes::SemanticType;
                auto semantic = new SemanticDetection;
                detectionType.semantic = semantic;
                semantic->detectionLevel = static_cast<detectionLevels::DetectionLevels>(
                        semanticDetection.choice.semanticDetectionReferenceCAM.detectionLevelCAM);
                semantic->errorCode = (std::bitset<16>)
                        semanticDetection.choice.semanticDetectionReferenceCAM.semanticDetectionErrorCodeCAM.buf;

                switch (semantic->detectionLevel) {
                    case detectionLevels::Level1: {
                        break;
                    }
                    case detectionLevels::Level2: {
                        if (reportContainer.evidenceContainer == nullptr) {
                            std::cout << "invalid report, evidenceContainer missing!" <<
                                      std::endl;
                            successfullyParsed = false;
                        }
                        if (reportContainer.evidenceContainer->reportedMessageContainer == nullptr) {
                            std::cout
                                    << "invalid report, reportedMessageContainer (messageEvidenceContainer) missing!"
                                    <<
                                    std::endl;
                            successfullyParsed = false;
                        }
                        auto *reportedMessageContainer = reportContainer.evidenceContainer->reportedMessageContainer;
                        if (reportedMessageContainer->list.count == 0) {
                            std::cout << "invalid report, reportedMessageContainer is empty" <<
                                      std::endl;
                            successfullyParsed = false;
                        }
                        parseMessageEvidenceContainer(*reportedMessageContainer, evidence.reportedMessages, camList);
                        break;
                    }
                    case detectionLevels::Level3: {
                        if (reportContainer.evidenceContainer == nullptr) {
                            std::cout << "invalid report, evidenceContainer missing!" <<
                                      std::endl;
                            successfullyParsed = false;
                        }
                        if (reportContainer.evidenceContainer->neighbourMessageContainer != nullptr) {
                            auto *neighbourMessageContainer = reportContainer.evidenceContainer->neighbourMessageContainer;
                            if (neighbourMessageContainer->list.count == 0) {
                                std::cout << "neighbourMessageContainer is empty" <<
                                          std::endl;
                            } else {
                                parseMessageEvidenceContainer(*neighbourMessageContainer, evidence.neighbourMessages,
                                                              camList);
                            }
                        }
                        break;
                    }
                    case detectionLevels::Level4: {
                        if (reportContainer.evidenceContainer == nullptr) {
                            std::cout << "invalid report, evidenceContainer missing!" << std::endl;
                            successfullyParsed = false;
                        } else if (reportContainer.evidenceContainer->senderInfoContainer == nullptr &&
                                   reportContainer.evidenceContainer->senderSensorContainer == nullptr) {
                            std::cout << "invalid report, senderInfo and senderSensor missing!" << std::endl;
                            successfullyParsed = false;
                        } else {
                            if (reportContainer.evidenceContainer->senderInfoContainer != nullptr) {
                                evidence.
                                        senderInfo = std::make_shared<SenderInfoContainer_t>
                                        (*reportContainer.evidenceContainer->senderInfoContainer);
                            }
                            if (reportContainer.evidenceContainer->senderSensorContainer != nullptr) {
                                evidence.
                                        senderSensors = std::make_shared<SenderSensorContainer_t>
                                        (*reportContainer.evidenceContainer->senderSensorContainer);
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

        }
        successfullyParsed = false;
    }

    void Report::parseMessageEvidenceContainer(const MessageEvidenceContainer &messageEvidenceContainer,
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

        auto it = camList.find(cam);
        if (it == camList.end()) {
            camList.insert(cam);
        } else {
            cam = (*it);
        }
        return cam;
    }
}
