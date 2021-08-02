//
// Created by bastian on 05.07.21.
//

#include <artery/application/CaObject.h>
#include "MisbehaviorAuthority.h"
#include "traci/Core.h"
#include "artery/traci/Cast.h"
#include "artery/application/misbehavior/MisbehaviorReportObject.h"
#include "artery/application/misbehavior/MisbehaviorCaService.h"
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

    template<typename T>
    bool compare(T value1, T value2) {
        if (value1 != value2) {
            return value1 < value2;
        }
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

                auto camPtr = std::make_shared<vanetza::asn1::Cam>(*evidenceCam);
                auto it = mCams.find(camPtr);
                if(it == mCams.end()) {
                    mCams.insert(camPtr);
                } else {
                    camPtr = (*it);
                }
                messages.emplace_back(camPtr);
            } else {
                std::cout << "ignoring CAM, only unsigned CAMs can be processed" << std::endl;
            }
        }
    }

} // namespace artery