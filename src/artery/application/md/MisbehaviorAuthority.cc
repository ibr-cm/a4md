//
// Created by bastian on 05.07.21.
//

#include <artery/application/CaObject.h>
#include "MisbehaviorAuthority.h"
#include "traci/Core.h"
#include "artery/traci/Cast.h"
#include "MisbehaviorReportObject.h"
#include "artery/application/md/util/HelperFunctions.h"
#include <bitset>

namespace artery {

    using namespace omnetpp;

    Define_Module(MisbehaviorAuthority)

    MisbehaviorAuthority::MisbehaviorAuthority() {
        traciInitSignal = cComponent::registerSignal("traci.init");
        traciCloseSignal = cComponent::registerSignal("traci.close");
        MAnewReport = cComponent::registerSignal("misbehaviorAuthority.newReport");
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
        getSimulation()->getSystemModule()->subscribe(MAnewReport, this);


        cModule *globalEnvMod = this->getParentModule()->getSubmodule("environmentModel");
        if (globalEnvMod == nullptr) {
            throw cRuntimeError("globalEnvMod not found");
        }
        mGlobalEnvironmentModel = dynamic_cast<GlobalEnvironmentModel *>(globalEnvMod);
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
        }
    }

    void MisbehaviorAuthority::receiveSignal(cComponent *source, omnetpp::simsignal_t signal, cObject *obj,
                                             cObject *) {
        if (signal == MAnewReport) {
            auto *reportObject = dynamic_cast<MisbehaviorReportObject *>(obj);
            const vanetza::asn1::MisbehaviorReport &misbehaviorReport = reportObject->shared_ptr().operator*();
            long generationTime;
            asn_INTEGER2long(&misbehaviorReport->reportMetadataContainer.generationTime, &generationTime);
            std::string reportId = ia5stringToString(misbehaviorReport->reportMetadataContainer.reportID);
            std::cout << "received report: " << reportId << " " << generationTime << std::endl;

            ReportMetadataContainer reportMetadataContainer = misbehaviorReport->reportMetadataContainer;
            if(reportMetadataContainer.relatedReportContainer != nullptr){
                RelatedReportContainer_t relatedReportContainer = *reportMetadataContainer.relatedReportContainer;
                std::string relatedReportId = ia5stringToString(relatedReportContainer.relatedReportID);
                std::cout << "  relatedReportId: " << relatedReportId << " " << std::endl;
            }

            ReportContainer reportContainer = misbehaviorReport->reportContainer;
            if (reportContainer.reportedMessageContainer.present ==
                ReportedMessageContainer_PR_certificateIncludedContainer) {
                EtsiTs103097Data_t reportedMessage = reportContainer.reportedMessageContainer.choice.certificateIncludedContainer.reportedMessage;
                if(reportedMessage.content != nullptr){
                    Ieee1609Dot2Content ieee1609Dot2Content = *reportedMessage.content;
                    if (ieee1609Dot2Content.present == Ieee1609Dot2Content_PR_unsecuredData) {
                        auto *cam = (vanetza::asn1::Cam *) ieee1609Dot2Content.choice.unsecuredData.buf;
                        std::cout << "  reported StationID: " << (*cam)->header.stationID << std::endl;
                        std::cout << "  genDeltaTime: " << (*cam)->cam.generationDeltaTime << std::endl;
                    }
                }
            }
            if(reportContainer.misbehaviorTypeContainer.present == MisbehaviorTypeContainer_PR_semanticDetection){
                SemanticDetection_t semanticDetection = reportContainer.misbehaviorTypeContainer.choice.semanticDetection;
                if(semanticDetection.present == SemanticDetection_PR_semanticDetectionReferenceCAM){
                    long detectionLevelCAM = semanticDetection.choice.semanticDetectionReferenceCAM.detectionLevelCAM;
                    std::cout << "  detection level: " << detectionLevelCAM << std::endl;
                    OCTET_STRING_t buffer = semanticDetection.choice.semanticDetectionReferenceCAM.semanticDetectionErrorCodeCAM;
                    std::bitset<16> semanticDetectionErrorCodeCAM = (std::bitset<16>) buffer.buf;
                    std::cout << "  " << semanticDetectionErrorCodeCAM.to_string() << std::endl;
                    if(detectionLevelCAM == 2){
                        auto *evidenceContainer = reportContainer.evidenceContainer;
                        if(evidenceContainer == nullptr) {
                            std::cout << "invalid report, evidenceContainer missing!" << std::endl;
                        }else {
                            auto *messageEvidenceContainer = evidenceContainer->reportedMessageContainer;
                            if(messageEvidenceContainer == nullptr){
                                std::cout << "invalid report, reportedMessageContainer (messageEvidenceContainer) missing!" << std::endl;
                            } else {
                                if(messageEvidenceContainer->list.count == 0){
                                    std::cout << "invalid report, messageEvidenceContainer is empty" << std::endl;
                                } else {
                                    std::cout << "  evidenceCams: " << std::endl;
                                    for (int i = 0; i < messageEvidenceContainer->list.count; i++){
                                        auto *singleMessageContainer = (EtsiTs103097Data_t *) messageEvidenceContainer->list.array[i];
                                        auto *ieee1609Dot2Content = singleMessageContainer->content;
                                        if(ieee1609Dot2Content->present == Ieee1609Dot2Content_PR_unsecuredData){
                                            auto *evidenceCam = (vanetza::asn1::Cam *) ieee1609Dot2Content->choice.unsecuredData.buf;
                                            std::cout << "    previous genDeltaTime: " << evidenceCam->operator->()->cam.generationDeltaTime << std::endl;
                                        }
                                    }

                                }
                            }
                        }
                    }
                }
            }
        }

    }


} // namespace artery