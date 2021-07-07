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
        cancelAndDelete(mSelfMsg);
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
        mSelfMsg = new cMessage("MisbehaviorAuthority");
//        scheduleAt(simTime() + 3.0, mSelfMsg);
    }

    void MisbehaviorAuthority::handleMessage(omnetpp::cMessage *msg) {
        Enter_Method("handleMessage");

        if (msg == mSelfMsg) {
//            std::cout << "self message" << std::endl;
//            std::cout << mGlobalEnvironmentModel->getObstacle("-167762")->getId() << std::endl;
//            std::cout << mTraciAPI->simulation.getCurrentTime() << std::endl;
        }
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
            ReportContainer reportContainer = misbehaviorReport->reportContainer;
            if (reportContainer.reportedMessageContainer.present ==
                ReportedMessageContainer_PR_certificateIncludedContainer) {
                EtsiTs103097Data_t reportedMessage = reportContainer.reportedMessageContainer.choice.certificateIncludedContainer.reportedMessage;
                Ieee1609Dot2Content ieee1609Dot2Content = *reportedMessage.content;
                if (ieee1609Dot2Content.present == Ieee1609Dot2Content_PR_unsecuredData) {
                    std::cout << "unsecuredData" << std::endl;
                    auto *cam = (vanetza::asn1::Cam *) ieee1609Dot2Content.choice.unsecuredData.buf;
                    std::cout << cam->operator->()->header.stationID << std::endl;
                }
            }
            if(reportContainer.misbehaviorTypeContainer.present == MisbehaviorTypeContainer_PR_semanticDetection){
                SemanticDetection_t semanticDetection = reportContainer.misbehaviorTypeContainer.choice.semanticDetection;
                if(semanticDetection.present == SemanticDetection_PR_semanticDetectionReferenceCAM){
                    std::cout << "detection level: " << semanticDetection.choice.semanticDetectionReferenceCAM.detectionLevelCAM << std::endl;
                    OCTET_STRING_t buffer = semanticDetection.choice.semanticDetectionReferenceCAM.semanticDetectionErrorCodeCAM;
                    std::bitset<16> semanticDetectionErrorCodeCAM = (std::bitset<16>) buffer.buf;
                    std::cout << semanticDetectionErrorCodeCAM.to_string() << std::endl;
                }
            }
        }

    }


} // namespace artery