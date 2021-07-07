//
// Created by bastian on 05.07.21.
//

#ifndef ARTERY_MISBEHAVIORAUTHORITY_H
#define ARTERY_MISBEHAVIORAUTHORITY_H

#include <omnetpp.h>
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/application/md/util/MisbehaviorTypes.h"
#include "artery/application/md/util/AttackTypes.h"

namespace artery {
    class MisbehaviorAuthority : public omnetpp::cSimpleModule, public omnetpp::cListener {
    public:
        MisbehaviorAuthority();

        ~MisbehaviorAuthority();
        void initialize() override;

        void handleMessage(omnetpp::cMessage *) override;

        void receiveSignal(omnetpp::cComponent *source, omnetpp::simsignal_t signalId, const omnetpp::SimTime &t,
                           omnetpp::cObject *) override;

        void receiveSignal(omnetpp::cComponent *source, omnetpp::simsignal_t signal, cObject *obj,
                           omnetpp::cObject *) override;

    private:

        void clear();

        omnetpp::simsignal_t traciInitSignal;
        omnetpp::simsignal_t traciCloseSignal;
        omnetpp::simsignal_t MAnewReport;

        omnetpp::cMessage* mSelfMsg;
        GlobalEnvironmentModel *mGlobalEnvironmentModel;
        std::shared_ptr<const traci::API> mTraciAPI;
    };
} // namespace artery

#endif //ARTERY_MISBEHAVIORAUTHORITY_H
