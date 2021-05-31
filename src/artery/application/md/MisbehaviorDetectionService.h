
#ifndef ARTERY_MDSERVICE_H_
#define ARTERY_MDSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/application/md/HTTPRequest.h"
#include <curl/curl.h>

namespace artery
{
    class MisbehaviorDetectionService : public ItsG5Service
    {
    public:
        MisbehaviorDetectionService();
        ~MisbehaviorDetectionService();
        void initialize() override;
        void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*, const NetworkInterface&) override;
        void trigger() override;
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;
    protected: 
        void handleMessage(omnetpp::cMessage*) override;
    private:
        omnetpp::cMessage* m_self_msg;
        CURL *curl;
    };

} // namespace artery

#endif /* ARTERY_MDSERVICE_H_ */