
#ifndef ARTERY_MDSERVICE_H_
#define ARTERY_MDSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/application/md/HTTPRequest.h"
#include <vanetza/asn1/asn1c_wrapper.hpp>
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
        std::string getCamJson(vanetza::asn1::Cam);
    };

} // namespace artery

#endif /* ARTERY_MDSERVICE_H_ */