
#include "artery/application/MisbehaviorDetectionService.h"
#include <omnetpp/cmessage.h>

namespace artery
{
    using namespace omnetpp;

    static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");

    Define_Module(MisbehaviorDetectionService)

        MisbehaviorDetectionService::MisbehaviorDetectionService()
    {
    }

    void MisbehaviorDetectionService::initialize()
    {
        ItsG5BaseService::initialize();
        m_self_msg = new cMessage("Example Service");
        subscribe(scSignalCamReceived);
        scheduleAt(simTime() + 3.0, m_self_msg);
    }

    void MisbehaviorDetectionService::trigger()
    {
        Enter_Method("trigger");
        std::string reportStr = "curTime:";
        reportStr.append(std::to_string(simTime().dbl()));
        HTTPRequest httpr = HTTPRequest(9981, "localhost");
        std::string response = httpr.Request(reportStr);
    }

    void MisbehaviorDetectionService::indicate(const vanetza::btp::DataIndication &, std::unique_ptr<vanetza::UpPacket>)
    {
        Enter_Method("indicate");
    }

    void MisbehaviorDetectionService::handleMessage(cMessage *msg)
    {
        Enter_Method("handleMessage");

        if (msg == m_self_msg)
        {
            EV_INFO << "self message\n";
        }
    }

}
