
#ifndef ARTERY_CASERVICE_H_
#define ARTERY_CASERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/md/HTTPRequest.h"
namespace artery
{
    class MisbehaviorDetectionService : public ItsG5BaseService
    {
    public:
        MisbehaviorDetectionService();
        void initialize() override;
        void indicate(const vanetza::btp::DataIndication &, std::unique_ptr<vanetza::UpPacket>) override;
        void trigger() override;
    protected: 
        void handleMessage(omnetpp::cMessage*) override;
    private:
        omnetpp::cMessage* m_self_msg;
    };

} // namespace artery

#endif /* ARTERY_CASERVICE_H_ */