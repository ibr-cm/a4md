
#include "artery/application/md/MisbehaviorDetectionService.h"
#include "artery/application/md/MisbehaviorCaService.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/its/CamParameters.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/writer.h>
#include "artery/application/md/base64.h"
#include "artery/application/CaService.h"
#include <vanetza/btp/ports.hpp>
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/md/MisbehaviorTypes.h"

namespace artery
{
    using namespace omnetpp;
    Define_Module(MisbehaviorDetectionService);

    static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");

    MisbehaviorDetectionService::MisbehaviorDetectionService()
    {
        curl = curl_easy_init();
    }

    MisbehaviorDetectionService::~MisbehaviorDetectionService()
    {
        cancelAndDelete(m_self_msg);
        curl_easy_cleanup(curl);
    }

    void MisbehaviorDetectionService::initialize()
    {
        ItsG5BaseService::initialize();
        m_self_msg = new cMessage("InitializeMessage");
        subscribe(scSignalCamReceived);
        scheduleAt(simTime() + 3.0, m_self_msg);
        mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    }

    void MisbehaviorDetectionService::trigger()
    {
        Enter_Method("trigger");
    }

    void MisbehaviorDetectionService::indicate(const vanetza::btp::DataIndication &ind, cPacket *packet, const NetworkInterface &net)
    {
        Enter_Method("indicate");

        EV_INFO << "packet indication on channel " << net.channel << "with byte length" << packet->getByteLength() << "\n";

        delete (packet);
    }

    void MisbehaviorDetectionService::handleMessage(cMessage *msg)
    {
        Enter_Method("handleMessage");

        if (msg == m_self_msg)
        {
            EV_INFO << "self message\n";
        }
    }

    void MisbehaviorDetectionService::receiveSignal(cComponent *source, simsignal_t signal, cObject *c_obj, cObject *)
    {
        Enter_Method("receiveSignal");
        if (signal == scSignalCamReceived)
        {
            CaObject *ca = dynamic_cast<CaObject *>(c_obj);
            vanetza::asn1::Cam message = ca->asn1();
            uint32_t senderStationId = message->header.stationID;
            // if (message->cam.camParameters.basicContainer.referencePosition.latitude < 4)
            misbehaviorTypes::MisbehaviorTypes senderMisbehaviorType = MisbehaviorCaService::getMisbehaviorTypeOfStationId(senderStationId);
            if (senderMisbehaviorType == misbehaviorTypes::LocalAttacker)
            {
                EV_INFO << "Received manipulated CAM!";
                vanetza::ByteBuffer byteBuffer = ca->asn1().encode();
                std::string encoded(byteBuffer.begin(), byteBuffer.end());
                std::string b64Encoded = base64_encode(reinterpret_cast<const unsigned char *>(encoded.c_str()), encoded.length(), false);
                if (curl)
                {
                    curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:9981/newCAM");
                    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, b64Encoded.c_str());
                    CURLcode curlResponse = curl_easy_perform(curl);
                    if (curlResponse != CURLE_OK)
                    {
                        EV_ERROR << "curl_easy_perform() failed: %s\n"
                                 << curl_easy_strerror(curlResponse);
                    }
                }
            } else if(senderMisbehaviorType == misbehaviorTypes::Benign){
                EV_INFO << "Received benign CAM!";
            } else {
                EV_INFO << "Received weird misbehaviorType";
            }
        }
    }

}
