
#include "artery/application/md/MisbehaviorDetectionService.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/its/CamParameters.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/writer.h>
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
        m_self_msg = new cMessage("Misbehavior Detection Service");
        subscribe(scSignalCamReceived);
        scheduleAt(simTime() + 3.0, m_self_msg);

        //     auto &vehicle = getFacilities().get_const<traci::VehicleController>();
        //     std::string reportStr = "curTime: ";
        //     // reportStr.append(std::to_string(simTime().dbl()));
        //     reportStr.append(vehicle.getVehicleId());
        //     HTTPRequest httpr = HTTPRequest(9981, "localhost");
        //     std::string response = httpr.Request(reportStr);
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
            vanetza::asn1::Cam msg = ca->asn1();
            CoopAwareness_t &cam = msg->cam;
            BasicContainer_t &basic = cam.camParameters.basicContainer;
            auto &vehicle = getFacilities().get_const<traci::VehicleController>();
            EV_INFO << "Vehicle " << vehicle.getVehicleId() << " received a CAM in sibling serivce\n";
            EV_INFO << "stationID: " << msg->header.stationID << "\n";

            rapidjson::Document d;
            d.SetObject();
            rapidjson::Document::AllocatorType &allocator = d.GetAllocator();
            d.AddMember("generationDeltaTime", 23445, allocator);

            rapidjson::Value camParameters(rapidjson::kObjectType);

            rapidjson::Value basicContainer(rapidjson::kObjectType);
            basicContainer.AddMember("stationType", msg->header.stationID, allocator);
            rapidjson::Value referencePosition(rapidjson::kObjectType);
            referencePosition.AddMember("latitude", basic.referencePosition.latitude, allocator);
            referencePosition.AddMember("longitude", basic.referencePosition.longitude, allocator);
            referencePosition.AddMember("altitude", basic.referencePosition.altitude.altitudeValue, allocator);
            rapidjson::Value positionConfidenceEllipse(rapidjson::kObjectType);
            positionConfidenceEllipse.AddMember("semiMajorConfidence", basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence, allocator);
            positionConfidenceEllipse.AddMember("semiMinorConfidence", basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence, allocator);
            // rapidjson::Value semiMajorOrientation(rapidjson::kObjectType);
            positionConfidenceEllipse.AddMember("semiMajorOrientation", basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation, allocator);
            // semiMajorOrientation.AddMember("headingConfidence", 10, allocator);
            // positionConfidenceEllipse.AddMember("semiMajorOrientation", semiMajorOrientation, allocator);
            referencePosition.AddMember("positionConfidenceEllipse", positionConfidenceEllipse, allocator);
            basicContainer.AddMember("referencePosition", referencePosition, allocator);
            camParameters.AddMember("basicContainer", basicContainer, allocator);
            d.AddMember("camParameters", camParameters, allocator);

            rapidjson::StringBuffer strbuf;
            rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
            d.Accept(writer);
            // std::cout << strbuf.GetString() << std::endl;
            // std::string reportStr = "curTime: ";
            // reportStr.append(std::to_string(simTime().dbl()));
            // reportStr.append(" vehicleID:");
            // reportStr.append(vehicle.getVehicleId());
            // reportStr.append(" stationID:");
            // reportStr.append(std::to_string(msg->header.stationID));
            if (curl)
            {
                curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:9981/newCAM");
                curl_easy_setopt(curl, CURLOPT_POSTFIELDS, strbuf.GetString());
                CURLcode curlResponse = curl_easy_perform(curl);
                if (curlResponse != CURLE_OK)
                {
                    EV_ERROR << "curl_easy_perform() failed: %s\n"
                             << curl_easy_strerror(curlResponse);
                }
            }
        }
    }

}
