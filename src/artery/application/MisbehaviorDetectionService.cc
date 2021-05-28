
#include "artery/application/MisbehaviorDetectionService.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
#include <vanetza/asn1/cam.hpp>
// #define COMPILETIME_LOGLEVEL LOGLEVEL_INFO
namespace artery
{
    using namespace omnetpp;

    static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");

    Define_Module(MisbehaviorDetectionService)

        MisbehaviorDetectionService::MisbehaviorDetectionService()
    {
    }

    MisbehaviorDetectionService::~MisbehaviorDetectionService()
    {
        cancelAndDelete(m_self_msg);
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
            auto &vehicle = getFacilities().get_const<traci::VehicleController>();
            EV_INFO << "Vehicle " << vehicle.getVehicleId() << " received a CAM in sibling serivce\n";
            EV_INFO << "stationID: " << msg->header.stationID << "\n";
            // msg->header.stationID
            std::string reportStr = "curTime: ";
            reportStr.append(std::to_string(simTime().dbl()));
            reportStr.append(" vehicleID:");
            reportStr.append(vehicle.getVehicleId());
            reportStr.append(" stationID:");
            reportStr.append(std::to_string(msg->header.stationID));
            HTTPRequest httpr = HTTPRequest(9981, "localhost");
            std::string response = httpr.Request(reportStr);
        }
    }

}
