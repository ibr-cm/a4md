
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
#include "artery/application/md/base64.h"
#include "artery/application/CaService.h"
#include <vanetza/btp/ports.hpp>
#include "artery/application/VehicleDataProvider.h"

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
        mTimer = &getFacilities().get_const<Timer>();
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

            uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
            auto message = createCooperativeAwarenessMessage(*mVehicleDataProvider, genDeltaTimeMod);

            message->cam.camParameters.basicContainer.referencePosition.latitude = 3.14;

            using namespace vanetza;
            btp::DataRequestB request;
            request.destination_port = btp::ports::CAM;
            request.gn.its_aid = aid::CA;
            request.gn.transport_type = geonet::TransportType::SHB;
            request.gn.maximum_lifetime = geonet::Lifetime{geonet::Lifetime::Base::One_Second, 1};
            request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
            request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

            CaObject obj(std::move(message));

            using CamByteBuffer = convertible::byte_buffer_impl<asn1::Cam>;
            std::unique_ptr<geonet::DownPacket> payload{new geonet::DownPacket()};
            std::unique_ptr<convertible::byte_buffer> buffer{new CamByteBuffer(obj.shared_ptr())};
            payload->layer(OsiLayer::Application) = std::move(buffer);
            this->request(request, std::move(payload));
            EV_INFO << "sent attack cam with latitude " << message->cam.camParameters.basicContainer.referencePosition.latitude << "\n";

        }
    }

    std::string MisbehaviorDetectionService::getCamJson(vanetza::asn1::Cam message)
    {
        rapidjson::Document d;
        d.SetObject();
        rapidjson::Document::AllocatorType &allocator = d.GetAllocator();

        // Header
        ItsPduHeader_t &header = message->header;
        rapidjson::Value headerJson(rapidjson::kObjectType);
        headerJson.AddMember("protocolVersion", header.protocolVersion, allocator);
        headerJson.AddMember("messageID", header.messageID, allocator);
        headerJson.AddMember("stationID", header.stationID, allocator);
        d.AddMember("header", headerJson, allocator);

        // CoopAwareness
        CoopAwareness_t &cam = message->cam;
        rapidjson::Value camParameters(rapidjson::kObjectType);
        rapidjson::Value coopAwarenessJson(rapidjson::kObjectType);
        coopAwarenessJson.AddMember("generationDeltaTime", cam.generationDeltaTime, allocator);

        // Basic Container
        BasicContainer_t &basic = cam.camParameters.basicContainer;
        rapidjson::Value basicContainer(rapidjson::kObjectType);
        basicContainer.AddMember("stationType", header.stationID, allocator);
        rapidjson::Value referencePosition(rapidjson::kObjectType);
        referencePosition.AddMember("latitude", basic.referencePosition.latitude, allocator);
        referencePosition.AddMember("longitude", basic.referencePosition.longitude, allocator);
        referencePosition.AddMember("altitude", basic.referencePosition.altitude.altitudeValue, allocator);
        rapidjson::Value positionConfidenceEllipse(rapidjson::kObjectType);
        positionConfidenceEllipse.AddMember("semiMajorConfidence", basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence, allocator);
        positionConfidenceEllipse.AddMember("semiMinorConfidence", basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence, allocator);
        positionConfidenceEllipse.AddMember("semiMajorOrientation", basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation, allocator);
        referencePosition.AddMember("positionConfidenceEllipse", positionConfidenceEllipse, allocator);
        basicContainer.AddMember("referencePosition", referencePosition, allocator);
        camParameters.AddMember("basicContainer", basicContainer, allocator);

        // HighFrequencyContainer
        BasicVehicleContainerHighFrequency_t &high = cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        rapidjson::Value highJson(rapidjson::kObjectType);
        rapidjson::Value heading(rapidjson::kObjectType);
        heading.AddMember("headingValue", high.heading.headingValue, allocator);
        heading.AddMember("headingConfidence", high.heading.headingConfidence, allocator);
        highJson.AddMember("heading", heading, allocator);
        rapidjson::Value speed(rapidjson::kObjectType);
        speed.AddMember("speedValue", high.speed.speedValue, allocator);
        speed.AddMember("speedConfidence", high.speed.speedConfidence, allocator);
        highJson.AddMember("speed", speed, allocator);
        highJson.AddMember("driveDirection", high.driveDirection, allocator);
        rapidjson::Value vehicleLength(rapidjson::kObjectType);
        vehicleLength.AddMember("vehicleLengthValue", high.vehicleLength.vehicleLengthValue, allocator);
        vehicleLength.AddMember("vehicleLengthConfidenceIndication", high.vehicleLength.vehicleLengthConfidenceIndication, allocator);
        highJson.AddMember("vehicleLength", vehicleLength, allocator);
        highJson.AddMember("vehicleWidth", high.vehicleWidth, allocator);
        rapidjson::Value longitudinalAcceleration(rapidjson::kObjectType);
        longitudinalAcceleration.AddMember("longitudinalAccelerationValue", high.longitudinalAcceleration.longitudinalAccelerationValue, allocator);
        longitudinalAcceleration.AddMember("longitudinalAccelerationConfidence", high.longitudinalAcceleration.longitudinalAccelerationConfidence, allocator);
        highJson.AddMember("longitudinalAcceleration", longitudinalAcceleration, allocator);
        rapidjson::Value curvature(rapidjson::kObjectType);
        curvature.AddMember("curvatureValue", high.curvature.curvatureValue, allocator);
        curvature.AddMember("curvatureConfidence", high.curvature.curvatureConfidence, allocator);
        highJson.AddMember("curvature", curvature, allocator);
        highJson.AddMember("curvatureCalculationMode", high.curvatureCalculationMode, allocator);
        rapidjson::Value yawRate(rapidjson::kObjectType);
        yawRate.AddMember("yawRateValue", high.yawRate.yawRateValue, allocator);
        yawRate.AddMember("yawRateConfidence", high.yawRate.yawRateConfidence, allocator);
        highJson.AddMember("yawRate", yawRate, allocator);
        if (high.accelerationControl)
        {
            highJson.AddMember("accelerationControl", *high.accelerationControl->buf, allocator);
        }
        if (high.lanePosition)
        {
            highJson.AddMember("lanePosition", *high.lanePosition, allocator);
        }
        if (high.steeringWheelAngle)
        {
            rapidjson::Value steeringWheelAngle(rapidjson::kObjectType);
            steeringWheelAngle.AddMember("steeringWheelAngleValue", high.steeringWheelAngle->steeringWheelAngleValue, allocator);
            steeringWheelAngle.AddMember("steeringWheelAngleConfidence", high.steeringWheelAngle->steeringWheelAngleConfidence, allocator);
            highJson.AddMember("steeringWheelAngle", steeringWheelAngle, allocator);
        }
        if (high.lateralAcceleration)
        {
            rapidjson::Value lateralAcceleration(rapidjson::kObjectType);
            lateralAcceleration.AddMember("lateralAccelerationValue", high.lateralAcceleration->lateralAccelerationValue, allocator);
            lateralAcceleration.AddMember("lateralAccelerationConfidence", high.lateralAcceleration->lateralAccelerationConfidence, allocator);
            highJson.AddMember("lateralAcceleration", lateralAcceleration, allocator);
        }
        if (high.verticalAcceleration)
        {
            rapidjson::Value verticalAcceleration(rapidjson::kObjectType);
            verticalAcceleration.AddMember("verticalAccelerationValue", high.verticalAcceleration->verticalAccelerationValue, allocator);
            verticalAcceleration.AddMember("verticalAccelerationConfidence", high.verticalAcceleration->verticalAccelerationConfidence, allocator);
            highJson.AddMember("verticalAcceleration", verticalAcceleration, allocator);
        }
        if (high.performanceClass)
        {
            highJson.AddMember("performanceClass", *high.performanceClass, allocator);
        }
        if (high.cenDsrcTollingZone)
        {
            rapidjson::Value cenDsrcTollingZone(rapidjson::kObjectType);
            cenDsrcTollingZone.AddMember("protectedZoneLatitude", high.cenDsrcTollingZone->protectedZoneLatitude, allocator);
            cenDsrcTollingZone.AddMember("protectedZoneLongitude", high.cenDsrcTollingZone->protectedZoneLongitude, allocator);
            if (high.cenDsrcTollingZone->cenDsrcTollingZoneID)
            {
                cenDsrcTollingZone.AddMember("cenDsrcTollingZoneID", *high.cenDsrcTollingZone->cenDsrcTollingZoneID, allocator);
            }
            highJson.AddMember("cenDsrcTollingZone", cenDsrcTollingZone, allocator);
        }
        camParameters.AddMember("highFrequencyContainer", highJson, allocator);
        coopAwarenessJson.AddMember("camParameters", camParameters, allocator);
        d.AddMember("coopAwarenessJson", coopAwarenessJson, allocator);

        rapidjson::StringBuffer strbuf;
        rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
        d.Accept(writer);
        std::string bla = strbuf.GetString();
        return bla;
    }

    void MisbehaviorDetectionService::receiveSignal(cComponent *source, simsignal_t signal, cObject *c_obj, cObject *)
    {
        Enter_Method("receiveSignal");
        if (signal == scSignalCamReceived)
        {
            CaObject *ca = dynamic_cast<CaObject *>(c_obj);
            vanetza::asn1::Cam message = ca->asn1();
            if (message->cam.camParameters.basicContainer.referencePosition.latitude < 4)
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
            }
        }
    }

}
