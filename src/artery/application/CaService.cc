/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/application/CaObject.h"
#include "artery/application/CaService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/traci/VehicleController.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <chrono>
#include "artery/application/md/MisbehaviorDetectionService.h"

namespace artery {

    using namespace omnetpp;

    namespace {
        auto microdegree = vanetza::units::degree * boost::units::si::micro;
        auto decidegree = vanetza::units::degree * boost::units::si::deci;
        auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
        auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

        static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
        static const simsignal_t scSignalCamSent = cComponent::registerSignal("CamSent");
    }


    Define_Module(CaService)

    template<typename T, typename U>
    long CaService::round(const boost::units::quantity<T> &q, const U &u) {
        boost::units::quantity<U> v{q};
        return std::round(v.value());
    }

    CaService::~CaService() {
        MisbehaviorDetectionService::removeStationIdFromVehicleList(mVehicleDataProvider->getStationId());
    }

    void CaService::initialize() {
        BaseCaService::initialize();

        // set vehicle color to green (benign)
        getFacilities().get_const<traci::VehicleController>().getTraCI()->vehicle.setColor(
                getFacilities().get_const<traci::VehicleController>().getVehicleId(),
                libsumo::TraCIColor(0, 255, 0, 255));


        MisbehaviorDetectionService::addStationIdToVehicleList(mVehicleDataProvider->getStationId(),
                                                               misbehaviorTypes::Benign);

    }

    void CaService::trigger() {
        Enter_Method("trigger");
        const SimTime &T_now = simTime();
        if (checkTriggeringConditions(T_now)) {
            sendCam(T_now);
        }
    }

    void CaService::indicate(const vanetza::btp::DataIndication &ind, std::unique_ptr<vanetza::UpPacket> packet) {
        Enter_Method("indicate");

        Asn1PacketVisitor<vanetza::asn1::Cam> visitor;
        const vanetza::asn1::Cam *cam = boost::apply_visitor(visitor, *packet);
        if (cam && cam->validate()) {
            CaObject obj = visitor.shared_wrapper;
            emit(scSignalCamReceived, &obj);
            mLocalDynamicMap->updateAwareness(obj);
        }
    }

    void CaService::sendCam(const SimTime &T_now) {
        uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
        auto cam = createCooperativeAwarenessMessage(*mVehicleDataProvider, *mVehicleController, genDeltaTimeMod);

        mLastCamPosition = mVehicleDataProvider->position();
        mLastCamSpeed = mVehicleDataProvider->speed();
        mLastCamHeading = mVehicleDataProvider->heading();
        mLastCamTimestamp = T_now;
        if (T_now - mLastLowCamTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
            addLowFrequencyContainer(cam, par("pathHistoryLength"));
            mLastLowCamTimestamp = T_now;
        }

        using namespace vanetza;
        btp::DataRequestB request;
        request.destination_port = btp::ports::CAM;
        request.gn.its_aid = aid::CA;
        request.gn.transport_type = geonet::TransportType::SHB;
        request.gn.maximum_lifetime = geonet::Lifetime{geonet::Lifetime::Base::One_Second, 1};
        request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
        request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

        CaObject obj(std::move(cam));
        emit(scSignalCamSent, &obj);

        using CamByteBuffer = convertible::byte_buffer_impl<asn1::Cam>;
        std::unique_ptr<geonet::DownPacket> payload{new geonet::DownPacket()};
        std::unique_ptr<convertible::byte_buffer> buffer{new CamByteBuffer(obj.shared_ptr())};
        payload->layer(OsiLayer::Application) = std::move(buffer);
        this->request(request, std::move(payload));
    }

} // namespace artery
