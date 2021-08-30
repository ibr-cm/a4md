/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/application/CaService.h"
#include "artery/application/CaObject.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/utility/simtime_cast.h"
#include <boost/units/systems/si/prefixes.hpp>

namespace artery {

    using namespace omnetpp;

    namespace {
        auto microdegree = vanetza::units::degree * boost::units::si::micro;
        auto decidegree = vanetza::units::degree * boost::units::si::deci;
        auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
        auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

        static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
    }


    Define_Module(CaService)

    template<typename T, typename U>
    long CaService::round(const boost::units::quantity<T> &q, const U &u) {
        boost::units::quantity<U> v{q};
        return std::round(v.value());
    }

    void CaService::initialize() {
        BaseCaService::initialize();

        // set vehicle color to green (benign)
        getFacilities().get_const<traci::VehicleController>().getTraCI()->vehicle.setColor(
                getFacilities().get_const<traci::VehicleController>().getVehicleId(),
                libsumo::TraCIColor(0, 255, 0, 255));
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
        auto cam = createCooperativeAwarenessMessage(genDeltaTimeMod);
        finalizeAndSendCam(cam, T_now);
    }

} // namespace artery
