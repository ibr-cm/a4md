//
// Created by bastian on 06.07.21.
//

#include "artery/application/BaseCaService.h"
#include "artery/application/md/util/HelperFunctions.h"
#include "artery/utility/simtime_cast.h"
#include <artery/traci/Cast.h>
#include <vanetza/facilities/cam_functions.hpp>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <boost/units/cmath.hpp>
#include <chrono>

namespace artery {

    using namespace omnetpp;

    Define_Module(BaseCaService)

    namespace {
        auto microdegree = vanetza::units::degree * boost::units::si::micro;
        auto decidegree = vanetza::units::degree * boost::units::si::deci;
        auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
        auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

        static const simsignal_t scSignalCamSent = cComponent::registerSignal("CamSent");
    }

    bool BaseCaService::staticInitializationComplete = false;
    std::shared_ptr<const traci::API> BaseCaService::mTraciAPI;
    traci::Boundary BaseCaService::mSimulationBoundary;
    std::chrono::milliseconds BaseCaService::scLowFrequencyContainerInterval;

    template<typename T, typename U>
    long BaseCaService::round(const boost::units::quantity<T> &q, const U &u) {
        boost::units::quantity<U> v{q};
        return std::round(v.value());
    }

    SpeedValue_t BaseCaService::buildSpeedValue(const vanetza::units::Velocity &v) {
        static const vanetza::units::Velocity lower{0.0 * boost::units::si::meter_per_second};
        static const vanetza::units::Velocity upper{163.82 * boost::units::si::meter_per_second};

        SpeedValue_t speed = SpeedValue_unavailable;
        if (v >= upper) {
            speed = 16382; // see CDD A.74 (TS 102 894 v1.2.1)
        } else if (v >= lower) {
            speed = round(v, centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
        }
        return speed;
    }

    BaseCaService::BaseCaService() : mGenCamMin{100, SIMTIME_MS},
                                     mGenCamMax{1000, SIMTIME_MS},
                                     mGenCam(mGenCamMax),
                                     mGenCamLowDynamicsCounter(0),
                                     mGenCamLowDynamicsLimit(3) {
    }

    void BaseCaService::initialize() {
        ItsG5BaseService::initialize();

        if (!staticInitializationComplete) {
            staticInitializationComplete = true;
            mTraciAPI = getFacilities().get_const<traci::VehicleController>().getTraCI();
            mSimulationBoundary = traci::Boundary{mTraciAPI->simulation.getNetBoundary()};
            scLowFrequencyContainerInterval = std::chrono::milliseconds(500);
        }

        mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
        mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
        mTimer = &getFacilities().get_const<Timer>();
        mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();
        mVehicleController = &getFacilities().get_const<traci::VehicleController>();

        mStationId = mVehicleDataProvider->station_id();
        WATCH(mStationId);

        // avoid unreasonable high elapsed time values for newly inserted vehicles
        mLastCamTimestamp = simTime();

        // first generated CAM shall include the low frequency container
        mLastLowCamTimestamp = mLastCamTimestamp - artery::simtime_cast(scLowFrequencyContainerInterval);

        // generation rate boundaries
        mGenCamMin = par("minInterval");
        mGenCamMax = par("maxInterval");
        mGenCam = mGenCamMax;

        // vehicle dynamics thresholds
        mHeadingDelta = vanetza::units::Angle{par("headingDelta").doubleValue() * vanetza::units::degree};
        mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
        mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

        mDccRestriction = par("withDccRestriction");
        mFixedRate = par("fixedRate");

        // look up primary channel for CA
        mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CA);
    }

    bool BaseCaService::checkTriggeringConditions(const SimTime &T_now) {
        // provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
        SimTime &T_GenCam = mGenCam;
        const SimTime &T_GenCamMin = mGenCamMin;
        const SimTime &T_GenCamMax = mGenCamMax;
        const SimTime T_GenCamDcc = mDccRestriction ? genCamDcc() : T_GenCamMin;
        const SimTime T_elapsed = T_now - mLastCamTimestamp;

        bool trigger = false;
        if (T_elapsed >= T_GenCamDcc) {
            if (mFixedRate) {
                trigger = true;
            } else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
                trigger = true;
                T_GenCam = std::min(T_elapsed, T_GenCamMax); /*< if middleware update interval is too long */
                mGenCamLowDynamicsCounter = 0;
            } else if (T_elapsed >= T_GenCam) {
                trigger = true;
                if (++mGenCamLowDynamicsCounter >= mGenCamLowDynamicsLimit) {
                    T_GenCam = T_GenCamMax;
                }
            }
        }
        return trigger;
    }

    bool BaseCaService::checkHeadingDelta() const {
        return !vanetza::facilities::similar_heading(mLastCamHeading, mVehicleDataProvider->heading(), mHeadingDelta);
    }

    bool BaseCaService::checkPositionDelta() const {
        return (distance(mLastCamPosition, mVehicleDataProvider->position()) > mPositionDelta);
    }

    bool BaseCaService::checkSpeedDelta() const {
        return abs(mLastCamSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
    }

    SimTime BaseCaService::genCamDcc() {
        // network interface may not be ready yet during initialization, so look it up at this later point
        auto networkInterfaceChannel = mNetworkInterfaceTable->select(mPrimaryChannel);
        vanetza::dcc::TransmitRateThrottle *trc = networkInterfaceChannel
                                                  ? networkInterfaceChannel->getDccEntity().getTransmitRateThrottle()
                                                  : nullptr;
        if (!trc) {
            throw cRuntimeError("No DCC TRC found for CA's primary channel %i", mPrimaryChannel);
        }

        static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
        vanetza::Clock::duration interval = trc->interval(ca_tx);
        SimTime dcc{std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS};
        return std::min(mGenCamMax, std::max(mGenCamMin, dcc));
    }

    void BaseCaService::finalizeAndSendCam(vanetza::asn1::Cam cam, const SimTime &T_now) {
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

    vanetza::asn1::Cam
    BaseCaService::createCooperativeAwarenessMessage(uint16_t genDeltaTime) {
        vanetza::asn1::Cam message;

        ItsPduHeader_t &header = (*message).header;
        header.protocolVersion = 2;
        header.messageID = ItsPduHeader__messageID_cam;
        header.stationID = mVehicleDataProvider->station_id();

        CoopAwareness_t &cam = (*message).cam;
        cam.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
        BasicContainer_t &basic = cam.camParameters.basicContainer;
        basic.stationType = StationType_passengerCar;
        basic.referencePosition = mVehicleDataProvider->approximateReferencePosition();

        HighFrequencyContainer_t &hfc = cam.camParameters.highFrequencyContainer;
        hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
        BasicVehicleContainerHighFrequency &bvc = hfc.choice.basicVehicleContainerHighFrequency;
        bvc.heading = mVehicleDataProvider->approximateHeading();
        bvc.speed = mVehicleDataProvider->approximateSpeed();
        bvc.driveDirection = mVehicleDataProvider->speed().value() >= 0.0 ?
                             DriveDirection_forward : DriveDirection_backward;
        bvc.longitudinalAcceleration = mVehicleDataProvider->approximateAcceleration();
        bvc.curvature.curvatureValue =
                abs(mVehicleDataProvider->curvature() / vanetza::units::reciprocal_metre) * 10000.0;
        if (bvc.curvature.curvatureValue >= 1023) {
            bvc.curvature.curvatureValue = 1023;
        }
        bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
        bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
        bvc.yawRate.yawRateValue =
                round(mVehicleDataProvider->yaw_rate(), degree_per_second) * YawRateValue_degSec_000_01ToLeft * 100.0;
        if (bvc.yawRate.yawRateValue < -32766 || bvc.yawRate.yawRateValue > 32766) {
            bvc.yawRate.yawRateValue = YawRateValue_unavailable;
        }
        bvc.vehicleLength.vehicleLengthValue = (long) (mVehicleController->getLength().value() * 10);
        bvc.vehicleLength.vehicleLengthConfidenceIndication =
                VehicleLengthConfidenceIndication_noTrailerPresent;
        bvc.vehicleWidth = (long) (mVehicleController->getWidth().value() * 10);

        std::string error;
        if (!message.validate(error)) {
            throw cRuntimeError("Invalid High Frequency CAM: %s", error.c_str());
        }

        return message;
    }

    void BaseCaService::addLowFrequencyContainer(vanetza::asn1::Cam &message, unsigned pathHistoryLength) {
        if (pathHistoryLength > 40) {
            EV_WARN << "path history can contain 40 elements at maximum";
            pathHistoryLength = 40;
        }

        LowFrequencyContainer_t *&lfc = message->cam.camParameters.lowFrequencyContainer;
        lfc = vanetza::asn1::allocate<LowFrequencyContainer_t>();
        lfc->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;
        BasicVehicleContainerLowFrequency &bvc = lfc->choice.basicVehicleContainerLowFrequency;
        bvc.vehicleRole = VehicleRole_default;
        bvc.exteriorLights.buf = static_cast<uint8_t *>(vanetza::asn1::allocate(1));
        assert(nullptr != bvc.exteriorLights.buf);
        bvc.exteriorLights.size = 1;
        bvc.exteriorLights.buf[0] |= 1 << (7 - ExteriorLights_daytimeRunningLightsOn);

        for (unsigned i = 0; i < pathHistoryLength; ++i) {
            auto *pathPoint = vanetza::asn1::allocate<PathPoint>();
            pathPoint->pathDeltaTime = vanetza::asn1::allocate<PathDeltaTime_t>();
            *(pathPoint->pathDeltaTime) = 0;
            pathPoint->pathPosition.deltaLatitude = DeltaLatitude_unavailable;
            pathPoint->pathPosition.deltaLongitude = DeltaLongitude_unavailable;
            pathPoint->pathPosition.deltaAltitude = DeltaAltitude_unavailable;
            ASN_SEQUENCE_ADD(&bvc.pathHistory, pathPoint);
        }

        std::string error;
        if (!message.validate(error)) {
            throw cRuntimeError("Invalid Low Frequency CAM: %s", error.c_str());
        }
    }

} // namespace artery