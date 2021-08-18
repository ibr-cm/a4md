//
// Created by bastian on 06.07.21.
//

#ifndef ARTERY_BASECASERVICE_H
#define ARTERY_BASECASERVICE_H

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include <artery/traci/VehicleController.h>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp.h>


namespace artery {

    class NetworkInterfaceTable;

    class Timer;

    class VehicleDataProvider;

    class BaseCaService : public ItsG5BaseService {

    public:
        BaseCaService();

        void initialize() override;

    protected:

        template<typename T, typename U>
        static long round(const boost::units::quantity<T> &q, const U &u);

        static SpeedValue_t buildSpeedValue(const vanetza::units::Velocity &v);

        bool checkTriggeringConditions(const omnetpp::SimTime &);

        bool checkHeadingDelta() const;

        bool checkPositionDelta() const;

        bool checkSpeedDelta() const;

        omnetpp::SimTime genCamDcc();

        void finalizeAndSendCam(vanetza::asn1::Cam cam,const omnetpp::SimTime &T_now);

        vanetza::asn1::Cam
        createCooperativeAwarenessMessage(uint16_t genDeltaTime);

        void addLowFrequencyContainer(vanetza::asn1::Cam &, unsigned pathHistoryLength = 0);

        ChannelNumber mPrimaryChannel = channel::CCH;
        const NetworkInterfaceTable *mNetworkInterfaceTable = nullptr;
        const VehicleDataProvider *mVehicleDataProvider = nullptr;
        const Timer *mTimer = nullptr;
        LocalDynamicMap *mLocalDynamicMap = nullptr;
        const traci::VehicleController *mVehicleController = nullptr;


        omnetpp::SimTime mGenCamMin;
        omnetpp::SimTime mGenCamMax;
        omnetpp::SimTime mGenCam;
        unsigned mGenCamLowDynamicsCounter;
        unsigned mGenCamLowDynamicsLimit;
        Position mLastCamPosition;
        vanetza::units::Velocity mLastCamSpeed;
        vanetza::units::Angle mLastCamHeading;
        omnetpp::SimTime mLastCamTimestamp;
        omnetpp::SimTime mLastLowCamTimestamp;
        vanetza::units::Angle mHeadingDelta;
        vanetza::units::Length mPositionDelta;
        vanetza::units::Velocity mSpeedDelta;
        VehicleWidth_t mVehicleWidth;
        VehicleLengthValue_t mVehicleLength;

        bool mDccRestriction;
        bool mFixedRate;
        long mStationId;

        static bool staticInitializationComplete;
        static std::shared_ptr<const traci::API> mTraciAPI;
        static traci::Boundary mSimulationBoundary;
        static std::chrono::milliseconds scLowFrequencyContainerInterval;
    };


} // namespace artery
#endif //ARTERY_BASECASERVICE_H
