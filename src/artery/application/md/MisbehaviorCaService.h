/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_CASERVICE_H_
#define ARTERY_CASERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/cam.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>
#include "artery/application/md/MisbehaviorTypes.h"
#include "artery/application/md/AttackTypes.h"
#include <omnetpp/crng.h>
#include <map>
#include "artery/traci/VehicleController.h"

namespace artery {

    class NetworkInterfaceTable;

    class Timer;

    class VehicleDataProvider;

    class MisbehaviorCaService : public ItsG5BaseService {
    public:
        MisbehaviorCaService();

        ~MisbehaviorCaService();

        void initialize() override;

        void indicate(const vanetza::btp::DataIndication &, std::unique_ptr<vanetza::UpPacket>) override;

        void trigger() override;

        static misbehaviorTypes::MisbehaviorTypes getMisbehaviorTypeOfStationId(uint32_t);

    private:
        void checkTriggeringConditions(const omnetpp::SimTime &);

        bool checkHeadingDelta() const;

        bool checkPositionDelta() const;

        bool checkSpeedDelta() const;

        void sendCam(const omnetpp::SimTime &);

        vanetza::asn1::Cam createBenignCAM(const VehicleDataProvider &, uint16_t genDeltaTime);

        vanetza::asn1::Cam createAttackCAM(const VehicleDataProvider &, uint16_t genDeltaTime);

        omnetpp::SimTime genCamDcc();




        std::queue<std::string> activePoIs;
        int CamLocationVisualizerMaxLength;

        void setMisbehaviorType(double localAttacker, double globalAttacker);

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
        bool mDccRestriction;
        bool mFixedRate;

        misbehaviorTypes::MisbehaviorTypes mMisbehaviorType;
        attackTypes::AttackTypes mAttackType;

        double LOCAL_ATTACKER_PROBABILITY;
        double GLOBAL_ATTACKER_PROBABILITY;
        double ATTACK_START_TIME;

        double playgroundSizeX;
        double playgroundSizeY;

        //Constant Position Attack
        double AttackConstantPositionMinLatitude;
        double AttackConstantPositionMaxLatitude;
        double AttackConstantPositionMinLongitude;
        double AttackConstantPositionMaxLongitude;
        long AttackConstantPositionLatitudeMicrodegrees;
        long AttackConstantPositionLongitudeMicrodegrees;

        //Constant Position Offset Attack
        double AttackConstantPositionOffsetMaxLatitudeOffset;
        double AttackConstantPositionOffsetMaxLongitudeOffset;
        long AttackConstantPositionOffsetLatitudeMicrodegrees;
        long AttackConstantPositionOffsetLongitudeMicrodegrees;

        // Random Position Attack
        double AttackRandomPositionMinLatitude;
        double AttackRandomPositionMaxLatitude;
        double AttackRandomPositionMinLongitude;
        double AttackRandomPositionMaxLongitude;

        // Random Position Offset Attack
        double AttackRandomPositionOffsetMaxLatitudeOffset;
        double AttackRandomPositionOffsetMaxLongitudeOffset;

        // Constant Speed Attack
        double AttackConstantSpeedMin;
        double AttackConstantSpeedMax;
        long AttackConstantSpeedValue;

        //Constant Speed Offset Attack
        double AttackConstantSpeedOffsetMax;
        vanetza::units::Velocity AttackConstantSpeedOffsetValue;

        // Random Speed Attack
        double AttackRandomSpeedMin;
        double AttackRandomSpeedMax;

        // Random Speed Offset Attack
        double AttackRandomSpeedOffsetMax;

        // Eventual Stop Attack
        double AttackEventualStopProbabilityThreshold;
        ReferencePosition_t attackEventualStopPosition;
        bool attackEventualStopHasStopped;

        // Disruptive Attack
        int AttackDisruptiveBufferSize;
        int AttackDisruptiveMinimumReceived;
        std::list<vanetza::asn1::Cam> disruptiveMessageQueue;

        // Denial of Service Attack
        int AttackDoSInterval;
        bool AttackDoSIgnoreDCC;

        // Stale Messages Attack
        int AttackStaleDelayCount;
        std::queue<vanetza::asn1::Cam> staleMessageQueue;
    };

    static double totalGenuine = 0;
    static double totalLocalAttacker = 0;
    static double totalGlobalAttacker = 0;
    static bool staticInitializationComplete = false;

    void addLowFrequencyContainer2(vanetza::asn1::Cam &, unsigned pathHistoryLength = 0);

    static std::map<uint32_t, misbehaviorTypes::MisbehaviorTypes> mStationIdMisbehaviorTypeMap;

    static TraCIAPI::VehicleScope *traciVehicleScope;
    static std::shared_ptr<const traci::API> traciAPI;
    static const TraCIAPI::POIScope *traciPoiScope;

} // namespace artery

#endif /* ARTERY_CASERVICE_H_ */
