/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_MISBEHAVIORCASERVICE_H_
#define ARTERY_MISBEHAVIORCASERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/cam.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>
#include "artery/application/md/MisbehaviorTypes.h"
#include "artery/application/md/checks/AttackTypes.h"
#include <omnetpp/crng.h>
#include <map>
#include "artery/traci/VehicleController.h"
#include "MisbehaviorCaParameters.h"

namespace artery {

    class NetworkInterfaceTable;

    class Timer;

    class VehicleDataProvider;

    class MisbehaviorCaService : public ItsG5BaseService {
    public:
        MisbehaviorCaService();

        ~MisbehaviorCaService() override;

        void initialize() override;

        void indicate(const vanetza::btp::DataIndication &, std::unique_ptr<vanetza::UpPacket>) override;

        void trigger() override;


    private:
        void checkTriggeringConditions(const omnetpp::SimTime &);

        bool checkHeadingDelta() const;

        bool checkPositionDelta() const;

        bool checkSpeedDelta() const;

        void sendCam(const omnetpp::SimTime &);

        vanetza::asn1::Cam createBenignCAM(uint16_t genDeltaTime);

        vanetza::asn1::Cam createAttackCAM(uint16_t genDeltaTime);

        omnetpp::SimTime genCamDcc();


        void visualizeCamPosition(vanetza::asn1::Cam cam);

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

        long AttackConstantPositionLatitudeMicrodegrees;
        long AttackConstantPositionLongitudeMicrodegrees;
        long AttackConstantPositionOffsetLatitudeMicrodegrees;
        long AttackConstantPositionOffsetLongitudeMicrodegrees;
        long AttackConstantSpeedValue;
        vanetza::units::Velocity AttackConstantSpeedOffsetValue;
        bool attackEventualStopHasStopped;
        ReferencePosition_t attackEventualStopPosition;
        uint32_t attackDataReplayCurrentStationId;



        std::list<vanetza::asn1::Cam> disruptiveMessageQueue;
        std::queue<vanetza::asn1::Cam> staleMessageQueue;
        std::map<uint32_t,std::queue<vanetza::asn1::Cam>> receivedMessages;
        std::list<std::string> activePoIs;


    };

    static bool staticInitializationComplete = false;

    void addLowFrequencyContainer2(vanetza::asn1::Cam &, unsigned pathHistoryLength = 0);

    static TraCIAPI::VehicleScope *traciVehicleScope;
    static std::shared_ptr<const traci::API> traciAPI;
    static const TraCIAPI::POIScope *traciPoiScope;
    static MisbehaviorCaParameters_t parameters;

} // namespace artery

#endif /* ARTERY_MISBEHAVIORCASERVICE_H_ */
