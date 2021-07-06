/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_MISBEHAVIORCASERVICE_H_
#define ARTERY_MISBEHAVIORCASERVICE_H_

#include "artery/application/BaseCaService.h"
#include "artery/application/md/util/F2MDParameters.h"
#include "artery/application/md/util/MisbehaviorTypes.h"
#include "artery/application/md/util/AttackTypes.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/btp/data_interface.hpp>
#include <map>

namespace artery {

    class NetworkInterfaceTable;

    class Timer;

    class VehicleDataProvider;

    class MisbehaviorCaService : public BaseCaService {
    public:
        template<typename T, typename U>
        static long round(const boost::units::quantity<T> &q, const U &u);

        MisbehaviorCaService() = default;

        ~MisbehaviorCaService() override;

        void initialize() override;

        void indicate(const vanetza::btp::DataIndication &, std::unique_ptr<vanetza::UpPacket>) override;

        void trigger() override;

    private:
        void sendCam(const omnetpp::SimTime &);

        vanetza::asn1::Cam createAttackCAM(uint16_t genDeltaTime);

        void initializeStaticParameters();

        void visualizeCamPosition(vanetza::asn1::Cam cam);

        vanetza::asn1::Cam getNextReplayCam();

        const LocalEnvironmentModel *mLocalEnvironmentModel = nullptr;
        static GlobalEnvironmentModel *mGlobalEnvironmentModel;
        static std::shared_ptr<const traci::API> mTraciAPI;
        static traci::Boundary mSimulationBoundary;

        double semiMajorConfidence;
        double semiMinorConfidence;
        double semiMajorOrientationOffset;


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
        int attackGridSybilVehicleCount;
        double attackGridSybilActualDistanceX;
        double attackGridSybilActualDistanceY;
        int attackGridSybilCurrentVehicleIndex;
        double attackGridSybilLastHeadingAngle;
        std::string lastPoiId;

        std::queue<uint32_t> mPseudonyms;
        std::list<vanetza::asn1::Cam> disruptiveMessageQueue;
        std::queue<vanetza::asn1::Cam> staleMessageQueue;
        std::map<uint32_t, std::queue<vanetza::asn1::Cam>> receivedMessages;
        std::list<std::string> activePoIs;
        static bool staticInitializationComplete;

    };
} // namespace artery

#endif /* ARTERY_MISBEHAVIORCASERVICE_H_ */
