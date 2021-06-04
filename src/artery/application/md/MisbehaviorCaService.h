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

namespace artery
{

	class NetworkInterfaceTable;
	class Timer;
	class VehicleDataProvider;

	class MisbehaviorCaService : public ItsG5BaseService
	{
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

		misbehaviorTypes::MisbehaviorTypes setMisbehaviorType(double, double);

		ChannelNumber mPrimaryChannel = channel::CCH;
		const NetworkInterfaceTable *mNetworkInterfaceTable = nullptr;
		const VehicleDataProvider *mVehicleDataProvider = nullptr;
		const Timer *mTimer = nullptr;
		LocalDynamicMap *mLocalDynamicMap = nullptr;

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

		double ConstPosX;
		double ConstPosY;
		int ConstPosOffsetLatitude;
		int ConstPosOffsetLongitude;
		double ConstSpeedX;
		double ConstSpeedY;
		double ConstSpeedOffsetX;
		double ConstSpeedOffsetY;
		double MaxRandomOffsetLatitude;
		double MaxRandomOffsetLongitude;
		long MaxRandomOffsetLatitudeMicrodegrees;
		long MaxRandomOffsetLongitudeMicrodegrees;
	};

	void addLowFrequencyContainer2(vanetza::asn1::Cam &, unsigned pathHistoryLength = 0);

} // namespace artery

#endif /* ARTERY_CASERVICE_H_ */