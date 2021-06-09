/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/application/CaObject.h"
#include "artery/application/md/MisbehaviorCaService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
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

#include "artery/application/CaService.h"
#include "inet/common/ModuleAccess.h"
#include "artery/traci/ControllableVehicle.h"
#include "artery/traci/VehicleController.h"


namespace artery
{

	auto microdegree2 = vanetza::units::degree * boost::units::si::micro;
	auto decidegree2 = vanetza::units::degree * boost::units::si::deci;
	auto degree_per_second2 = vanetza::units::degree / vanetza::units::si::second;
	auto centimeter_per_second2 = vanetza::units::si::meter_per_second * boost::units::si::centi;

	static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
	static const simsignal_t scSignalCamSent = cComponent::registerSignal("CamSent");
	static const auto scLowFrequencyContainerInterval = std::chrono::milliseconds(500);

	template <typename T, typename U>
	long round(const boost::units::quantity<T> &q, const U &u)
	{
		boost::units::quantity<U> v{q};
		return std::round(v.value());
	}

	SpeedValue_t buildSpeedValue2(const vanetza::units::Velocity &v)
	{
		static const vanetza::units::Velocity lower{0.0 * boost::units::si::meter_per_second};
		static const vanetza::units::Velocity upper{163.82 * boost::units::si::meter_per_second};

		SpeedValue_t speed = SpeedValue_unavailable;
		if (v >= upper)
		{
			speed = 16382; // see CDD A.74 (TS 102 894 v1.2.1)
		}
		else if (v >= lower)
		{
			speed = round(v, centimeter_per_second2) * SpeedValue_oneCentimeterPerSec;
		}
		return speed;
	}

	Define_Module(MisbehaviorCaService)

		MisbehaviorCaService::MisbehaviorCaService() : mGenCamMin{100, SIMTIME_MS},
													   mGenCamMax{1000, SIMTIME_MS},
													   mGenCam(mGenCamMax),
													   mGenCamLowDynamicsCounter(0),
													   mGenCamLowDynamicsLimit(3)
	{
	}

	MisbehaviorCaService::~MisbehaviorCaService()
	{
		auto it = mStationIdMisbehaviorTypeMap.find(mVehicleDataProvider->getStationId());
		if (it != mStationIdMisbehaviorTypeMap.end())
		{
			it = mStationIdMisbehaviorTypeMap.erase(it);
		}
	}

	void MisbehaviorCaService::initialize()
	{
		ItsG5BaseService::initialize();

		mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
		mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
		mTimer = &getFacilities().get_const<Timer>();
		mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();

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

		playgroundSizeX = 1500;
		playgroundSizeY = 1500;

		LOCAL_ATTACKER_PROBABILITY = par("LOCAL_ATTACKER_PROB");
		GLOBAL_ATTACKER_PROBABILITY = par("GLOBAL_ATTACKER_PROB");
		ATTACK_START_TIME = par("START_ATTACK");

		// Constant Position Attack
		AttackConstantPositionMinLatitude = par("AttackConstantPositionMinLatitude");
		AttackConstantPositionMaxLatitude = par("AttackConstantPositionMaxLatitude");
		AttackConstantPositionMinLongitude = par("AttackConstantPositionMinLongitude");
		AttackConstantPositionMaxLongitude = par("AttackConstantPositionMaxLongitude");
		AttackConstantPositionLatitudeMicrodegrees = uniform(AttackConstantPositionMinLatitude, AttackConstantPositionMaxLatitude) * 100000;
		AttackConstantPositionLongitudeMicrodegrees = uniform(AttackConstantPositionMinLongitude, AttackConstantPositionMaxLongitude) * 100000;

		// Constant Position Offset Attack
		AttackConstantPositionOffsetMaxLatitudeOffset = par("AttackConstantPositionOffsetMaxLatitudeOffset");
		AttackConstantPositionOffsetMaxLongitudeOffset = par("AttackConstantPositionOffsetMaxLongitudeOffset");
		AttackConstantPositionOffsetLatitudeMicrodegrees = uniform(-AttackConstantPositionOffsetMaxLatitudeOffset, AttackConstantPositionOffsetMaxLatitudeOffset);
		AttackConstantPositionOffsetLongitudeMicrodegrees = uniform(-AttackConstantPositionOffsetMaxLongitudeOffset, AttackConstantPositionOffsetMaxLongitudeOffset);

		// Random Position Attack
		AttackRandomPositionMinLatitude = par("AttackRandomPositionMinLatitude");
		AttackRandomPositionMaxLatitude = par("AttackRandomPositionMaxLatitude");
		AttackRandomPositionMinLongitude = par("AttackRandomPositionMinLongitude");
		AttackRandomPositionMaxLongitude = par("AttackRandomPositionMaxLongitude");

		// Random Position Offset Attack
		AttackRandomPositionOffsetMaxLatitudeOffset = par("AttackRandomPositionOffsetMaxLatitudeOffset");
		AttackRandomPositionOffsetMaxLongitudeOffset = par("AttackRandomPositionOffsetMaxLongitudeOffset");

		// Constant Speed Attack
		// Meters per Second
		AttackConstantSpeedMin = par("AttackConstantSpeedMin");
		AttackConstantSpeedMax = par("AttackConstantSpeedMax");
		AttackConstantSpeedValue = buildSpeedValue2(uniform(AttackConstantSpeedMin, AttackConstantSpeedMax) * boost::units::si::meter_per_second);

		// Constant Speed Offset Attack
		AttackConstantSpeedOffsetMax = par("AttackConstantSpeedOffsetMax");
		double speedOffset = uniform(0, AttackConstantSpeedOffsetMax);
		AttackConstantSpeedOffsetValue = ((long)speedOffset) * boost::units::si::meter_per_second;

		// Random Speed Attack
		// Meters per Second
		AttackRandomSpeedMin = par("AttackRandomSpeedMin");
		AttackRandomSpeedMax = par("AttackRandomSpeedMax");

		// Random Speed Offset Attack
		AttackRandomSpeedOffsetMax = par("AttackRandomSpeedOffsetMax");

		// Eventual Stop Attack
		AttackEventualStopProbabilityThreshold = par("AttackEventualStopProbabilityThreshold");
		attackEventualStopHasStopped = false;

		// Disruptive Attack
		AttackDisruptiveBufferSize = par("AttackDisruptiveBufferSize");
		AttackDisruptiveMinimumReceived = par("AttackDisruptiveMinimumReceived");

		// Denial of Service Attack
		AttackDoSInterval = par("AttackDoSInterval");
		AttackDoSIgnoreDCC = par("AttackDoSIgnoreDCC");

		// Stale Messages Attack
		AttackStaleDelayCount = par("AttackStaleDelayCount");

		mMisbehaviorType = setMisbehaviorType(LOCAL_ATTACKER_PROBABILITY, GLOBAL_ATTACKER_PROBABILITY);
		mAttackType = attackTypes::RandomPosOffset;

		if (mAttackType == attackTypes::DoS)
		{
			mGenCamMin = {AttackDoSInterval, SIMTIME_MS};
			mDccRestriction = !AttackDoSIgnoreDCC;
			mFixedRate = true;
		}

		// traciAPI = getFacilities().get_const<traci::VehicleController>().getTraCI();
		const traci::VehicleController *mVehicleController = &getFacilities().get_const<traci::VehicleController>();
		const TraCIAPI::VehicleScope &vehicle_api = mVehicleController->getTraCI()->vehicle;
		auto &bla = mVehicleController->getTraCI()->poi;
		if (mMisbehaviorType == misbehaviorTypes::Benign)
		{
			vehicle_api.setColor(mVehicleController->getVehicleId(), libsumo::TraCIColor(0, 255, 0, 255));
		}
		else
		{
			vehicle_api.setColor(mVehicleController->getVehicleId(), libsumo::TraCIColor(255, 0, 0, 255));
		}

		mStationIdMisbehaviorTypeMap[mVehicleDataProvider->getStationId()] = mMisbehaviorType;
	}

	misbehaviorTypes::MisbehaviorTypes MisbehaviorCaService::getMisbehaviorTypeOfStationId(uint32_t stationId)
	{
		auto search = mStationIdMisbehaviorTypeMap.find(stationId);
		if (search != mStationIdMisbehaviorTypeMap.end())
		{
			return search->second;
		}
		else
		{
			return misbehaviorTypes::SIZE_OF_ENUM;
		}
	}

	misbehaviorTypes::MisbehaviorTypes MisbehaviorCaService::setMisbehaviorType(double localAttacker, double globalAttacker)
	{
		if (simTime().dbl() < ATTACK_START_TIME)
		{
			return misbehaviorTypes::Benign;
		}

		if ((totalLocalAttacker + totalGenuine) == 0)
		{
			totalGenuine++;
			return misbehaviorTypes::Benign;
		}

		double realFactor = totalLocalAttacker / (totalGenuine + totalLocalAttacker);
		if (localAttacker > realFactor)
		{
			totalLocalAttacker++;
			return misbehaviorTypes::LocalAttacker;
		}
		else
		{
			double realGFactor = totalGlobalAttacker / (totalGenuine + totalGlobalAttacker);
			if (globalAttacker > realGFactor)
			{
				totalGlobalAttacker++;
				return misbehaviorTypes::GlobalAttacker;
			}
			else
			{
				totalGenuine++;
				return misbehaviorTypes::Benign;
			}
		}
	}

	void MisbehaviorCaService::trigger()
	{
		Enter_Method("trigger");
		// if (mMisbehaviorType != misbehaviorTypes::Benign)
		// {
		// 	EV_INFO << "Trigger " << simTime().inUnit(SimTimeUnit::SIMTIME_MS) << "\n";
		// }
		checkTriggeringConditions(simTime());
	}

	void MisbehaviorCaService::indicate(const vanetza::btp::DataIndication &ind, std::unique_ptr<vanetza::UpPacket> packet)
	{
		Enter_Method("indicate");

		Asn1PacketVisitor<vanetza::asn1::Cam> visitor;
		const vanetza::asn1::Cam *cam = boost::apply_visitor(visitor, *packet);
		if (cam && cam->validate())
		{
			CaObject obj = visitor.shared_wrapper;
			emit(scSignalCamReceived, &obj);
			mLocalDynamicMap->updateAwareness(obj);
			if (mAttackType == attackTypes::Disruptive)
			{
				disruptiveMessageQueue.emplace_back(*cam);
				if (disruptiveMessageQueue.size() > AttackDisruptiveBufferSize)
				{
					disruptiveMessageQueue.pop_front();
				}
			}
		}
	}

	void MisbehaviorCaService::checkTriggeringConditions(const SimTime &T_now)
	{
		// provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
		SimTime &T_GenCam = mGenCam;
		const SimTime &T_GenCamMin = mGenCamMin;
		const SimTime &T_GenCamMax = mGenCamMax;
		const SimTime T_GenCamDcc = mDccRestriction ? genCamDcc() : T_GenCamMin;
		const SimTime T_elapsed = T_now - mLastCamTimestamp;

		if (T_elapsed >= T_GenCamDcc)
		{
			if (mFixedRate)
			{
				sendCam(T_now);
			}
			else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta())
			{
				sendCam(T_now);
				T_GenCam = std::min(T_elapsed, T_GenCamMax); /*< if middleware update interval is too long */
				mGenCamLowDynamicsCounter = 0;
			}
			else if (T_elapsed >= T_GenCam)
			{
				sendCam(T_now);
				if (++mGenCamLowDynamicsCounter >= mGenCamLowDynamicsLimit)
				{
					T_GenCam = T_GenCamMax;
				}
			}
		}
	}

	bool MisbehaviorCaService::checkHeadingDelta() const
	{
		return !vanetza::facilities::similar_heading(mLastCamHeading, mVehicleDataProvider->heading(), mHeadingDelta);
	}

	bool MisbehaviorCaService::checkPositionDelta() const
	{
		return (distance(mLastCamPosition, mVehicleDataProvider->position()) > mPositionDelta);
	}

	bool MisbehaviorCaService::checkSpeedDelta() const
	{
		return abs(mLastCamSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
	}

	void MisbehaviorCaService::sendCam(const SimTime &T_now)
	{
		uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
		vanetza::asn1::Cam cam;
		switch (mMisbehaviorType)
		{
		case misbehaviorTypes::Benign:
			cam = createBenignCAM(*mVehicleDataProvider, genDeltaTimeMod);
			break;
		case misbehaviorTypes::LocalAttacker:
		{
			cam = createAttackCAM(*mVehicleDataProvider, genDeltaTimeMod);
			const traci::VehicleController *mVehicleController = &getFacilities().get_const<traci::VehicleController>();
			auto &bla = mVehicleController->getTraCI()->poi;
			traci::TraCIGeoPosition traciGeoPosition = {cam->cam.camParameters.basicContainer.referencePosition.longitude / 10000000.0, cam->cam.camParameters.basicContainer.referencePosition.latitude / 10000000.0};
			traci::TraCIPosition traciPosition = mVehicleController->getTraCI()->convert2D(traciGeoPosition);
			std::string id = "CamLocation_";
			id += std::to_string(cam->cam.generationDeltaTime);
			bla.add(id, traciPosition.x, traciPosition.y, libsumo::TraCIColor(255, 0, 255, 255), id, 5, "", 105, 105, 0);
			break;
		}
		case misbehaviorTypes::GlobalAttacker:
			cam = createAttackCAM(*mVehicleDataProvider, genDeltaTimeMod);
			break;
		default:
			cam = createBenignCAM(*mVehicleDataProvider, genDeltaTimeMod);
		}
		if (&cam->header == nullptr)
		{
			return;
		}

		mLastCamPosition = mVehicleDataProvider->position();
		mLastCamSpeed = mVehicleDataProvider->speed();
		mLastCamHeading = mVehicleDataProvider->heading();
		mLastCamTimestamp = T_now;
		if (T_now - mLastLowCamTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval))
		{
			addLowFrequencyContainer2(cam, par("pathHistoryLength"));
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

	SimTime MisbehaviorCaService::genCamDcc()
	{
		// network interface may not be ready yet during initialization, so look it up at this later point
		auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
		vanetza::dcc::TransmitRateThrottle *trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
		if (!trc)
		{
			throw cRuntimeError("No DCC TRC found for CA's primary channel %i", mPrimaryChannel);
		}

		static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
		vanetza::Clock::duration interval = trc->interval(ca_tx);
		SimTime dcc{std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS};
		return std::min(mGenCamMax, std::max(mGenCamMin, dcc));
	}

	vanetza::asn1::Cam MisbehaviorCaService::createBenignCAM(const VehicleDataProvider &vdp, uint16_t genDeltaTime)
	{
		vanetza::asn1::Cam message;

		ItsPduHeader_t &header = (*message).header;
		header.protocolVersion = 2;
		header.messageID = ItsPduHeader__messageID_cam;
		header.stationID = vdp.station_id();

		CoopAwareness_t &cam = (*message).cam;
		cam.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
		BasicContainer_t &basic = cam.camParameters.basicContainer;
		HighFrequencyContainer_t &hfc = cam.camParameters.highFrequencyContainer;

		basic.stationType = StationType_passengerCar;
		basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
		basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
		basic.referencePosition.longitude = round(vdp.longitude(), microdegree2) * Longitude_oneMicrodegreeEast;
		basic.referencePosition.latitude = round(vdp.latitude(), microdegree2) * Latitude_oneMicrodegreeNorth;
		basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
		basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
			SemiAxisLength_unavailable;
		basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
			SemiAxisLength_unavailable;

		hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
		BasicVehicleContainerHighFrequency &bvc = hfc.choice.basicVehicleContainerHighFrequency;
		bvc.heading.headingValue = round(vdp.heading(), decidegree2);
		bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
		bvc.speed.speedValue = buildSpeedValue2(vdp.speed());
		bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
		bvc.driveDirection = vdp.speed().value() >= 0.0 ? DriveDirection_forward : DriveDirection_backward;
		const double lonAccelValue = vdp.acceleration() / vanetza::units::si::meter_per_second_squared;
		// extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
		if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0)
		{
			bvc.longitudinalAcceleration.longitudinalAccelerationValue = lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
		}
		else
		{
			bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
		}
		bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
		bvc.curvature.curvatureValue = abs(vdp.curvature() / vanetza::units::reciprocal_metre) * 10000.0;
		if (bvc.curvature.curvatureValue >= 1023)
		{
			bvc.curvature.curvatureValue = 1023;
		}
		bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
		bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
		bvc.yawRate.yawRateValue = round(vdp.yaw_rate(), degree_per_second2) * YawRateValue_degSec_000_01ToLeft * 100.0;
		if (bvc.yawRate.yawRateValue < -32766 || bvc.yawRate.yawRateValue > 32766)
		{
			bvc.yawRate.yawRateValue = YawRateValue_unavailable;
		}
		bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
		bvc.vehicleLength.vehicleLengthConfidenceIndication =
			VehicleLengthConfidenceIndication_noTrailerPresent;
		bvc.vehicleWidth = VehicleWidth_unavailable;

		std::string error;
		if (!message.validate(error))
		{
			throw cRuntimeError("Invalid High Frequency CAM: %s", error.c_str());
		}

		return message;
	}

	vanetza::asn1::Cam MisbehaviorCaService::createAttackCAM(const VehicleDataProvider &vdp, uint16_t genDeltaTime)
	{
		vanetza::asn1::Cam message = createBenignCAM(vdp, genDeltaTime);

		switch (mAttackType)
		{
		case attackTypes::ConstPos:
		{
			message->cam.camParameters.basicContainer.referencePosition.latitude = AttackConstantPositionLatitudeMicrodegrees * Latitude_oneMicrodegreeNorth;
			message->cam.camParameters.basicContainer.referencePosition.longitude = AttackConstantPositionLongitudeMicrodegrees * Longitude_oneMicrodegreeEast;
			break;
		}
		case attackTypes::ConstPosOffset:
		{
			message->cam.camParameters.basicContainer.referencePosition.latitude =
				(round(vdp.latitude(), microdegree2) + AttackConstantPositionOffsetLatitudeMicrodegrees) * Latitude_oneMicrodegreeNorth;
			message->cam.camParameters.basicContainer.referencePosition.longitude =
				(round(vdp.longitude(), microdegree2) + AttackConstantPositionOffsetLongitudeMicrodegrees) * Longitude_oneMicrodegreeEast;
			break;
		}
		case attackTypes::RandomPos:
		{
			long attackLatitude = uniform(-AttackRandomPositionMinLatitude, AttackRandomPositionMaxLatitude) * 1000000;
			long attackLongitude = uniform(-AttackRandomPositionMinLongitude, AttackRandomPositionMaxLongitude) * 1000000;
			message->cam.camParameters.basicContainer.referencePosition.latitude = attackLatitude * Latitude_oneMicrodegreeNorth;
			message->cam.camParameters.basicContainer.referencePosition.longitude = attackLongitude * Longitude_oneMicrodegreeEast;
			break;
		}
		case attackTypes::RandomPosOffset:
		{
			long attackLatitudeOffset = uniform(-AttackRandomPositionOffsetMaxLatitudeOffset, AttackRandomPositionOffsetMaxLatitudeOffset) * 1000000;
			long attackLongitudeOffset = uniform(-AttackRandomPositionOffsetMaxLongitudeOffset, AttackRandomPositionOffsetMaxLongitudeOffset) * 1000000;
			message->cam.camParameters.basicContainer.referencePosition.latitude =
				(round(vdp.latitude(), microdegree2) + attackLatitudeOffset) * Latitude_oneMicrodegreeNorth;
			message->cam.camParameters.basicContainer.referencePosition.longitude =
				(round(vdp.longitude(), microdegree2) + attackLongitudeOffset) * Longitude_oneMicrodegreeEast;
			break;
		}
		case attackTypes::ConstSpeed:
		{
			message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = AttackConstantSpeedValue;
			break;
		}
		case attackTypes::ConstSpeedOffset:
		{
			message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = buildSpeedValue2(vdp.speed() + AttackConstantSpeedOffsetValue);
			break;
		}
		case attackTypes::RandomSpeed:
		{
			double randomSpeed = uniform(AttackRandomSpeedMin, AttackRandomSpeedMax);
			message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = buildSpeedValue2(randomSpeed * boost::units::si::meter_per_second);
			break;
		}
		case attackTypes::RandomSpeedOffset:
		{
			message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = buildSpeedValue2(vdp.speed() + (uniform(0, AttackRandomSpeedOffsetMax) * boost::units::si::meter_per_second));
			break;
		}
		case attackTypes::EventualStop:
		{
			if (attackEventualStopHasStopped)
			{
				message->cam.camParameters.basicContainer.referencePosition = attackEventualStopPosition;
				message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = 0;
				message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = 0;
			}
			else if (AttackEventualStopProbabilityThreshold < uniform(0, 1))
			{
				attackEventualStopHasStopped = true;
				attackEventualStopPosition = message->cam.camParameters.basicContainer.referencePosition;
				message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = 0;
				message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = 0;
			}
			break;
		}
		case attackTypes::Disruptive:
		{
			if (disruptiveMessageQueue.size() >= AttackDisruptiveMinimumReceived)
			{
				int index = uniform(0, disruptiveMessageQueue.size());
				auto it = disruptiveMessageQueue.begin();
				std::advance(it, index);
				message = *it;
				message->cam.generationDeltaTime = (uint16_t)countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
				message->header.stationID = vdp.getStationId();
			}
			break;
		}
		case attackTypes::DataReplay:
		{
			break;
		}
		case attackTypes::StaleMessages:
		{
			staleMessageQueue.push(message);
			if (staleMessageQueue.size() >= AttackStaleDelayCount)
			{

				message = staleMessageQueue.front();
				message->cam.generationDeltaTime = (uint16_t)countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
				staleMessageQueue.pop();
			}
			else
			{
				vanetza::asn1::Cam emptyMessage;
				return emptyMessage;
			}
			break;
		}
		case attackTypes::DoS:
		{
			EV_INFO << "Sent DoS " << simTime().inUnit(SimTimeUnit::SIMTIME_MS) << " old:  " << message->cam.generationDeltaTime << " new: " << (uint16_t)countTaiMilliseconds(mTimer->getTimeFor(simTime())) << "\n";
			message->cam.generationDeltaTime = (uint16_t)countTaiMilliseconds(mTimer->getTimeFor(simTime()));
			break;
		}
		case attackTypes::DoSRandom:
		{
			break;
		}
		case attackTypes::DoSDisruptive:
		{
			break;
		}
		case attackTypes::GridSybil:
		{
			break;
		}
		case attackTypes::DataReplaySybil:
		{
			break;
		}
		case attackTypes::DoSRandomSybil:
		{
			break;
		}
		case attackTypes::DoSDisruptiveSybil:
		{
			break;
		}
		case attackTypes::MAStress:
		{
			break;
		}

		default:
			EV_ERROR << "invalid attack type \n";
			break;
		}
		return message;
	}

	void addLowFrequencyContainer2(vanetza::asn1::Cam &message, unsigned pathHistoryLength)
	{
		if (pathHistoryLength > 40)
		{
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

		for (unsigned i = 0; i < pathHistoryLength; ++i)
		{
			PathPoint *pathPoint = vanetza::asn1::allocate<PathPoint>();
			pathPoint->pathDeltaTime = vanetza::asn1::allocate<PathDeltaTime_t>();
			*(pathPoint->pathDeltaTime) = 0;
			pathPoint->pathPosition.deltaLatitude = DeltaLatitude_unavailable;
			pathPoint->pathPosition.deltaLongitude = DeltaLongitude_unavailable;
			pathPoint->pathPosition.deltaAltitude = DeltaAltitude_unavailable;
			ASN_SEQUENCE_ADD(&bvc.pathHistory, pathPoint);
		}

		std::string error;
		if (!message.validate(error))
		{
			throw cRuntimeError("Invalid Low Frequency CAM: %s", error.c_str());
		}
	}

} // namespace artery
