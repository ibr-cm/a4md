/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/application/CaObject.h"
#include "artery/application/md/MisbehaviorCaService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/utility/simtime_cast.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <chrono>
#include <omnetpp/ccomponent.h>
#include <artery/traci/Cast.h>

#include "artery/application/md/MisbehaviorDetectionService.h"


using namespace omnetpp;
namespace artery {

    namespace {
        auto microdegree = vanetza::units::degree * boost::units::si::micro;
        auto decidegree = vanetza::units::degree * boost::units::si::deci;
        auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
        auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

        static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
        static const simsignal_t scSignalCamSent = cComponent::registerSignal("CamSent");
    }
    std::shared_ptr<const traci::API> MisbehaviorCaService::mTraciAPI;
    traci::Boundary MisbehaviorCaService::mSimulationBoundary;
    GlobalEnvironmentModel *MisbehaviorCaService::mGlobalEnvironmentModel;

    bool MisbehaviorCaService::staticInitializationComplete = false;


    Define_Module(MisbehaviorCaService)

    template<typename T, typename U>
    long MisbehaviorCaService::round(const boost::units::quantity<T> &q, const U &u) {
        boost::units::quantity<U> v{q};
        return std::round(v.value());
    }

    MisbehaviorCaService::~MisbehaviorCaService() {
        MisbehaviorDetectionService::removeStationIdFromVehicleList(mVehicleDataProvider->getStationId());

        while (!activePoIs.empty()) {
            traciPoiScope->remove(activePoIs.front());
            activePoIs.pop_front();
        }
    }

    void MisbehaviorCaService::initialize() {
        BaseCaService::initialize();

        mLocalEnvironmentModel = getFacilities().get_mutable_ptr<LocalEnvironmentModel>();

        if (!staticInitializationComplete) {
            staticInitializationComplete = true;
            mGlobalEnvironmentModel = mLocalEnvironmentModel->getGlobalEnvMod();
            mTraciAPI = getFacilities().get_const<traci::VehicleController>().getTraCI();
            mSimulationBoundary = traci::Boundary{mTraciAPI->simulation.getNetBoundary()};
            traciPoiScope = &mTraciAPI->poi;
            traciVehicleScope = (TraCIAPI::VehicleScope *) &mTraciAPI->vehicle;
            initializeParameters();
        }


        mMisbehaviorType = misbehaviorTypes::LocalAttacker;

        semiMajorConfidence = par("semiMajorConfidence");
        semiMinorConfidence = par("semiMinorConfidence");
        if (semiMajorConfidence >= 4094) {
            semiMajorConfidence = SemiAxisLength_outOfRange;
        }
        if (semiMinorConfidence > 4094) {
            semiMinorConfidence = SemiAxisLength_outOfRange;
        }
        semiMajorOrientationOffset = semiMinorConfidence > semiMajorConfidence ? 90 : 0;

        std::cout << "semiMajorConfidence: " << semiMajorConfidence << std::endl;
        std::cout << "semiMinorConfidence: " << semiMinorConfidence << std::endl;


        //Constant Position Attack
        AttackConstantPositionLatitudeMicrodegrees =
                (long) (uniform(F2MDParameters::attackParameters.AttackConstantPositionMinLatitude,
                                F2MDParameters::attackParameters.AttackConstantPositionMaxLatitude) * 1000000);
        AttackConstantPositionLongitudeMicrodegrees =
                (long) (uniform(F2MDParameters::attackParameters.AttackConstantPositionMinLongitude,
                                F2MDParameters::attackParameters.AttackConstantPositionMaxLongitude) * 1000000);

        //Constant Position Offset Attack
        AttackConstantPositionOffsetLatitudeMicrodegrees = (long) (uniform(
                -F2MDParameters::attackParameters.AttackConstantPositionOffsetMaxLatitudeOffset,
                F2MDParameters::attackParameters.AttackConstantPositionOffsetMaxLatitudeOffset) * 1000000);
        AttackConstantPositionOffsetLongitudeMicrodegrees = (long) (uniform(
                -F2MDParameters::attackParameters.AttackConstantPositionOffsetMaxLongitudeOffset,
                F2MDParameters::attackParameters.AttackConstantPositionOffsetMaxLongitudeOffset) * 1000000);

        // Constant Speed Attack
        AttackConstantSpeedValue = buildSpeedValue(
                uniform(F2MDParameters::attackParameters.AttackConstantSpeedMin,
                        F2MDParameters::attackParameters.AttackConstantSpeedMax) *
                boost::units::si::meter_per_second);

        //Constant Speed Offset Attack
        AttackConstantSpeedOffsetValue =
                ((long) uniform(0, F2MDParameters::attackParameters.AttackConstantSpeedOffsetMax)) *
                boost::units::si::meter_per_second;

        // Eventual Stop Attack
        attackEventualStopHasStopped = false;

        // Data Replay Attack
        attackDataReplayCurrentStationId = -1;

        // Grid Sybil Attack
        attackGridSybilVehicleCount = intuniform(F2MDParameters::attackParameters.AttackGridSybilVehicleCount -
                                                 F2MDParameters::attackParameters.AttackGridSybilVehicleCountVariation,
                                                 F2MDParameters::attackParameters.AttackGridSybilVehicleCount +
                                                 F2MDParameters::attackParameters.AttackGridSybilVehicleCountVariation);
        if (attackGridSybilVehicleCount < 1) {
            attackGridSybilVehicleCount = 1;
        }
        for (int i = 0; i < attackGridSybilVehicleCount; i++) {
            StationType_t stationId = Identity::randomStationId(getRNG(0));
            mPseudonyms.push(stationId);
            MisbehaviorDetectionService::addStationIdToVehicleList(stationId,
                                                                   mMisbehaviorType);

        }
        attackGridSybilCurrentVehicleIndex = 0;
        attackGridSybilActualDistanceX = uniform(F2MDParameters::attackParameters.AttackGridSybilDistanceX -
                                                 F2MDParameters::attackParameters.AttackGridSybilDistanceVariation,
                                                 F2MDParameters::attackParameters.AttackGridSybilDistanceX +
                                                 F2MDParameters::attackParameters.AttackGridSybilDistanceVariation);
        attackGridSybilActualDistanceY = uniform(F2MDParameters::attackParameters.AttackGridSybilDistanceY -
                                                 F2MDParameters::attackParameters.AttackGridSybilDistanceVariation,
                                                 F2MDParameters::attackParameters.AttackGridSybilDistanceY +
                                                 F2MDParameters::attackParameters.AttackGridSybilDistanceVariation);


        mAttackType = attackTypes::AttackTypes(F2MDParameters::attackParameters.StaticAttackType);

        par("AttackType").setStringValue(attackTypes::AttackNames[mAttackType]);

        traciVehicleScope->setColor(mVehicleController->getVehicleId(), libsumo::TraCIColor(255, 0, 0, 255));

        if (mAttackType == attackTypes::DoS || mAttackType == attackTypes::GridSybil ||
            mAttackType == attackTypes::DataReplaySybil || mAttackType == attackTypes::DoSDisruptiveSybil ||
            mAttackType == attackTypes::DoSRandomSybil) {
            mGenCamMin = {F2MDParameters::attackParameters.AttackDoSInterval, SIMTIME_MS};
            mDccRestriction = !F2MDParameters::attackParameters.AttackDoSIgnoreDCC;
            mFixedRate = true;
        }
        MisbehaviorDetectionService::addStationIdToVehicleList(mVehicleDataProvider->getStationId(), mMisbehaviorType);
    }

    void MisbehaviorCaService::initializeParameters() {
        // CAM Location Visualizer (PoI)
        F2MDParameters::miscParameters.CamLocationVisualizer = par("CamLocationVisualizer");
        F2MDParameters::miscParameters.CamLocationVisualizerMaxLength = par("CamLocationVisualizerMaxLength");

        F2MDParameters::attackParameters.StaticAttackType = par("StaticAttackType");

        // Constant Position Attack
        F2MDParameters::attackParameters.AttackConstantPositionMinLatitude = par("AttackConstantPositionMinLatitude");
        F2MDParameters::attackParameters.AttackConstantPositionMaxLatitude = par("AttackConstantPositionMaxLatitude");
        F2MDParameters::attackParameters.AttackConstantPositionMinLongitude = par("AttackConstantPositionMinLongitude");
        F2MDParameters::attackParameters.AttackConstantPositionMaxLongitude = par("AttackConstantPositionMaxLongitude");

        // Constant Position Offset Attack
        F2MDParameters::attackParameters.AttackConstantPositionOffsetMaxLatitudeOffset = par(
                "AttackConstantPositionOffsetMaxLatitudeOffset");
        F2MDParameters::attackParameters.AttackConstantPositionOffsetMaxLongitudeOffset = par(
                "AttackConstantPositionOffsetMaxLongitudeOffset");

        // Random Position Attack
        F2MDParameters::attackParameters.AttackRandomPositionMinLatitude = par("AttackRandomPositionMinLatitude");
        F2MDParameters::attackParameters.AttackRandomPositionMaxLatitude = par("AttackRandomPositionMaxLatitude");
        F2MDParameters::attackParameters.AttackRandomPositionMinLongitude = par("AttackRandomPositionMinLongitude");
        F2MDParameters::attackParameters.AttackRandomPositionMaxLongitude = par("AttackRandomPositionMaxLongitude");

        // Random Position Offset Attack
        F2MDParameters::attackParameters.AttackRandomPositionOffsetMaxLatitudeOffset = par(
                "AttackRandomPositionOffsetMaxLatitudeOffset");
        F2MDParameters::attackParameters.AttackRandomPositionOffsetMaxLongitudeOffset = par(
                "AttackRandomPositionOffsetMaxLongitudeOffset");

        // Constant Speed Attack
        // Meters per Second
        F2MDParameters::attackParameters.AttackConstantSpeedMin = par("AttackConstantSpeedMin");
        F2MDParameters::attackParameters.AttackConstantSpeedMax = par("AttackConstantSpeedMax");

        // Constant Speed Offset Attack
        F2MDParameters::attackParameters.AttackConstantSpeedOffsetMax = par("AttackConstantSpeedOffsetMax");

        // Random Speed Attack
        // Meters per Second
        F2MDParameters::attackParameters.AttackRandomSpeedMin = par("AttackRandomSpeedMin");
        F2MDParameters::attackParameters.AttackRandomSpeedMax = par("AttackRandomSpeedMax");

        // Random Speed Offset Attack
        F2MDParameters::attackParameters.AttackRandomSpeedOffsetMax = par("AttackRandomSpeedOffsetMax");

        // Eventual Stop Attack
        F2MDParameters::attackParameters.AttackEventualStopProbabilityThreshold = par(
                "AttackEventualStopProbabilityThreshold");

        // Disruptive Attack
        F2MDParameters::attackParameters.AttackDisruptiveBufferSize = par("AttackDisruptiveBufferSize");
        F2MDParameters::attackParameters.AttackDisruptiveMinimumReceived = par("AttackDisruptiveMinimumReceived");

        // Denial of Service Attack
        F2MDParameters::attackParameters.AttackDoSInterval = par("AttackDoSInterval");
        F2MDParameters::attackParameters.AttackDoSIgnoreDCC = par("AttackDoSIgnoreDCC");

        // Stale Messages Attack
        F2MDParameters::attackParameters.AttackStaleDelayCount = par("AttackStaleDelayCount");

        // Grid Sybil Attack
        F2MDParameters::attackParameters.AttackGridSybilVehicleCount = par("AttackGridSybilVehicleCount");
        F2MDParameters::attackParameters.AttackGridSybilVehicleCountVariation = par(
                "AttackGridSybilVehicleCountVariation");
        F2MDParameters::attackParameters.AttackGridSybilSelfSybil = par("AttackGridSybilSelfSybil");
        F2MDParameters::attackParameters.AttackGridSybilDistanceX = par("AttackGridSybilDistanceX");
        F2MDParameters::attackParameters.AttackGridSybilDistanceY = par("AttackGridSybilDistanceY");
        F2MDParameters::attackParameters.AttackGridSybilDistanceVariation = par("AttackGridSybilDistanceVariation");
        F2MDParameters::attackParameters.AttackGridSybilMaxDistanceFromRoad = par("AttackGridSybilMaxDistanceFromRoad");
    }

    void MisbehaviorCaService::trigger() {
        Enter_Method("trigger");
        const SimTime &T_now = simTime();
        if (checkTriggeringConditions(T_now)) {
            sendCam(T_now);
        }
    }

    void
    MisbehaviorCaService::indicate(const vanetza::btp::DataIndication &ind, std::unique_ptr<vanetza::UpPacket> packet) {
        Enter_Method("indicate");

        Asn1PacketVisitor<vanetza::asn1::Cam> visitor;
        const vanetza::asn1::Cam *cam = boost::apply_visitor(visitor, *packet);
        if (cam && cam->validate()) {
            CaObject obj = visitor.shared_wrapper;
            emit(scSignalCamReceived, &obj);
            mLocalDynamicMap->updateAwareness(obj);
            vanetza::asn1::Cam message = *cam;
            switch (mAttackType) {
                case attackTypes::Disruptive: {
                    disruptiveMessageQueue.emplace_back(message);
                    if (disruptiveMessageQueue.size() > F2MDParameters::attackParameters.AttackDisruptiveBufferSize) {
                        disruptiveMessageQueue.pop_front();
                    }
                    break;
                }
                case attackTypes::DataReplay:
                case attackTypes::DoSDisruptive:
                case attackTypes::GridSybil: {
                    receivedMessages[message->header.stationID].push(message);
                    break;
                }
                default:
                    break;
            }
            if (mAttackType == attackTypes::Disruptive) {

            }
        }
    }

    void MisbehaviorCaService::visualizeCamPosition(vanetza::asn1::Cam cam) {

        std::vector<libsumo::TraCIColor> colors = {libsumo::TraCIColor(255, 0, 255, 255),
                                                   libsumo::TraCIColor(207, 255, 0, 255),
                                                   libsumo::TraCIColor(255, 155, 155, 255),
                                                   libsumo::TraCIColor(0, 140, 255, 255),
                                                   libsumo::TraCIColor(0, 255, 162, 255)};
        libsumo::TraCIColor color = libsumo::TraCIColor(255, 0, 255, 255);
        int maxActivePoIs = F2MDParameters::miscParameters.CamLocationVisualizerMaxLength;
        if (mAttackType == attackTypes::GridSybil && attackGridSybilVehicleCount <= 5) {
            color = colors[(attackGridSybilCurrentVehicleIndex + (attackGridSybilVehicleCount - 1)) %
                           attackGridSybilVehicleCount];
            maxActivePoIs = attackGridSybilVehicleCount;
        }
        traci::TraCIGeoPosition traciGeoPosition = {
                (double) cam->cam.camParameters.basicContainer.referencePosition.longitude / 10000000.0,
                (double) cam->cam.camParameters.basicContainer.referencePosition.latitude / 10000000.0};
        traci::TraCIPosition traciPosition = mVehicleController->getTraCI()->convert2D(traciGeoPosition);
        std::string poiId = {
                mVehicleController->getVehicleId() + "_CAM_" + std::to_string(cam->header.stationID) + "_" +
                std::to_string(cam->cam.generationDeltaTime)};
        traciPoiScope->add(poiId, traciPosition.x, traciPosition.y, color,
                           poiId, 5, "", 0,
                           0, 0);
        activePoIs.push_back(poiId);
        if (activePoIs.size() > maxActivePoIs) {
            traciPoiScope->remove(activePoIs.front());
            activePoIs.pop_front();
        }
        if (mAttackType != attackTypes::GridSybil) {
            int alphaStep = 185 / maxActivePoIs;
            int currentAlpha = 80;
            for (const auto &poi : activePoIs) {
                traciPoiScope->setColor(poi, libsumo::TraCIColor(255, 0, 255, currentAlpha));
                currentAlpha += alphaStep;
            }
        }
    }

    void MisbehaviorCaService::sendCam(const SimTime &T_now) {
        uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
        vanetza::asn1::Cam cam;
        switch (mMisbehaviorType) {
            case misbehaviorTypes::Benign:
                cam = createCooperativeAwarenessMessage(*mVehicleDataProvider, *mVehicleController, genDeltaTimeMod);
                break;
            case misbehaviorTypes::LocalAttacker: {
                cam = createAttackCAM(genDeltaTimeMod);
                break;
            }
            case misbehaviorTypes::GlobalAttacker:
                cam = createAttackCAM(genDeltaTimeMod);
                break;
            default:
                cam = createCooperativeAwarenessMessage(*mVehicleDataProvider, *mVehicleController, genDeltaTimeMod);
        }
        if (cam->header.messageID != 2) {
            return;
        }
        if (F2MDParameters::miscParameters.CamLocationVisualizer) {
            visualizeCamPosition(cam);
        }

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

    long setValueToRange(long value, long lower, long upper) {
        if (value <= lower) {
            return lower;
        } else if (value >= upper) {
            return upper;
        } else {
            return value;
        }
    }

    void
    addOffsetToCamParameter(cRNG *rng, long *camParameter, long unavailabilityValue, double variance, long lower,
                            long upper) {
        if (*camParameter < unavailabilityValue) {
            int offset = intuniform(rng, (int) (-variance * (double) *camParameter),
                                    (int) (+variance * (double) *camParameter));
            *camParameter = (long) setValueToRange((int) (*camParameter + offset), lower, upper);
        }
    }

    vanetza::asn1::Cam MisbehaviorCaService::getNextReplayCam() {
        vanetza::asn1::Cam message;
        if (attackDataReplayCurrentStationId == -1) {
            auto it = receivedMessages.begin();
            if (it != receivedMessages.end()) {
                uint32_t mostReceivedStationId = -1;
                unsigned long maxSize = 0;
                for (; it != receivedMessages.end(); ++it) {
                    if (receivedMessages[it->first].size() > maxSize) {
                        maxSize = receivedMessages[it->first].size();
                        mostReceivedStationId = it->first;
                    }
                }
                attackDataReplayCurrentStationId = mostReceivedStationId;
                message = receivedMessages[attackDataReplayCurrentStationId].front();
                receivedMessages[attackDataReplayCurrentStationId].pop();
            } else {
                message = vanetza::asn1::Cam();
            }
        } else {
            if (!receivedMessages[attackDataReplayCurrentStationId].empty() &&
                receivedMessages[attackDataReplayCurrentStationId].front()->cam.generationDeltaTime <
                (uint16_t) (countTaiMilliseconds(mTimer->getCurrentTime()) - 1100)) {
                message = receivedMessages[attackDataReplayCurrentStationId].front();
                receivedMessages[attackDataReplayCurrentStationId].pop();
            } else {
                receivedMessages.erase(attackDataReplayCurrentStationId);
                attackDataReplayCurrentStationId = -1;
                message = vanetza::asn1::Cam();
            }
        }
        return message;
    }


    vanetza::asn1::Cam MisbehaviorCaService::createAttackCAM(uint16_t genDeltaTime) {
        vanetza::asn1::Cam message = createCooperativeAwarenessMessage(*mVehicleDataProvider, *mVehicleController,
                                                                       genDeltaTime);

        switch (mAttackType) {
            case attackTypes::ConstPos: {
                message->cam.camParameters.basicContainer.referencePosition.latitude =
                        AttackConstantPositionLatitudeMicrodegrees * Latitude_oneMicrodegreeNorth;
                message->cam.camParameters.basicContainer.referencePosition.longitude =
                        AttackConstantPositionLongitudeMicrodegrees * Longitude_oneMicrodegreeEast;
                break;
            }
            case attackTypes::ConstPosOffset: {
                message->cam.camParameters.basicContainer.referencePosition.latitude =
                        (round(mVehicleDataProvider->latitude(), microdegree) +
                         AttackConstantPositionOffsetLatitudeMicrodegrees) *
                        Latitude_oneMicrodegreeNorth;
                message->cam.camParameters.basicContainer.referencePosition.longitude =
                        (round(mVehicleDataProvider->longitude(), microdegree) +
                         AttackConstantPositionOffsetLongitudeMicrodegrees) *
                        Longitude_oneMicrodegreeEast;
                break;
            }
            case attackTypes::RandomPos: {
                long attackLatitude =
                        (long) (uniform(-F2MDParameters::attackParameters.AttackRandomPositionMinLatitude,
                                        F2MDParameters::attackParameters.AttackRandomPositionMaxLatitude) * 1000000);
                long attackLongitude =
                        (long) (uniform(-F2MDParameters::attackParameters.AttackRandomPositionMinLongitude,
                                        F2MDParameters::attackParameters.AttackRandomPositionMaxLongitude) * 1000000);
                message->cam.camParameters.basicContainer.referencePosition.latitude =
                        attackLatitude * Latitude_oneMicrodegreeNorth;
                message->cam.camParameters.basicContainer.referencePosition.longitude =
                        attackLongitude * Longitude_oneMicrodegreeEast;
                break;
            }
            case attackTypes::RandomPosOffset: {
                long attackLatitudeOffset = (long) (
                        uniform(-F2MDParameters::attackParameters.AttackRandomPositionOffsetMaxLatitudeOffset,
                                F2MDParameters::attackParameters.AttackRandomPositionOffsetMaxLatitudeOffset) *
                        1000000);
                long attackLongitudeOffset = (long) (
                        uniform(-F2MDParameters::attackParameters.AttackRandomPositionOffsetMaxLongitudeOffset,
                                F2MDParameters::attackParameters.AttackRandomPositionOffsetMaxLongitudeOffset) *
                        1000000);
                message->cam.camParameters.basicContainer.referencePosition.latitude =
                        (round(mVehicleDataProvider->latitude(), microdegree) + attackLatitudeOffset) *
                        Latitude_oneMicrodegreeNorth;
                message->cam.camParameters.basicContainer.referencePosition.longitude =
                        (round(mVehicleDataProvider->longitude(), microdegree) + attackLongitudeOffset) *
                        Longitude_oneMicrodegreeEast;
                break;
            }
            case attackTypes::ConstSpeed: {
                message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = AttackConstantSpeedValue;
                break;
            }
            case attackTypes::ConstSpeedOffset: {
                message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = buildSpeedValue(
                        mVehicleDataProvider->speed() + AttackConstantSpeedOffsetValue);
                break;
            }
            case attackTypes::RandomSpeed: {
                double randomSpeed = uniform(F2MDParameters::attackParameters.AttackRandomSpeedMin,
                                             F2MDParameters::attackParameters.AttackRandomSpeedMax);
                message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = buildSpeedValue(
                        randomSpeed * boost::units::si::meter_per_second);
                break;
            }
            case attackTypes::RandomSpeedOffset: {
                message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = buildSpeedValue(
                        mVehicleDataProvider->speed() +
                        (uniform(0, F2MDParameters::attackParameters.AttackRandomSpeedOffsetMax) *
                         boost::units::si::meter_per_second));
                break;
            }
            case attackTypes::EventualStop: {
                if (attackEventualStopHasStopped) {
                    message->cam.camParameters.basicContainer.referencePosition = attackEventualStopPosition;
                    message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = 0;
                    message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = 0;
                } else {
                    if (F2MDParameters::attackParameters.AttackEventualStopProbabilityThreshold > uniform(0, 1)) {
                        attackEventualStopHasStopped = true;
                        attackEventualStopPosition = message->cam.camParameters.basicContainer.referencePosition;
                        message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = 0;
                        message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = 0;
                    } else {
                    }
                }
                break;
            }
            case attackTypes::Disruptive: {
                if (disruptiveMessageQueue.size() >= F2MDParameters::attackParameters.AttackDisruptiveMinimumReceived) {
                    int index = (int) uniform(0, (double) disruptiveMessageQueue.size());
                    auto it = disruptiveMessageQueue.begin();
                    std::advance(it, index);
                    message = *it;
                    message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                            mTimer->getTimeFor(mVehicleDataProvider->updated()));
                    message->header.stationID = mVehicleDataProvider->getStationId();
                } else {
                    message = vanetza::asn1::Cam();
                }
                break;
            }
            case attackTypes::DataReplay: {
                message = getNextReplayCam();
                if (message->header.messageID != 2) {
                    break;
                } else {
                    message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                            mTimer->getTimeFor(mVehicleDataProvider->updated()));
                    message->header.stationID = mVehicleDataProvider->getStationId();
                }
                break;
            }
            case attackTypes::StaleMessages: {
                staleMessageQueue.push(message);
                if (staleMessageQueue.size() >= F2MDParameters::attackParameters.AttackStaleDelayCount) {

                    message = staleMessageQueue.front();
                    message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                            mTimer->getTimeFor(mVehicleDataProvider->updated()));
                    staleMessageQueue.pop();
                } else {
                    message = vanetza::asn1::Cam();
                }
                break;
            }
            case attackTypes::DoS: {
                message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(mTimer->getTimeFor(simTime()));
                break;
            }
            case attackTypes::DoSRandom: {
                message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(mTimer->getTimeFor(simTime()));
                long attackLatitude =
                        (long) (uniform(-F2MDParameters::attackParameters.AttackRandomPositionMinLatitude,
                                        F2MDParameters::attackParameters.AttackRandomPositionMaxLatitude) * 1000000);
                long attackLongitude =
                        (long) (uniform(-F2MDParameters::attackParameters.AttackRandomPositionMinLongitude,
                                        F2MDParameters::attackParameters.AttackRandomPositionMaxLongitude) * 1000000);
                message->cam.camParameters.basicContainer.referencePosition.latitude =
                        attackLatitude * Latitude_oneMicrodegreeNorth;
                message->cam.camParameters.basicContainer.referencePosition.longitude =
                        attackLongitude * Longitude_oneMicrodegreeEast;
                double randomSpeed = uniform(F2MDParameters::attackParameters.AttackRandomSpeedMin,
                                             F2MDParameters::attackParameters.AttackRandomSpeedMax);
                message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = buildSpeedValue(
                        randomSpeed * boost::units::si::meter_per_second);
                break;
            }
            case attackTypes::DoSDisruptive: {
                auto it = receivedMessages.begin();
                int index = (int) uniform(0, (double) receivedMessages.size());
                std::advance(it, index);
                if (!it->second.empty()) {
                    message = it->second.front();
                    it->second.pop();
                    message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                            mTimer->getTimeFor(mVehicleDataProvider->updated()));
                    message->header.stationID = mVehicleDataProvider->getStationId();
                } else {
                    receivedMessages.erase(it->first);
                    message = vanetza::asn1::Cam();
                }
                break;
            }
            case attackTypes::GridSybil: {
                double offsetX;
                double offsetY;
                double currentHeadingAngle;
                Position originalPosition;
                if (F2MDParameters::attackParameters.AttackGridSybilSelfSybil) {
                    offsetX =
                            -((double) attackGridSybilCurrentVehicleIndex / 2) *
                            (mVehicleController->getWidth().value() + attackGridSybilActualDistanceX) +
                            uniform(-attackGridSybilActualDistanceX / 10, attackGridSybilActualDistanceX / 10);
                    offsetY =
                            -((double) (attackGridSybilCurrentVehicleIndex % 2)) *
                            (mVehicleController->getLength().value() + attackGridSybilActualDistanceY) +
                            uniform(-attackGridSybilActualDistanceY / 10, attackGridSybilActualDistanceY / 10);
                    currentHeadingAngle = artery::Angle::from_radian(
                            mVehicleDataProvider->heading().value()).degree();
                    originalPosition = mVehicleDataProvider->position();
                } else {
                    message = getNextReplayCam();
                    if (message->header.messageID != 2) {
                        break;
                    } else {
                        double width;
                        if (message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth !=
                            VehicleWidth_unavailable) {
                            width = (double) message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth;
                        } else {
                            width = mVehicleController->getWidth().value();
                        }
                        double length;
                        if (message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue !=
                            VehicleLengthValue_unavailable) {
                            length = (double) message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue;
                        } else {
                            length = mVehicleController->getLength().value();
                        }

                        offsetX = -((double) (attackGridSybilCurrentVehicleIndex + 1) / 2) *
                                  (width + attackGridSybilActualDistanceX) +
                                  uniform(-attackGridSybilActualDistanceX / 10, attackGridSybilActualDistanceX / 10);
                        offsetY = -((double) (attackGridSybilCurrentVehicleIndex + 1 % 2)) *
                                  (length + attackGridSybilActualDistanceY) +
                                  uniform(-attackGridSybilActualDistanceY / 10, attackGridSybilActualDistanceY / 10);
                        currentHeadingAngle = artery::Angle::from_radian(
                                (double) message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue /
                                10).degree();
                        originalPosition = LegacyChecks::convertCamPosition(
                                message->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary,
                                mTraciAPI);
                    }
                }
                Position relativePosition = Position(offsetX, offsetY);
                double newAngle = currentHeadingAngle + LegacyChecks::calculateHeadingAngle(relativePosition);
                newAngle = 360 - std::fmod(newAngle, 360);

                double offsetDistance = sqrt(pow(offsetX, 2) + pow(offsetY, 2));
                double relativeX = offsetDistance * sin(newAngle * PI / 180);
                double relativeY = offsetDistance * cos(newAngle * PI / 180);
                Position newPosition = Position(originalPosition.x.value() + relativeX,
                                                originalPosition.y.value() + relativeY);
                if (LegacyChecks::getDistanceToNearestRoad(mGlobalEnvironmentModel, newPosition) >
                    F2MDParameters::attackParameters.AttackGridSybilMaxDistanceFromRoad) {
                    message = vanetza::asn1::Cam();
                    break;
                }

                ReferencePosition_t *referencePosition = &message->cam.camParameters.basicContainer.referencePosition;
                addOffsetToCamParameter(getRNG(0),
                                        &referencePosition->positionConfidenceEllipse.semiMajorConfidence,
                                        SemiAxisLength_unavailable, 0.1, 0, SemiAxisLength_outOfRange);
                addOffsetToCamParameter(getRNG(0),
                                        &referencePosition->positionConfidenceEllipse.semiMinorConfidence,
                                        SemiAxisLength_unavailable, 0.1, 0, SemiAxisLength_outOfRange);
                traci::TraCIGeoPosition traciGeoPos = mTraciAPI->convertGeo(
                        position_cast(mSimulationBoundary, newPosition));
                referencePosition->longitude = (long) (traciGeoPos.longitude * 10000000);
                referencePosition->latitude = (long) (traciGeoPos.latitude * 10000000);

                addOffsetToCamParameter(getRNG(0),
                                        &message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue,
                                        SpeedValue_unavailable, 0.05, 0, 16382);
                addOffsetToCamParameter(getRNG(0),
                                        &message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedConfidence,
                                        SpeedConfidence_unavailable, 0.1, 0, SpeedConfidence_outOfRange);

                addOffsetToCamParameter(getRNG(0),
                                        &message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue,
                                        LongitudinalAccelerationValue_unavailable, 0.1, -160, 160);
                addOffsetToCamParameter(getRNG(0),
                                        &message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence,
                                        AccelerationConfidence_unavailable, 0.1, 0,
                                        AccelerationConfidence_outOfRange);

                double deltaAngle =
                        currentHeadingAngle + uniform(-currentHeadingAngle / 360, +currentHeadingAngle / 360);
                deltaAngle = std::fmod(deltaAngle + 360, 360);

                message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue = round(
                        Angle::from_degree(deltaAngle).value, decidegree);
                addOffsetToCamParameter(getRNG(0),
                                        &message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence,
                                        HeadingConfidence_unavailable, 0.1, 0, HeadingConfidence_outOfRange);

                double steeringAngle = std::fmod(attackGridSybilLastHeadingAngle - currentHeadingAngle, 360);
                steeringAngle = steeringAngle > 180 ? 360 - steeringAngle : steeringAngle;
                attackGridSybilLastHeadingAngle = currentHeadingAngle;
                if (steeringAngle > 5 && attackGridSybilCurrentVehicleIndex > 0) {
                    message = vanetza::asn1::Cam();
                    break;
                }

                attackGridSybilCurrentVehicleIndex = ++attackGridSybilCurrentVehicleIndex % attackGridSybilVehicleCount;
                message->header.stationID = mPseudonyms.front();
                message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                        mTimer->getTimeFor(mVehicleDataProvider->updated()));
                mPseudonyms.push(mPseudonyms.front());
                mPseudonyms.pop();
                break;
            }
            case attackTypes::DataReplaySybil: {
                if (attackDataReplayCurrentStationId == -1) {
                    auto it = receivedMessages.begin();
                    if (it != receivedMessages.end()) {
                        uint32_t mostReceivedStationId = -1;
                        unsigned long maxSize = 0;
                        for (; it != receivedMessages.end(); ++it) {
                            if (receivedMessages[it->first].size() > maxSize) {
                                maxSize = receivedMessages[it->first].size();
                                mostReceivedStationId = it->first;
                            }
                        }
                        attackDataReplayCurrentStationId = mostReceivedStationId;
                        message = receivedMessages[attackDataReplayCurrentStationId].front();
                        receivedMessages[attackDataReplayCurrentStationId].pop();
                        message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                                mTimer->getTimeFor(mVehicleDataProvider->updated()));
                        message->header.stationID = mPseudonyms.front();
                    } else {
                        message = vanetza::asn1::Cam();
                    }
                } else {
                    if (!receivedMessages[attackDataReplayCurrentStationId].empty()) {
                        message = receivedMessages[attackDataReplayCurrentStationId].front();
                        receivedMessages[attackDataReplayCurrentStationId].pop();
                        message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                                mTimer->getTimeFor(mVehicleDataProvider->updated()));
                        message->header.stationID = mPseudonyms.front();
                    } else {
                        receivedMessages.erase(attackDataReplayCurrentStationId);
                        attackDataReplayCurrentStationId = -1;
                        message = vanetza::asn1::Cam();
                        mPseudonyms.push(mPseudonyms.front());
                        mPseudonyms.pop();
                    }
                }
                break;
            }
            case attackTypes::DoSRandomSybil: {
                message->header.stationID = mPseudonyms.front();
                mPseudonyms.push(mPseudonyms.front());
                mPseudonyms.pop();
                message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(mTimer->getTimeFor(simTime()));
                long attackLatitude =
                        (long) (uniform(-F2MDParameters::attackParameters.AttackRandomPositionMinLatitude,
                                        F2MDParameters::attackParameters.AttackRandomPositionMaxLatitude) * 1000000);
                long attackLongitude =
                        (long) (uniform(-F2MDParameters::attackParameters.AttackRandomPositionMinLongitude,
                                        F2MDParameters::attackParameters.AttackRandomPositionMaxLongitude) * 1000000);
                message->cam.camParameters.basicContainer.referencePosition.latitude =
                        attackLatitude * Latitude_oneMicrodegreeNorth;
                message->cam.camParameters.basicContainer.referencePosition.longitude =
                        attackLongitude * Longitude_oneMicrodegreeEast;
                double randomSpeed = uniform(F2MDParameters::attackParameters.AttackRandomSpeedMin,
                                             F2MDParameters::attackParameters.AttackRandomSpeedMax);
                message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = buildSpeedValue(
                        randomSpeed * boost::units::si::meter_per_second);
                break;
            }
            case attackTypes::DoSDisruptiveSybil: {
                auto it = receivedMessages.begin();
                int index = (int) uniform(0, (double) receivedMessages.size());
                std::advance(it, index);
                if (!it->second.empty()) {
                    message = it->second.front();
                    it->second.pop();
                    message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                            mTimer->getTimeFor(mVehicleDataProvider->updated()));
                    message->header.stationID = mPseudonyms.front();
                    mPseudonyms.push(mPseudonyms.front());
                    mPseudonyms.pop();
                } else {
                    receivedMessages.erase(it->first);
                    message = vanetza::asn1::Cam();
                }
                break;
            }
            case attackTypes::MAStress: {
                break;
            }

            default:
                ReferencePosition_t *referencePosition = &message->cam.camParameters.basicContainer.referencePosition;
                double semiMajorOrientation = std::fmod(
                        mVehicleDataProvider->heading().value() + semiMajorOrientationOffset, 360);
                referencePosition->positionConfidenceEllipse.semiMajorOrientation = round(
                        semiMajorOrientation * vanetza::units::degree, decidegree);
                referencePosition->positionConfidenceEllipse.semiMajorConfidence = (long) semiMajorConfidence * 100;
                if (referencePosition->positionConfidenceEllipse.semiMajorConfidence > SemiAxisLength_outOfRange) {
                    referencePosition->positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_outOfRange;
                }
                referencePosition->positionConfidenceEllipse.semiMinorConfidence = (long) semiMinorConfidence * 100;
                if (referencePosition->positionConfidenceEllipse.semiMinorConfidence > SemiAxisLength_outOfRange) {
                    referencePosition->positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_outOfRange;
                }

                double offsetX = sqrt(uniform(0, 1)) * cos(uniform(0, 2 * PI)) * semiMajorConfidence / 2;
                double offsetY = sqrt(uniform(0, 1)) * sin(uniform(0, 2 * PI)) * semiMinorConfidence / 2;
                double currentHeadingAngle = artery::Angle::from_radian(
                        mVehicleDataProvider->heading().value()).degree() + semiMajorOrientationOffset;
                double newAngle = currentHeadingAngle + LegacyChecks::calculateHeadingAngle(Position(offsetX, offsetY));
                newAngle = 360 - std::fmod(newAngle, 360);

                double offsetDistance = sqrt(pow(offsetX, 2) + pow(offsetY, 2));
                double relativeX = offsetDistance * sin(newAngle * PI / 180);
                double relativeY = offsetDistance * cos(newAngle * PI / 180);
                Position originalPosition = mVehicleDataProvider->position();
                Position newPosition = Position(originalPosition.x.value() + relativeX,
                                                originalPosition.y.value() + relativeY);
                traci::TraCIGeoPosition traciGeoPos = mTraciAPI->convertGeo(
                        position_cast(mSimulationBoundary, newPosition));
                referencePosition->longitude = (long) (traciGeoPos.longitude * 10000000);
                referencePosition->latitude = (long) (traciGeoPos.latitude * 10000000);
//                EV_ERROR << "invalid attack type \n";
                break;
        }
        return message;
    }

} // namespace artery
