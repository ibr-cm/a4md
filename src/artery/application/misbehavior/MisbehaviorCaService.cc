/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/application/misbehavior/MisbehaviorCaService.h"
#include "artery/application/misbehavior/util/HelperFunctions.h"
#include "artery/application/misbehavior/report/Report.h"
#include "artery/application/misbehavior/report/MisbehaviorReportObject.h"
#include "artery/application/CaObject.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include <artery/traci/Cast.h>
#include <boost/units/systems/si/prefixes.hpp>

using namespace omnetpp;
namespace artery {

    namespace {
        auto microdegree = vanetza::units::degree * boost::units::si::micro;
        auto decidegree = vanetza::units::degree * boost::units::si::deci;
        auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
        auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

        static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
        static const simsignal_t scSignalMaMisbehaviorAnnouncement = cComponent::registerSignal(
                "misbehaviorAuthority.MisbehaviorAnnouncement");
        static const simsignal_t scSignalMisbehaviorAuthorityNewReport = cComponent::registerSignal(
                "newMisbehaviorReport");
    }

    GlobalEnvironmentModel *MisbehaviorCaService::mGlobalEnvironmentModel;
    bool MisbehaviorCaService::staticInitializationComplete = false;


    Define_Module(MisbehaviorCaService)

    template<typename T, typename U>
    long MisbehaviorCaService::round(const boost::units::quantity<T> &q, const U &u) {
        boost::units::quantity<U> v{q};
        return std::round(v.value());
    }

    MisbehaviorCaService::~MisbehaviorCaService() {
        while (!activePoIs.empty()) {
            mTraciAPI->poi.remove(activePoIs.front());
            activePoIs.pop_front();
        }
        disruptiveMessageQueue.clear();
        if (receivedMessages.empty()) {
            for (auto &sender: receivedMessages) {
                while (!sender.second.empty()) {
                    sender.second.pop_front();
                }
                receivedMessages.erase(sender.first);
            }
        }
    }

    void MisbehaviorCaService::initialize() {
        BaseCaService::initialize();

        mLocalEnvironmentModel = getFacilities().get_mutable_ptr<LocalEnvironmentModel>();

        if (!staticInitializationComplete) {
            staticInitializationComplete = true;
            mGlobalEnvironmentModel = mLocalEnvironmentModel->getGlobalEnvMod();
            initializeStaticParameters();
        }


        mMisbehaviorType = misbehaviorTypes::LocalAttacker;

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
        attackConstantSpeedOffsetValue =
                ((long) uniform(-F2MDParameters::attackParameters.AttackConstantSpeedOffsetMax,
                                F2MDParameters::attackParameters.AttackConstantSpeedOffsetMax)) *
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

        attackGridSybilCurrentVehicleIndex = 0;
        attackGridSybilActualDistanceX = uniform(F2MDParameters::attackParameters.AttackGridSybilDistanceX -
                                                 F2MDParameters::attackParameters.AttackGridSybilDistanceVariation,
                                                 F2MDParameters::attackParameters.AttackGridSybilDistanceX +
                                                 F2MDParameters::attackParameters.AttackGridSybilDistanceVariation);
        attackGridSybilActualDistanceY = uniform(F2MDParameters::attackParameters.AttackGridSybilDistanceY -
                                                 F2MDParameters::attackParameters.AttackGridSybilDistanceVariation,
                                                 F2MDParameters::attackParameters.AttackGridSybilDistanceY +
                                                 F2MDParameters::attackParameters.AttackGridSybilDistanceVariation);


        mAttackType = attackTypes::AttackTypes((int) par("StaticAttackType"));
        par("AttackType").setStringValue(attackTypes::AttackNames[mAttackType]);

        mTraciAPI->vehicle.setColor(mVehicleController->getVehicleId(), libsumo::TraCIColor(255, 0, 0, 255));

        std::vector<StationID_t> stationIds;
        if (mAttackType == attackTypes::GridSybil || mAttackType == attackTypes::DoSRandomSybil ||
            mAttackType == attackTypes::DoSDisruptiveSybil) {
            for (int i = 0; i < attackGridSybilVehicleCount; i++) {
                StationType_t stationId = Identity::randomStationId(getRNG(0));
                mPseudonyms.emplace_back(stationId);
                stationIds.emplace_back(stationId);
            }
        }

        if (mAttackType == attackTypes::DoS || mAttackType == attackTypes::DoSRandom ||
            mAttackType == attackTypes::DoSDisruptive || mAttackType == attackTypes::GridSybil ||
            mAttackType == attackTypes::DataReplaySybil || mAttackType == attackTypes::DoSRandomSybil ||
            mAttackType == attackTypes::DoSDisruptiveSybil) {
            mGenCamMin = {F2MDParameters::attackParameters.AttackDoSInterval, SIMTIME_MS};
            mDccRestriction = !F2MDParameters::attackParameters.AttackDoSIgnoreDCC;
            mFixedRate = true;
        }

        if (mAttackType != attackTypes::Benign) {
            stationIds.emplace_back(mStationId);
            std::vector<StationID_t> *ptr = &stationIds;
            auto *cObj = reinterpret_cast<cObject *>(ptr);
            emit(scSignalMaMisbehaviorAnnouncement, cObj);
        }
    }

    void MisbehaviorCaService::initializeStaticParameters() {
        // CAM Location Visualizer (PoI)
        F2MDParameters::miscParameters.CamLocationVisualizer = par("CamLocationVisualizer");
        F2MDParameters::miscParameters.CamLocationVisualizerMaxLength = par("CamLocationVisualizerMaxLength");

//        F2MDParameters::attackParameters.StaticAttackType = par("StaticAttackType");

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

        // Fake Report Attack
        F2MDParameters::attackParameters.AttackFakeReportInterval = par("AttackFakeReportInterval");
    }

    void MisbehaviorCaService::trigger() {
        Enter_Method("trigger");
        const SimTime &T_now = simTime();
        if (mAttackType == attackTypes::Disruptive || mAttackType == attackTypes::DataReplay ||
            mAttackType == attackTypes::DoS || mAttackType == attackTypes::DoSRandom ||
            mAttackType == attackTypes::DoSDisruptive || mAttackType == attackTypes::GridSybil ||
            mAttackType == attackTypes::DataReplaySybil || mAttackType == attackTypes::DoSRandomSybil ||
            mAttackType == attackTypes::DoSDisruptiveSybil || checkTriggeringConditions(T_now)) {
            sendCam(T_now);
        }
        if (mAttackType == attackTypes::FakeReport &&
            simTime().inUnit(SimTimeUnit::SIMTIME_MS) - attackFakeReportLastReportTime >
            uint64_t(F2MDParameters::attackParameters.AttackFakeReportInterval * 1000)) {
            attackFakeReportLastReportTime = simTime().inUnit(SimTimeUnit::SIMTIME_MS);
            createFakeReport();
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
            switch (mAttackType) {
                case attackTypes::Disruptive: {
                    disruptiveMessageQueue.emplace_back(*cam);
                    if (disruptiveMessageQueue.size() > F2MDParameters::attackParameters.AttackDisruptiveBufferSize) {
                        disruptiveMessageQueue.pop_front();
                    }
                    break;
                }
                case attackTypes::DataReplay:
                case attackTypes::DoSDisruptive:
                case attackTypes::GridSybil:
                case attackTypes::FakeReport: {
                    receivedMessages[(*cam)->header.stationID].push_back(*cam);
                    if (receivedMessages[(*cam)->header.stationID].size() > 20) {
                        receivedMessages[(*cam)->header.stationID].pop_front();
                    }
                    break;
                }
                default:
                    break;
            }
        }
    }

    void MisbehaviorCaService::sendCam(const SimTime &T_now) {
        uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
        vanetza::asn1::Cam cam;
        switch (mMisbehaviorType) {
            case misbehaviorTypes::Benign:
                cam = createCooperativeAwarenessMessage(genDeltaTimeMod);
                break;
            case misbehaviorTypes::LocalAttacker: {
                cam = createAttackCAM(genDeltaTimeMod);
                break;
            }
            case misbehaviorTypes::GlobalAttacker:
                cam = createAttackCAM(genDeltaTimeMod);
                break;
            default:
                cam = createCooperativeAwarenessMessage(genDeltaTimeMod);
        }
        if (cam->header.messageID != 2) {
            return;
        }
        if (F2MDParameters::miscParameters.CamLocationVisualizer) {
            visualizeCamPosition(cam);
        }
        finalizeAndSendCam(cam, T_now);
    }

    vanetza::asn1::Cam MisbehaviorCaService::createAttackCAM(uint16_t genDeltaTime) {
        vanetza::asn1::Cam message = createCooperativeAwarenessMessage(genDeltaTime);

        switch (mAttackType) {
            case attackTypes::Benign:
                break;
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
                vanetza::units::Velocity speed;
                if (mVehicleDataProvider->speed() + attackConstantSpeedOffsetValue <
                    0 * boost::units::si::meter_per_second) {
                    speed = 0 * boost::units::si::meter_per_second;
                } else {
                    speed = mVehicleDataProvider->speed() + attackConstantSpeedOffsetValue;
                }
                message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue =
                        buildSpeedValue(speed);
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
                double speed = std::min(0.0, uniform(-F2MDParameters::attackParameters.AttackRandomSpeedOffsetMax,
                                                     F2MDParameters::attackParameters.AttackRandomSpeedOffsetMax));
                message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = buildSpeedValue(
                        mVehicleDataProvider->speed() +
                        (speed * boost::units::si::meter_per_second));
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
                    disruptiveMessageQueue.erase(it);
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
                    staleMessageQueue.pop();
                    message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                            mTimer->getTimeFor(mVehicleDataProvider->updated()));
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
                if (!receivedMessages.empty()) {
                    auto it = receivedMessages.begin();
                    int index = (int) uniform(0, (double) receivedMessages.size());
                    std::advance(it, index);
                    if (!it->second.empty()) {
                        message = it->second.front();
                        it->second.pop_front();
                        if (it->second.empty()) {
                            receivedMessages.erase(it->first);
                        }
                        message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                                mTimer->getTimeFor(mVehicleDataProvider->updated()));
                        message->header.stationID = mVehicleDataProvider->getStationId();
                    } else {
                        receivedMessages.erase(it->first);
                        message = vanetza::asn1::Cam();
                    }
                } else {
                    message = vanetza::asn1::Cam();
                }
                break;
            }
            case attackTypes::GridSybil: {
                int offsetIndex;
                if (F2MDParameters::attackParameters.AttackGridSybilSelfSybil) {
                    offsetIndex = attackGridSybilCurrentVehicleIndex;
                } else {
                    message = getNextReplayCam();
                    if (message->header.messageID != 2) {
                        message = vanetza::asn1::Cam();
                        break;
                    } else {
                        offsetIndex = attackGridSybilCurrentVehicleIndex + 1;
                    }
                }
                BasicVehicleContainerHighFrequency &hfc =
                        message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
                double width = hfc.vehicleWidth == VehicleWidth_unavailable ?
                               mVehicleController->getLength().value() :
                               (double) hfc.vehicleWidth / 10;
                double length = hfc.vehicleLength.vehicleLengthValue == VehicleLengthValue_unavailable ?
                                mVehicleController->getLength().value() :
                                (double) hfc.vehicleLength.vehicleLengthValue / 10;
                double offsetX = -((double) offsetIndex / 2) *
                                 (width + attackGridSybilActualDistanceX) +
                                 uniform(-attackGridSybilActualDistanceX / 10, attackGridSybilActualDistanceX / 10);
                double offsetY = -((double) (offsetIndex % 2)) *
                                 (length + attackGridSybilActualDistanceY) +
                                 uniform(-attackGridSybilActualDistanceY / 10, attackGridSybilActualDistanceY / 10);

                ReferencePosition_t &referencePosition = message->cam.camParameters.basicContainer.referencePosition;
                Position originalPosition = convertReferencePosition(referencePosition, mSimulationBoundary, mTraciAPI);

                Position relativePosition = Position(offsetX, offsetY);
                double currentHeadingAngle = (double) hfc.heading.headingValue / 10.0;
                double newAngle = currentHeadingAngle + calculateHeadingAngle(relativePosition);
                newAngle = 360 - std::fmod(newAngle, 360);

                double offsetDistance = sqrt(pow(offsetX, 2) + pow(offsetY, 2));
                double relativeX = offsetDistance * sin(newAngle * PI / 180);
                double relativeY = offsetDistance * cos(newAngle * PI / 180);
                Position sybilPosition = Position(originalPosition.x.value() + relativeX,
                                                  originalPosition.y.value() + relativeY);
                if (getDistanceToNearestRoad(mGlobalEnvironmentModel, sybilPosition) >
                    F2MDParameters::attackParameters.AttackGridSybilMaxDistanceFromRoad) {
                    message = vanetza::asn1::Cam();
                    break;
                }
                setPositionWithJitter(referencePosition, sybilPosition, mSimulationBoundary, mTraciAPI, getRNG(0));

                if (hfc.speed.speedConfidence != SpeedConfidence_unavailable) {
                    long speedConfidence = hfc.speed.speedConfidence;
                    hfc.speed.speedValue = intuniform(std::max(0, (int) (hfc.speed.speedValue - speedConfidence)),
                                                      std::min(16382,
                                                               (int) (hfc.speed.speedValue + speedConfidence)));
                }
                if (hfc.longitudinalAcceleration.longitudinalAccelerationConfidence !=
                    AccelerationConfidence_unavailable) {
                    long accelerationConfidence = hfc.longitudinalAcceleration.longitudinalAccelerationConfidence;
                    hfc.longitudinalAcceleration.longitudinalAccelerationValue =
                            intuniform(std::max(-160,
                                                (int) (hfc.longitudinalAcceleration.longitudinalAccelerationValue -
                                                       accelerationConfidence)),
                                       std::min(160,
                                                (int) (hfc.longitudinalAcceleration.longitudinalAccelerationValue +
                                                       accelerationConfidence)));
                }
                if (hfc.heading.headingConfidence != HeadingConfidence_unavailable) {
                    long headingConfidence = hfc.heading.headingConfidence;
                    long newHeading = intuniform((int) (hfc.heading.headingValue - headingConfidence),
                                                 (int) (hfc.heading.headingValue + headingConfidence));
                    hfc.heading.headingValue = (newHeading + 3600) % 3600;
                }

                double steeringAngle = std::fmod(attackGridSybilLastHeadingAngle - currentHeadingAngle, 360);
                steeringAngle = steeringAngle > 180 ? 360 - steeringAngle : steeringAngle;
                attackGridSybilLastHeadingAngle = currentHeadingAngle;
                if (steeringAngle > 5 && attackGridSybilCurrentVehicleIndex > 0) {
                    message = vanetza::asn1::Cam();
                    break;
                }

                attackGridSybilCurrentVehicleIndex = ++attackGridSybilCurrentVehicleIndex % attackGridSybilVehicleCount;
                message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                        mTimer->getCurrentTime());
                message->header.stationID = mPseudonyms[mPseudonymIndex++];
                mPseudonymIndex %= attackGridSybilVehicleCount;
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
                        receivedMessages[attackDataReplayCurrentStationId].pop_front();
                        message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                                mTimer->getTimeFor(mVehicleDataProvider->updated()));
                        message->header.stationID = mPseudonyms.front();
                    } else {
                        message = vanetza::asn1::Cam();
                    }
                } else {
                    if (!receivedMessages[attackDataReplayCurrentStationId].empty()) {
                        message = receivedMessages[attackDataReplayCurrentStationId].front();
                        receivedMessages[attackDataReplayCurrentStationId].pop_front();
                        message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                                mTimer->getTimeFor(mVehicleDataProvider->updated()));
                        message->header.stationID = mPseudonyms.front();
                    } else {
                        receivedMessages.erase(attackDataReplayCurrentStationId);
                        attackDataReplayCurrentStationId = -1;
                        message = vanetza::asn1::Cam();
                        mPseudonymIndex = ++mPseudonymIndex % attackGridSybilVehicleCount;
                    }
                }
                break;
            }
            case attackTypes::DoSRandomSybil: {
                message->header.stationID = mPseudonyms[mPseudonymIndex++];
                mPseudonymIndex %= attackGridSybilVehicleCount;
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
                if (!receivedMessages.empty()) {
                    auto it = receivedMessages.begin();
                    int index = (int) uniform(0, (double) receivedMessages.size());
                    std::advance(it, index);
                    if (!it->second.empty()) {
                        message = it->second.front();
                        it->second.pop_front();
                        if (it->second.empty()) {
                            receivedMessages.erase(it->first);
                        }
                        message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                                mTimer->getTimeFor(mVehicleDataProvider->updated()));
                        message->header.stationID = mPseudonyms[mPseudonymIndex++];
                        mPseudonymIndex %= attackGridSybilVehicleCount;
                    } else {
                        receivedMessages.erase(it->first);
                        message = vanetza::asn1::Cam();
                    }
                } else {
                    message = vanetza::asn1::Cam();
                }
                break;
            }
            case attackTypes::FakeReport: {
                break;
            }
            default:
                break;
        }
        return message;
    }

    vanetza::asn1::Cam MisbehaviorCaService::getNextReplayCam() {
        vanetza::asn1::Cam message;
        if (attackDataReplayCurrentStationId == -1) {
            auto it = receivedMessages.begin();
            if (it != receivedMessages.end()) {
                int64_t mostReceivedStationId = -1;
                int maxSize = 0;
                for (; it != receivedMessages.end(); ++it) {
                    if ((int) it->second.size() > maxSize) {
                        maxSize = (int) it->second.size();
                        mostReceivedStationId = it->first;
                    }
                }
                attackDataReplayCurrentStationId = mostReceivedStationId;
                message = receivedMessages[attackDataReplayCurrentStationId].front();
                receivedMessages[attackDataReplayCurrentStationId].pop_front();
                if (maxSize == 1) {
                    receivedMessages.erase(attackDataReplayCurrentStationId);
                }
            } else {
                message = vanetza::asn1::Cam();
            }
        } else {
            if (!receivedMessages[attackDataReplayCurrentStationId].empty() &&
                receivedMessages[attackDataReplayCurrentStationId].front()->cam.generationDeltaTime <
                (uint16_t) (countTaiMilliseconds(mTimer->getCurrentTime()) - 1100)) {
                message = receivedMessages[attackDataReplayCurrentStationId].front();
                receivedMessages[attackDataReplayCurrentStationId].pop_front();
            } else {
                receivedMessages.erase(attackDataReplayCurrentStationId);
                attackDataReplayCurrentStationId = -1;
                message = vanetza::asn1::Cam();
            }
        }
        return message;
    }

    void MisbehaviorCaService::createFakeReport() {
        if (!receivedMessages.empty()) {
            auto it = receivedMessages.begin();
            int index = (int) uniform(0, (double) receivedMessages.size());
            std::advance(it, index);
            if (!it->second.empty()) {
                std::shared_ptr<vanetza::asn1::Cam> cam = std::make_shared<vanetza::asn1::Cam>(it->second.front());
                it->second.pop_front();
                std::string reportId(generateReportId((*cam)->header.stationID,
                                                      mVehicleDataProvider->getStationId(),
                                                      getRNG(0)));
                Report report(reportId, cam, countTaiMilliseconds(mTimer->getTimeFor(simTime())));
                auto detectionLevel = static_cast<detectionLevels::DetectionLevels>(intuniform(
                        detectionLevels::Level1, detectionLevels::Level4));
                std::bitset<16> errorCode;
                errorCode[intuniform(0, 7)] = true;
                switch (detectionLevel) {
                    case detectionLevels::Level1:
                        break;
                    case detectionLevels::Level2: {
                        if (it->second.empty()) {
                            detectionLevel = detectionLevels::Level1;
                        } else {
                            std::vector<std::shared_ptr<vanetza::asn1::Cam>> evidenceCams;
                            for (int i = 0; i < std::min((int) it->second.size(),
                                                         F2MDParameters::reportParameters.evidenceContainerMaxCamCount); i++) {
                                evidenceCams.emplace_back(std::make_shared<vanetza::asn1::Cam>(it->second.back()));
                                it->second.pop_back();
                            }
                            report.setReportedMessages(evidenceCams,
                                                       F2MDParameters::reportParameters.evidenceContainerMaxCamCount);
                        }
                        break;
                    }
                    case detectionLevels::Level3: {
                        std::vector<std::shared_ptr<vanetza::asn1::Cam>> neighbourCams;
                        for (auto iterator = receivedMessages.begin(), next_it = iterator;
                             iterator != receivedMessages.end(); iterator = next_it) {
                            ++next_it;
                            if (iterator->first != it->first && !iterator->second.empty()) {
                                neighbourCams.emplace_back(
                                        std::make_shared<vanetza::asn1::Cam>(iterator->second.back()));
                                iterator->second.pop_back();
                                if (iterator->second.empty()) {
                                    receivedMessages.erase(iterator);
                                    continue;
                                }
                            }
                        }
                        report.evidence.neighbourMessages = neighbourCams;
                        break;
                    }
                    case detectionLevels::Level4:
                        report.fillSenderInfoContainer(mVehicleDataProvider, mVehicleController);
                        break;
                    default:
                        break;
                }
                report.setSemanticDetection(detectionLevel, errorCode);
//                if (detectionLevel == attackFakeReportDetectionLevel) {
//                    vanetza::asn1::MisbehaviorReport misbehaviorReport = report.encode();
//                    MisbehaviorReportObject obj(std::move(misbehaviorReport));
//                    emit(scSignalMisbehaviorAuthorityNewReport, &obj);
//                }
                vanetza::asn1::MisbehaviorReport misbehaviorReport = report.encode();
                MisbehaviorReportObject obj(std::move(misbehaviorReport));
                emit(scSignalMisbehaviorAuthorityNewReport, &obj);
                if (it->second.empty()) {
                    receivedMessages.erase(it->first);
                }
            } else {
                receivedMessages.erase(it->first);
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
        std::string poiId = {
                mVehicleController->getVehicleId() + "_CAM_" + std::to_string(cam->header.stationID) + "_" +
                std::to_string((uint16_t) countTaiMilliseconds(mTimer->getCurrentTime()))};
        if (mAttackType == attackTypes::GridSybil && attackGridSybilVehicleCount <= 5) {
            color = colors[(attackGridSybilCurrentVehicleIndex + (attackGridSybilVehicleCount - 1)) %
                           attackGridSybilVehicleCount];
            maxActivePoIs = attackGridSybilVehicleCount;
            poiId = {mVehicleController->getVehicleId() + "_CAM_" + std::to_string(cam->header.stationID) + "_" +
                     std::to_string((uint16_t) countTaiMilliseconds(
                             mTimer->getCurrentTime()))};
        }
        traci::TraCIGeoPosition traciGeoPosition = {
                (double) cam->cam.camParameters.basicContainer.referencePosition.longitude / 10000000.0,
                (double) cam->cam.camParameters.basicContainer.referencePosition.latitude / 10000000.0};
        traci::TraCIPosition traciPosition = mVehicleController->getTraCI()->convert2D(traciGeoPosition);
        mTraciAPI->poi.add(poiId, traciPosition.x, traciPosition.y, color,
                           poiId, 5, "", 0,
                           0, 0);
        activePoIs.push_back(poiId);
        if (activePoIs.size() > maxActivePoIs) {
            mTraciAPI->poi.remove(activePoIs.front());
            activePoIs.pop_front();
        }
        if (mAttackType != attackTypes::GridSybil) {
            int alphaStep = 185 / maxActivePoIs;
            int currentAlpha = 80;
            for (const auto &poi: activePoIs) {
                mTraciAPI->poi.setColor(poi, libsumo::TraCIColor(255, 0, 255, currentAlpha));
                currentAlpha += alphaStep;
            }
        }
    }

} // namespace artery
