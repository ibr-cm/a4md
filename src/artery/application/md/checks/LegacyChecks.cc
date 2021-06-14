#include "LegacyChecks.h"
#include "artery/envmod/sensor/Sensor.h"

namespace artery {

    using namespace omnetpp;

    double LegacyChecks::calculateHeadingAngle(const Position &position) {
        double angle = atan2(-position.y.value(), position.x.value()) * 180 / PI;
        return angle;
    }

    double LegacyChecks::ProximityPlausibilityCheck(Position &testPosition, Position &myPosition) {
        Position::value_type deltaDistance = distance(testPosition, myPosition);
        double deltaAngle = calculateHeadingAngle(
                Position(testPosition.x - myPosition.x, testPosition.y - myPosition.y));

        if (deltaDistance.value() < maxProximityRangeL) {
            if (deltaDistance.value() < maxProximityRangeW * 2 ||
                (deltaAngle < 90 && deltaDistance.value() < (maxProximityRangeW / cos((90 - deltaAngle) * PI / 180)))) {
                Position::value_type minimumDistance = Position::value_type::from_value(9999);

                for (const auto &object: envModObjects) {
                    std::weak_ptr<EnvironmentModelObject> obj_ptr = object.first;
                    if (obj_ptr.expired()) continue; /*< objects remain in tracking briefly after leaving simulation */
                    const auto &vd = obj_ptr.lock()->getVehicleData();
                    Position::value_type currentDistance = distance(testPosition, vd.position());
                    if (currentDistance < minimumDistance) {
                        minimumDistance = currentDistance;
                    }
                }
                if (minimumDistance.value() < maxProximityDistance) {
                    return 1;
                } else {
                    return 0;
                }
            }
        }
        return 1;
    }

    double LegacyChecks::RangePlausibilityCheck(Position &senderPosition, Position &receiverPosition) const {
        if (distance(senderPosition, receiverPosition).value() < maxPlausibleRange) {
            return 1;
        } else {
            return 0;
        }
    }

    double LegacyChecks::PositionConsistencyCheck(Position &senderPosition, Position &receiverPosition, double time) const {
        if (distance(senderPosition, receiverPosition).value() < maxPlausibleSpeed * time) {
            return 1;
        } else {
            return 0;
        }
    }

    double LegacyChecks::SpeedConsistencyCheck(double currentSpeed, double oldSpeed, double time) const {
        double deltaSpeed = currentSpeed - oldSpeed;
        if (deltaSpeed > 0) {
            if (deltaSpeed < maxPlausibleAcceleration * time) {
                return 1;
            } else {
                return 0;
            }
        } else {
            if (fabs(deltaSpeed) < maxPlausibleDeceleration * time) {
                return 1;
            } else {
                return 0;
            }
        }
    }


    double
    LegacyChecks::PositionSpeedConsistencyCheck(Position &currentPosition, Position &oldPosition, double currentSpeed,
                                                double oldSpeed, double deltaTime) const {
        if (deltaTime < maxTimeDelta) {
            Position::value_type deltaDistance = distance(currentPosition, oldPosition);
            double theoreticalSpeed = deltaDistance.value() / (double) deltaTime;
            if (std::max(currentSpeed, oldSpeed) - theoreticalSpeed >
                (maxPlausibleDeceleration + maxMgtRng) * (double) deltaTime) {
                return 0;
            } else {
                if (theoreticalSpeed - std::min(currentSpeed, oldSpeed) >
                    (maxPlausibleAcceleration + maxMgtRng) * (double) deltaTime) {
                    return 0;
                }
            }
        }
        return 1;
    }

    double LegacyChecks::PositionSpeedMaxConsistencyCheck(Position &currentPosition, Position &oldPosition,
                                                          double currentSpeed, double oldSpeed, double deltaTime) const {
        if (deltaTime < maxTimeDelta) {
            double deltaDistance = distance(currentPosition, oldPosition).value();
            double currentMinimumSpeed = std::min(currentSpeed, oldSpeed);
            double minimumDistance = oldSpeed * deltaTime - 0.5 * maxPlausibleDeceleration * pow(deltaTime, 2);
            double maximumDistance = oldSpeed * deltaTime + 0.5 * maxPlausibleAcceleration * pow(deltaTime, 2);
            double addonMgtRange = maxMgtRngDown + 0.3571 * currentMinimumSpeed - 0.01694 * pow(currentMinimumSpeed, 2);
            addonMgtRange = (addonMgtRange < 0) ? 0 : addonMgtRange;

            if ((deltaDistance - minimumDistance + addonMgtRange) < 0 ||
                (maximumDistance - deltaDistance + maxMgtRngUp) < 0) {
                return 0;
            }
        }
        return 1;
    }

    double LegacyChecks::SpeedPlausibilityCheck(double speed) const {
        if (fabs(speed) < maxPlausibleSpeed) {
            return 1;
        } else {
            return 0;
        }
    }

    double
    LegacyChecks::IntersectionCheck(Position nodePosition1, Position nodeSize1, Position head1, Position nodePosition2,
                                    Position nodeSize2, Position head2, double deltaTime) {
        //TODO
        return 0;
    }

    double LegacyChecks::SuddenAppearenceCheck(Position &senderPosition, Position &receiverPosition) {
        if (distance(senderPosition, receiverPosition).value() > maxSuddenAppearanceRange) {
            return 1;
        } else {
            return 0;
        }
    }

    double LegacyChecks::PositionPlausibilityCheck(Position &senderPosition, double senderSpeed) {
        if (senderSpeed < maxOffroadSpeed) {
            return 1;
        } else {
            //TODO check distance from road
        }
        return 1;
    }

    double LegacyChecks::FrequencyCheck(long newTime,long oldTime) const {
        if (newTime - oldTime < maxCamFrequency) {
            return 0;
        } else {
            return 1;
        }
    }


    double
    LegacyChecks::PositionHeadingConsistencyCheck(const HeadingValue_t &currentHeading, Position &currentPosition,
                                                  Position &oldPosition,
                                                  double deltaTime, double currentSpeed) const {
        if (deltaTime < positionHeadingTime) {
            if (distance(currentPosition, oldPosition).value() < 1 || currentSpeed < 1) {
                return 1;
            }

            double currentHeadingAngle = ((double) currentHeading) / 10.0;
            double positionAngle = calculateHeadingAngle(
                    Position(currentPosition.x - oldPosition.x, currentPosition.y - oldPosition.y));
            double angleDelta = fabs(currentHeadingAngle - positionAngle);
            if (angleDelta > 180) {
                angleDelta = 360 - angleDelta;
            }
            if (angleDelta > maxHeadingChange) {
                return 0;
            }
        }
        return 1;
    }

    void LegacyChecks::KalmanPositionSpeedConsistencyCheck(Position *curPosition, Position *curPositionConfidence,
                                                           Position *curSpeed, Position *oldSpeed,
                                                           Position *curSpeedConfidence, double time,
                                                           Kalman_SVI *kalmanSVI, double *retVal) {

    }


    void LegacyChecks::KalmanPositionSpeedScalarConsistencyCheck(Position *curPosition, Position *oldPosition,
                                                                 Position *curPositionConfidence, Position *curSpeed,
                                                                 Position *oldSpeed, Position *curSpeedConfidence,
                                                                 double time, Kalman_SC *kalmanSC, double *retVal) {

    }

    double LegacyChecks::KalmanPositionConsistencyCheck(Position *curPosition, Position *oldPosition,
                                                        Position *curPosConfidence, double time, Kalman_SI *kalmanSI) {
        return 0;
    }

    double LegacyChecks::KalmanPositionAccConsistencyCheck(Position *curPosition, Position *curSpeed,
                                                           Position *curPosConfidence, double time,
                                                           Kalman_SI *kalmanSI) {
        return 0;
    }

    double
    LegacyChecks::KalmanSpeedConsistencyCheck(Position *curSpeed, Position *oldSpeed, Position *curSpeedConfidence,
                                              double time, Kalman_SI *kalmanSI) {
        return 0;
    }

    CheckResult LegacyChecks::checkCAM(const vanetza::asn1::Cam &newCam) {


        traci::TraCIGeoPosition traciGeoPosition = {
                (double) newCam->cam.camParameters.basicContainer.referencePosition.longitude / 10000000.0,
                (double) newCam->cam.camParameters.basicContainer.referencePosition.latitude / 10000000.0};
        traci::TraCIPosition traciPosition = mVehicleController->getTraCI()->convert2D(traciGeoPosition);

        Position currentCamPosition = Position(traciPosition.x, traciPosition.y);
        double  currentCamSpeed = (double) newCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue / 100.0;
        mPosition = mVehicleDataProvider->position();
        if (lastCam->header.messageID != 0) {
            camDeltaTime = (double) (newCam->cam.generationDeltaTime - lastCam->cam.generationDeltaTime);
            if (lastCam->cam.generationDeltaTime > newCam->cam.generationDeltaTime) {
                camDeltaTime += 65536;
            }
            camDeltaTime /= 1000;
        }

        CheckResult_t result;
        result.proximityPlausibility = ProximityPlausibilityCheck(currentCamPosition, mPosition);
        result.rangePlausibility = RangePlausibilityCheck(currentCamPosition, mPosition);
        result.positionConsistency = PositionConsistencyCheck(currentCamPosition, mPosition, camDeltaTime);
        result.speedConsistency = SpeedConsistencyCheck(currentCamSpeed, lastCamSpeed, camDeltaTime);
        result.positionSpeedConsistency = PositionSpeedConsistencyCheck(currentCamPosition, lastCamPosition,
                                                                        currentCamSpeed, lastCamSpeed, camDeltaTime);
        result.positionSpeedMaxConsistency = PositionSpeedMaxConsistencyCheck(currentCamPosition, lastCamPosition,
                                                                              currentCamSpeed, lastCamSpeed,
                                                                              camDeltaTime);
        result.speedPlausibility = SpeedPlausibilityCheck(currentCamSpeed);
        //TODO IntersectionCheck
        result.suddenAppearance = SuddenAppearenceCheck(currentCamPosition, mPosition);
        result.positionPlausibility = PositionPlausibilityCheck(currentCamPosition, currentCamSpeed);
        result.frequency = FrequencyCheck(newCam->cam.generationDeltaTime, lastCam->cam.generationDeltaTime);
        result.positionHeadingConsistency = PositionHeadingConsistencyCheck(
                newCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue,
                currentCamPosition, lastCamPosition, camDeltaTime, currentCamSpeed);

        lastCamPosition = currentCamPosition;
        lastCam = newCam;
        lastCamSpeed = currentCamSpeed;
        return result;
    }
}