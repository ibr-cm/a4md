#include "LegacyChecks.h"
#include <utility>
#include <artery/traci/Cast.h>
#include "artery/envmod/sensor/Sensor.h"
#include "artery/application/md/util/HelperFunctions.h"

namespace artery {
    bool LegacyChecks::staticInitializationComplete = false;
    GlobalEnvironmentModel *LegacyChecks::mGlobalEnvironmentModel;
    std::shared_ptr<const traci::API> LegacyChecks::mTraciAPI;
    traci::Boundary LegacyChecks::mSimulationBoundary;

    using namespace omnetpp;

    double LegacyChecks::ProximityPlausibilityCheck(const Position &senderPosition, const Position &receiverPosition,
                                                    const vector<vanetza::asn1::Cam *> &surroundingCamObjects) {
        Position::value_type deltaDistance = distance(senderPosition, receiverPosition);
        double deltaAngle = calculateHeadingAngle(
                Position(senderPosition.x - receiverPosition.x, senderPosition.y - receiverPosition.y));
        if (deltaDistance.value() < detectionParameters->maxProximityRangeL) {
            if (deltaDistance.value() < detectionParameters->maxProximityRangeW * 2 ||
                (deltaAngle < 90 && deltaDistance.value() <
                                    (detectionParameters->maxProximityRangeW / cos((90 - deltaAngle) * PI / 180)))) {
                Position::value_type minimumDistance = Position::value_type::from_value(9999);

                for (auto cam : surroundingCamObjects) {
                    Position::value_type currentDistance = distance(senderPosition, convertCamPosition(
                            (*cam)->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary,
                            mTraciAPI));
                    if (currentDistance < minimumDistance) {
                        minimumDistance = currentDistance;
                    }
                }
                if (minimumDistance.value() < detectionParameters->maxProximityDistance) {
                    return 1;
                } else {
                    return 0;
                }
            }
        }
        return 1;
    }

    double
    LegacyChecks::RangePlausibilityCheck(const Position &senderPosition, const Position &receiverPosition) const {
        if (distance(senderPosition, receiverPosition).value() < detectionParameters->maxPlausibleRange) {
            return 1;
        } else {
            return 0;
        }
    }

    double
    LegacyChecks::PositionConsistencyCheck(Position &senderPosition, const Position &receiverPosition,
                                           double time) const {
        if (distance(senderPosition, receiverPosition).value() < detectionParameters->maxPlausibleSpeed * time) {
            return 1;
        } else {
            return 0;
        }
    }

    double LegacyChecks::SpeedConsistencyCheck(double currentSpeed, double oldSpeed, double time) const {
        double deltaSpeed = currentSpeed - oldSpeed;
        if (deltaSpeed > 0) {
            if (deltaSpeed < detectionParameters->maxPlausibleAcceleration * time) {
                return 1;
            } else {
                return 0;
            }
        } else {
            if (fabs(deltaSpeed) < detectionParameters->maxPlausibleDeceleration * time) {
                return 1;
            } else {
                return 0;
            }
        }
    }

    double LegacyChecks::PositionSpeedConsistencyCheck(Position &currentPosition, Position &oldPosition,
                                                       double currentSpeed, double oldSpeed,
                                                       double deltaTime) const {
        if (deltaTime < detectionParameters->maxTimeDelta) {
            double deltaDistance = distance(currentPosition, oldPosition).value();
            double currentMinimumSpeed = std::min(currentSpeed, oldSpeed);
            double minimumDistance =
                    oldSpeed * deltaTime - 0.5 * detectionParameters->maxPlausibleDeceleration * pow(deltaTime, 2);
            double maximumDistance =
                    oldSpeed * deltaTime + 0.5 * detectionParameters->maxPlausibleAcceleration * pow(deltaTime, 2);
            double addonMgtRange = detectionParameters->maxMgtRngDown + 0.3571 * currentMinimumSpeed -
                                   0.01694 * pow(currentMinimumSpeed, 2);
            addonMgtRange = (addonMgtRange < 0) ? 0 : addonMgtRange;

            if ((deltaDistance - minimumDistance + addonMgtRange) < 0 ||
                (maximumDistance - deltaDistance + detectionParameters->maxMgtRngUp) < 0) {
                return 0;
            }
        }
        return 1;
    }

    double
    LegacyChecks::PositionSpeedMaxConsistencyCheck(Position &currentPosition, Position &oldPosition,
                                                   double currentSpeed,
                                                   double oldSpeed, double deltaTime) const {
        if (deltaTime < detectionParameters->maxTimeDelta) {
            double deltaDistance = distance(currentPosition, oldPosition).value();
            double theoreticalSpeed = deltaDistance / deltaTime;
            if (std::max(currentSpeed, oldSpeed) - theoreticalSpeed >
                (detectionParameters->maxPlausibleDeceleration + detectionParameters->maxMgtRng) * (double) deltaTime) {
                return 0;
            } else {
                if (theoreticalSpeed - std::min(currentSpeed, oldSpeed) >
                    (detectionParameters->maxPlausibleAcceleration + detectionParameters->maxMgtRng) *
                    (double) deltaTime) {
                    return 0;
                }
            }
        }
        return 1;
    }

    double LegacyChecks::SpeedPlausibilityCheck(double speed) const {
        if (fabs(speed) < detectionParameters->maxPlausibleSpeed) {
            return 1;
        } else {
            return 0;
        }
    }

    double
    LegacyChecks::IntersectionCheck(const std::vector<Position> &receiverVehicleOutline,
                                    const vector<vanetza::asn1::Cam *> &relevantCams, const Position &senderPosition,
                                    const double &senderLength, const double &senderWidth,
                                    const double &senderHeading) {
        std::vector<Position> senderOutline = getVehicleOutline(senderPosition, Angle::from_degree(senderHeading),
                                                                senderLength, senderWidth);
        if (boost::geometry::intersects(senderOutline, receiverVehicleOutline)) {
            return 0;
        }
        for (auto cam : relevantCams) {
            std::vector<Position> outline = getVehicleOutline((*cam), mSimulationBoundary, mTraciAPI);
            if (boost::geometry::intersects(senderOutline, outline)) {
                return 0;
            }
        }
        return 1;

    }

    double LegacyChecks::SuddenAppearanceCheck(Position &senderPosition, const Position &receiverPosition) const {
        if (distance(senderPosition, receiverPosition).value() > detectionParameters->maxSuddenAppearanceRange) {
            return 1;
        } else {
            return 0;
        }
    }

    double LegacyChecks::PositionPlausibilityCheck(Position &senderPosition, double senderSpeed) const {
        if (senderSpeed < detectionParameters->maxOffroadSpeed ||
            getDistanceToNearestRoad(mGlobalEnvironmentModel, senderPosition) <
            detectionParameters->maxDistanceFromRoad) {
            return 1;
        } else {
            return 0;
        }
    }

    double LegacyChecks::FrequencyCheck(long newTime, long oldTime) const {
        if (newTime - oldTime > (long) (detectionParameters->maxCamFrequency * 1000)) {
            return 0;
        } else {
            return 1;
        }
    }


    double
    LegacyChecks::PositionHeadingConsistencyCheck(const double &currentHeading, Position &currentPosition,
                                                  Position &oldPosition,
                                                  double deltaTime, double currentSpeed) const {
        if (deltaTime < detectionParameters->positionHeadingTime) {
            if (distance(currentPosition, oldPosition).value() < 1 || currentSpeed < 1) {
                return 1;
            }
            libsumo::TraCIPosition currentPositionTraci = position_cast(mSimulationBoundary, currentPosition);
            libsumo::TraCIPosition oldPositionTraci = position_cast(mSimulationBoundary, oldPosition);
            Position deltaPosition = Position(currentPositionTraci.x - oldPositionTraci.x,
                                              currentPositionTraci.y - oldPositionTraci.y);
            // add 90 degree for traci offset
            double positionAngle = std::fmod(calculateHeadingAngle(deltaPosition) + 90, 360);
            double deltaHeading = calculateHeadingDifference(currentHeading, positionAngle);
            if (deltaHeading > detectionParameters->maxHeadingChange) {
                return 0;
            }
        }
        return 1;
    }

    void LegacyChecks::KalmanPositionSpeedConsistencyCheck(Position &currentPosition,
                                                           const PosConfidenceEllipse_t &currentPositionConfidence,
                                                           const Position &currentSpeed,
                                                           const Position &currentAcceleration,
                                                           double currentSpeedConfidence,
                                                           double &deltaTime, double *returnValue) const {
        if (!kalmanSVI->isInitialized()) {
            returnValue[0] = 1;
            returnValue[1] = 1;
        } else {
            if (deltaTime < detectionParameters->maxKalmanTime) {
                double deltaPosition[4];

                double currentSemiMajorConfidence = std::max(
                        (double) currentPositionConfidence.semiMajorConfidence / 100.0,
                        detectionParameters->kalmanMinPosRange);
                double currentSemiMinorConfidence = std::max(
                        (double) currentPositionConfidence.semiMinorConfidence / 100.0,
                        detectionParameters->kalmanMinPosRange);
                currentSpeedConfidence = std::max(currentSpeedConfidence, detectionParameters->kalmanMinSpeedRange);

                kalmanSVI->getDeltaPos(deltaTime, currentPosition.x.value(), currentPosition.y.value(),
                                       currentSpeed.x.value(), currentSpeed.y.value(), currentAcceleration.x.value(),
                                       currentAcceleration.y.value(), currentSemiMajorConfidence,
                                       currentSemiMinorConfidence,
                                       currentSpeedConfidence, currentSpeedConfidence, deltaPosition);

                double ret_1 = 1 - sqrt(pow(deltaPosition[0], 2.0) + pow(deltaPosition[2], 2.0)) /
                                   (detectionParameters->kalmanPosRange * currentSemiMajorConfidence * deltaTime);

                double ret_2 = 1 - sqrt(pow(deltaPosition[1], 2.0) + pow(deltaPosition[3], 2.0)) /
                                   (detectionParameters->kalmanSpeedRange * currentSpeedConfidence * deltaTime);
                returnValue[0] = isnan(ret_1) || ret_1 < 0.5 ? 0 : 1;
                returnValue[1] = isnan(ret_2) || ret_2 < 0.5 ? 0 : 1;
            } else {
                returnValue[0] = 1;
                returnValue[1] = 1;
                kalmanSVI->setInitial(currentPosition.x.value(), currentPosition.y.value(), currentSpeed.x.value(),
                                      currentSpeed.y.value());
            }
        }
    }


    void LegacyChecks::KalmanPositionSpeedScalarConsistencyCheck(Position &currentPosition, Position &oldPosition,
                                                                 const PosConfidenceEllipse_t &currentPositionConfidence,
                                                                 double &currentSpeed, double &currentAcceleration,
                                                                 double &currentSpeedConfidence, double &deltaTime,
                                                                 double *returnValue) {
        if (!kalmanSVSI->isInitialized()) {
            returnValue[0] = 1;
            returnValue[1] = 1;
        } else {
            if (deltaTime < detectionParameters->maxKalmanTime) {
                double deltaPosition[2];

                double deltaDistance = distance(currentPosition, oldPosition).value();
                double curPosConfX = std::max((double) currentPositionConfidence.semiMajorConfidence,
                                              detectionParameters->kalmanMinPosRange);
                double curSpdConfX = std::max(currentSpeedConfidence, detectionParameters->kalmanSpeedRange);

                kalmanSVSI->getDeltaPos(deltaTime, deltaDistance, currentSpeed, currentAcceleration,
                                        currentAcceleration,
                                        curPosConfX, curSpdConfX, deltaPosition);

                double ret_1 = 1 - (deltaPosition[0] / (detectionParameters->kalmanPosRange * curPosConfX * deltaTime));
                double ret_2 =
                        1 - (deltaPosition[1] / (detectionParameters->kalmanSpeedRange * curSpdConfX * deltaTime));
                returnValue[0] = isnan(ret_1) || ret_1 < 0.5 ? 0 : 1;
                returnValue[1] = isnan(ret_2) || ret_2 < 0.5 ? 0 : 1;
            } else {
                returnValue[0] = 1;
                returnValue[1] = 1;
                kalmanSVSI->setInitial(0, currentSpeed);
            }
        }
    }

    double LegacyChecks::KalmanPositionConsistencyCheck(Position &currentPosition, Position &oldPosition,
                                                        const PosConfidenceEllipse_t &currentPositionConfidence,
                                                        double &deltaTime) {
        if (!kalmanSI->isInit()) {
            return 1;
        } else {
            if (deltaTime < detectionParameters->maxKalmanTime) {
                double deltaPosition[2];
                double curPosConfX = std::max((double) currentPositionConfidence.semiMajorConfidence,
                                              detectionParameters->kalmanMinPosRange);
                double curPosConfY = std::max((double) currentPositionConfidence.semiMinorConfidence,
                                              detectionParameters->kalmanMinPosRange);

                kalmanSI->getDeltaPos(deltaTime, currentPosition.x.value(), currentPosition.y.value(),
                                      curPosConfX, curPosConfY, deltaPosition);

                double ret_1 = 1 - sqrt(pow(deltaPosition[0], 2.0) + pow(deltaPosition[1], 2.0)) /
                                   (4 * detectionParameters->kalmanPosRange * curPosConfX * deltaTime);
                return isnan(ret_1) || ret_1 < 0.5 ? 0 : 1;
            } else {
                kalmanSI->setInitial(currentPosition.x.value(), currentPosition.y.value());
                return 1;
            }
        }
    }

    double
    LegacyChecks::KalmanPositionAccConsistencyCheck(const Position &currentPosition, const Position &currentSpeed,
                                                    const PosConfidenceEllipse_t &currentPositionConfidence,
                                                    double &deltaTime) {
        if (!kalmanSI->isInit()) {
            return 1;
        } else {
            if (deltaTime < detectionParameters->maxKalmanTime) {
                double deltaPosition[2];
                double curPosConfX = std::max((double) currentPositionConfidence.semiMajorConfidence,
                                              detectionParameters->kalmanMinPosRange);
                double curPosConfY = std::max((double) currentPositionConfidence.semiMinorConfidence,
                                              detectionParameters->kalmanMinPosRange);

                kalmanSI->getDeltaPos(deltaTime, currentPosition.x.value(), currentPosition.y.value(),
                                      currentSpeed.x.value(),
                                      currentSpeed.y.value(),
                                      curPosConfX, curPosConfY, deltaPosition);

                double ret_1 = 1 - sqrt(pow(deltaPosition[0], 2.0) + pow(deltaPosition[1], 2.0)) /
                                   (4 * detectionParameters->kalmanPosRange * curPosConfX * deltaTime);
                return isnan(ret_1) || ret_1 < 0.5 ? 0 : 1;
            } else {
                kalmanSI->setInitial(currentPosition.x.value(), currentPosition.x.value());
                return 1;
            }
        }
    }

    double
    LegacyChecks::KalmanSpeedConsistencyCheck(const Position &currentSpeed, const Position &oldSpeed,
                                              double &currentSpeedConfidence, const Position &currentAcceleration,
                                              double &deltaTime) {
        if (!kalmanVI->isInit()) {
            return 1;
        } else {
            if (deltaTime < detectionParameters->maxKalmanTime) {
                double deltaPosition[2];

                double curSpdConf = std::max(currentSpeedConfidence, detectionParameters->kalmanMinSpeedRange);

                kalmanVI->getDeltaPos(deltaTime, currentSpeed.x.value(), currentSpeed.y.value(),
                                      currentAcceleration.x.value(),
                                      currentAcceleration.y.value(), curSpdConf, curSpdConf, deltaPosition);

                double ret_1 = 1 - sqrt(pow(deltaPosition[0], 2.0) + pow(deltaPosition[1], 2.0)) /
                                   (detectionParameters->kalmanSpeedRange * curSpdConf * deltaTime);
                return isnan(ret_1) || ret_1 < 0.5 ? 0 : 1;
            } else {
                kalmanVI->setInitial(currentSpeed.x.value(), currentSpeed.y.value());
                return 1;
            }
        }
    }

    CheckResult *
    LegacyChecks::checkCAM(const VehicleDataProvider *receiverVDP, const std::vector<Position> &receiverVehicleOutline,
                           const vanetza::asn1::Cam &currentCam, const vanetza::asn1::Cam *lastCamPtr,
                           const std::vector<vanetza::asn1::Cam *> &surroundingCamObjects) {

        BasicVehicleContainerHighFrequency_t hfc = currentCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;

        Position currentCamPosition = convertCamPosition(
                currentCam->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
        const Position &receiverPosition = receiverVDP->position();
        double currentCamSpeed = (double) hfc.speed.speedValue / 100.0;
        double currentCamSpeedConfidence = (double) hfc.speed.speedConfidence / 100.0;
        double currentCamAcceleration =
                (double) hfc.longitudinalAcceleration.longitudinalAccelerationValue / 10.0;
        double currentCamHeading = (double) hfc.heading.headingValue / 10;
        double currentCamVehicleLength = (double) hfc.vehicleLength.vehicleLengthValue / 10;
        double currentCamVehicleWidth = (double) hfc.vehicleWidth / 10;
        Position currentCamSpeedVector = getVector(currentCamSpeed, currentCamHeading);
        Position currentCamAccelerationVector = getVector(currentCamAcceleration, currentCamHeading);
        auto *result = new CheckResult;
        result->positionPlausibility = PositionPlausibilityCheck(currentCamPosition, currentCamSpeed);
        result->speedPlausibility = SpeedPlausibilityCheck(currentCamSpeed);
        result->proximityPlausibility = ProximityPlausibilityCheck(currentCamPosition, receiverPosition,
                                                                   surroundingCamObjects);
        result->rangePlausibility = RangePlausibilityCheck(currentCamPosition, receiverPosition);

        if (lastCamPtr != nullptr) {
            vanetza::asn1::Cam lastCam = *lastCamPtr;
            Position lastCamPosition = convertCamPosition(lastCam->cam.camParameters.basicContainer.referencePosition,
                                                          mSimulationBoundary, mTraciAPI);
            double lastCamSpeed =
                    (double) lastCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue /
                    100.0;
            auto camDeltaTime = (double) (currentCam->cam.generationDeltaTime - lastCam->cam.generationDeltaTime);
            if (lastCam->cam.generationDeltaTime > currentCam->cam.generationDeltaTime) {
                camDeltaTime += 65536;
            }
            camDeltaTime /= 1000;
            result->consistencyIsChecked = true;
            result->positionConsistency = PositionConsistencyCheck(currentCamPosition, lastCamPosition, camDeltaTime);
            result->speedConsistency = SpeedConsistencyCheck(currentCamSpeed, lastCamSpeed, camDeltaTime);
            result->positionSpeedConsistency = PositionSpeedConsistencyCheck(currentCamPosition, lastCamPosition,
                                                                             currentCamSpeed, lastCamSpeed,
                                                                             camDeltaTime);
            result->positionSpeedMaxConsistency = PositionSpeedMaxConsistencyCheck(currentCamPosition, lastCamPosition,
                                                                                   currentCamSpeed, lastCamSpeed,
                                                                                   camDeltaTime);
            result->positionHeadingConsistency = PositionHeadingConsistencyCheck(
                    currentCamHeading,
                    currentCamPosition, lastCamPosition, camDeltaTime, currentCamSpeed);

            double returnValue[2];
            KalmanPositionSpeedConsistencyCheck(currentCamPosition,
                                                currentCam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse,
                                                currentCamSpeedVector,
                                                getVector(currentCamAcceleration, currentCamHeading),
                                                currentCamSpeedConfidence, camDeltaTime, returnValue);
            result->kalmanPositionSpeedConsistencyPosition = returnValue[0];
            result->kalmanPositionSpeedConsistencySpeed = returnValue[1];
            KalmanPositionSpeedScalarConsistencyCheck(currentCamPosition, lastCamPosition,
                                                      currentCam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse,
                                                      currentCamSpeed, currentCamAcceleration,
                                                      currentCamSpeedConfidence,
                                                      camDeltaTime, returnValue);
            result->kalmanPositionSpeedScalarConsistencyPosition = returnValue[0];
            result->kalmanPositionSpeedScalarConsistencySpeed = returnValue[1];
            result->kalmanPositionConsistencyConfidence = KalmanPositionConsistencyCheck(currentCamPosition,
                                                                                         lastCamPosition,
                                                                                         currentCam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse,
                                                                                         camDeltaTime);
            result->kalmanPositionAccelerationConsistencySpeed = KalmanPositionAccConsistencyCheck(currentCamPosition,
                                                                                                   currentCamSpeedVector,
                                                                                                   currentCam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse,
                                                                                                   camDeltaTime);
            result->kalmanSpeedConsistencyConfidence = KalmanSpeedConsistencyCheck(currentCamSpeedVector,
                                                                                   getVector(lastCamSpeed,
                                                                                             (double) lastCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue /
                                                                                             10.0),
                                                                                   currentCamSpeedConfidence,
                                                                                   currentCamAccelerationVector,
                                                                                   camDeltaTime);
            result->frequency = FrequencyCheck(currentCam->cam.generationDeltaTime, lastCam->cam.generationDeltaTime);
            result->intersection = IntersectionCheck(receiverVehicleOutline, surroundingCamObjects, currentCamPosition,
                                                     currentCamVehicleLength, currentCamVehicleWidth,
                                                     currentCamHeading);
        } else {
            result->suddenAppearance = SuddenAppearanceCheck(currentCamPosition, receiverPosition);
        }
        return result;
    }

    LegacyChecks::LegacyChecks(std::shared_ptr<const traci::API> traciAPI,
                               GlobalEnvironmentModel *globalEnvironmentModel,
                               DetectionParameters *detectionParameters,
                               Kalman_SVI *kalmanSVI, Kalman_SC *kalmanSVSI,
                               Kalman_SI *kalmanSI, Kalman_SI *kalmanVI) :
            detectionParameters(detectionParameters),
            kalmanSVI(kalmanSVI), kalmanSVSI(kalmanSVSI),
            kalmanSI(kalmanSI), kalmanVI(kalmanVI) {
        if (!staticInitializationComplete) {
            staticInitializationComplete = true;
            mGlobalEnvironmentModel = globalEnvironmentModel;
            mTraciAPI = std::move(traciAPI);
            mSimulationBoundary = traci::Boundary{mTraciAPI->simulation.getNetBoundary()};
        }
    }
}