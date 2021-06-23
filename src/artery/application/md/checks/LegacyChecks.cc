#include "LegacyChecks.h"

#include <utility>
#include "artery/envmod/sensor/Sensor.h"

namespace artery {
    bool LegacyChecks::staticInitializationComplete = false;
    GlobalEnvironmentModel *LegacyChecks::mGlobalEnvironmentModel;
    using namespace omnetpp;

    double LegacyChecks::calculateHeadingAngle(const Position &position) {
        double angle = atan2(-position.y.value(), position.x.value()) * 180 / PI;
        return angle;
    }

    double LegacyChecks::ProximityPlausibilityCheck(Position &testPosition, const Position &myPosition,
                                                    TrackedObjectsFilterRange &envModObjects) {
        Position::value_type deltaDistance = distance(testPosition, myPosition);
        double deltaAngle = calculateHeadingAngle(
                Position(testPosition.x - myPosition.x, testPosition.y - myPosition.y));

        if (deltaDistance.value() < detectionParameters->maxProximityRangeL) {
            if (deltaDistance.value() < detectionParameters->maxProximityRangeW * 2 ||
                (deltaAngle < 90 && deltaDistance.value() <
                                    (detectionParameters->maxProximityRangeW / cos((90 - deltaAngle) * PI / 180)))) {
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


    double
    LegacyChecks::PositionSpeedConsistencyCheck(Position &currentPosition, Position &oldPosition, double currentSpeed,
                                                double oldSpeed, double deltaTime) const {
        if (deltaTime < detectionParameters->maxTimeDelta) {
            Position::value_type deltaDistance = distance(currentPosition, oldPosition);
            double theoreticalSpeed = deltaDistance.value() / (double) deltaTime;
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

    double LegacyChecks::PositionSpeedMaxConsistencyCheck(Position &currentPosition, Position &oldPosition,
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

    double LegacyChecks::SpeedPlausibilityCheck(double speed) const {
        if (fabs(speed) < detectionParameters->maxPlausibleSpeed) {
            return 1;
        } else {
            return 0;
        }
    }

//    double
//    LegacyChecks::IntersectionCheck(Position nodePosition1, Position nodeSize1, Position head1, Position nodePosition2,
//                                    Position nodeSize2, Position head2, double deltaTime) {
//        //TODO
//        return 0;
//    }

    double LegacyChecks::SuddenAppearanceCheck(Position &senderPosition, const Position &receiverPosition) const {
        if (distance(senderPosition, receiverPosition).value() > detectionParameters->maxSuddenAppearanceRange) {
            return 1;
        } else {
            return 0;
        }
    }

    double LegacyChecks::PositionPlausibilityCheck(Position &senderPosition, double senderSpeed) const {
        if (senderSpeed < detectionParameters->maxOffroadSpeed) {
            return 1;
        } else {
            double minDistance = 9999;
            std::vector<GeometryRtreeValue> laneResults;
            mGlobalEnvironmentModel->getLaneRTree()->query(boost::geometry::index::nearest(senderPosition, 10),
                                                           std::back_inserter(laneResults));

            for (const auto &lResult : laneResults) {
                const auto &lane = mGlobalEnvironmentModel->getLane(lResult.second);
                double distance = boost::geometry::distance(senderPosition, lane->getShape()) - lane->getWidth() / 2;
                if(distance < minDistance){
                    minDistance = distance;
                }
            }
            if (minDistance < detectionParameters->maxDistanceFromRoad) {
                return 1;
            }
        }
        return 1;
    }

    double LegacyChecks::FrequencyCheck(long newTime, long oldTime) const {
        if (newTime - oldTime > (long) (detectionParameters->maxCamFrequency * 1000)) {
            return 0;
        } else {
            return 1;
        }
    }


    double
    LegacyChecks::PositionHeadingConsistencyCheck(const HeadingValue_t &currentHeading, Position &currentPosition,
                                                  Position &oldPosition,
                                                  double deltaTime, double currentSpeed) const {
        if (deltaTime < detectionParameters->positionHeadingTime) {
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
            if (angleDelta > detectionParameters->maxHeadingChange) {
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
                if (isnan(ret_1)) {
                    ret_1 = 0;
                }

                if (ret_1 < 0.5) {
                    ret_1 = 0;
                } else {
                    ret_1 = 1;
                }

                double ret_2 = 1 - sqrt(pow(deltaPosition[1], 2.0) + pow(deltaPosition[3], 2.0)) /
                                   (detectionParameters->kalmanSpeedRange * currentSpeedConfidence * deltaTime);
                if (isnan(ret_2)) {
                    ret_2 = 0;
                }

                if (ret_2 < 0.5) {
                    ret_2 = 0;
                } else {
                    ret_2 = 1;
                }

                returnValue[0] = ret_1;
                returnValue[1] = ret_2;
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
                if (isnan(ret_1)) {
                    ret_1 = 0;
                }

                if (ret_1 < 0.5) {
                    ret_1 = 0;
                } else {
                    ret_1 = 1;
                }
                double ret_2 =
                        1 - (deltaPosition[1] / (detectionParameters->kalmanSpeedRange * curSpdConfX * deltaTime));
                if (isnan(ret_2)) {
                    ret_2 = 0;
                }
                if (ret_2 < 0.5) {
                    ret_2 = 0;
                } else {
                    ret_2 = 1;
                }
                returnValue[0] = ret_1;
                returnValue[1] = ret_2;
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

                if (isnan(ret_1)) {
                    ret_1 = 0;
                }
                if (ret_1 < 0.5) {
                    ret_1 = 0;
                } else {
                    ret_1 = 1;
                }

                return ret_1;
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
                if (isnan(ret_1)) {
                    ret_1 = 0;
                }

                if (ret_1 < 0.5) {
                    ret_1 = 0;
                } else {
                    ret_1 = 1;
                }

                return ret_1;
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
                if (isnan(ret_1)) {
                    ret_1 = 0;
                }
                if (ret_1 < 0.5) {
                    ret_1 = 0;
                } else {
                    ret_1 = 1;
                }

                return ret_1;
            } else {
                kalmanVI->setInitial(currentSpeed.x.value(), currentSpeed.y.value());
                return 1;
            }
        }
    }

    Position LegacyChecks::convertCamPosition(const ReferencePosition_t &referencePosition) {
        traci::TraCIGeoPosition traciGeoPosition = {
                (double) referencePosition.longitude / 10000000.0,
                (double) referencePosition.latitude / 10000000.0};
        traci::TraCIPosition traciPosition = traciAPI->convert2D(traciGeoPosition);
        return Position(traciPosition.x, traciPosition.y);
    }

    Position LegacyChecks::getVector(const double &value, const double &angle) {
        double x = value * sin(angle * PI / 180);
        double y = sqrt(pow(value, 2) - pow(x, 2));
        if (angle > 90 && angle < 270) {
            y *= -1;
        }
        return Position(x, y);
    }

    CheckResult *LegacyChecks::checkCAM(const Position &receiverPosition, TrackedObjectsFilterRange &envModObjects,
                                        const vanetza::asn1::Cam &currentCam, const vanetza::asn1::Cam *lastCamPtr) {

        BasicVehicleContainerHighFrequency_t highFrequencyContainer = currentCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;

        Position currentCamPosition = convertCamPosition(
                currentCam->cam.camParameters.basicContainer.referencePosition);

        double currentCamSpeed = (double) highFrequencyContainer.speed.speedValue / 100.0;
        double currentCamSpeedConfidence = (double) highFrequencyContainer.speed.speedConfidence / 100.0;
        double currentCamAcceleration =
                (double) highFrequencyContainer.longitudinalAcceleration.longitudinalAccelerationValue / 10.0;
        double currentCamHeading = (double) highFrequencyContainer.heading.headingValue / 10.0;
        Position currentCamSpeedVector = getVector(currentCamSpeed, currentCamHeading);
        Position currentCamAccelerationVector = getVector(currentCamAcceleration, currentCamHeading);
        auto *result = new CheckResult;
        result->positionPlausibility = PositionPlausibilityCheck(currentCamPosition, currentCamSpeed);
        result->speedPlausibility = SpeedPlausibilityCheck(currentCamSpeed);
        result->proximityPlausibility = ProximityPlausibilityCheck(currentCamPosition, receiverPosition, envModObjects);
        result->rangePlausibility = RangePlausibilityCheck(currentCamPosition, receiverPosition);

        if (lastCamPtr != nullptr) {
            vanetza::asn1::Cam lastCam = *lastCamPtr;
            Position lastCamPosition = convertCamPosition(lastCam->cam.camParameters.basicContainer.referencePosition);
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
                    highFrequencyContainer.heading.headingValue,
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
            //TODO IntersectionCheck
            result->frequency = FrequencyCheck(currentCam->cam.generationDeltaTime, lastCam->cam.generationDeltaTime);
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
            traciAPI(std::move(traciAPI)),
            detectionParameters(detectionParameters),
            kalmanSVI(kalmanSVI), kalmanSVSI(kalmanSVSI),
            kalmanSI(kalmanSI), kalmanVI(kalmanVI) {
            if(!staticInitializationComplete){
                staticInitializationComplete = true;
                mGlobalEnvironmentModel = globalEnvironmentModel;
            }

    }
}