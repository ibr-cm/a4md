//
// Created by bastian on 12.07.21.
//

#include "CatchChecks.h"
#include <utility>
#include <artery/traci/Cast.h>
#include "artery/application/md/util/HelperFunctions.h"

namespace artery {

    using namespace omnetpp;


    double CatchChecks::ProximityPlausibilityCheck(Position &testPosition, const Position &myPosition,
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
                if (minimumDistance.value() < 2 * detectionParameters->maxProximityDistance) {
                    return 1 - minimumDistance.value() / (2 * detectionParameters->maxProximityDistance);
                } else {
                    return 0;
                }
            }
        }
        return 1;
    }

    double CatchChecks::RangePlausibilityCheck(const Position &senderPosition,
                                               const PosConfidenceEllipse_t &senderConfidenceEllipse,
                                               const Position &receiverPosition,
                                               const PosConfidenceEllipse_t &receiverConfidenceEllipse) {
        return confidenceIntersectionArea(senderPosition, senderConfidenceEllipse, receiverPosition,
                                          receiverConfidenceEllipse,
                                          detectionParameters->maxPlausibleRange);
    }


    double
    CatchChecks::PositionConsistencyCheck(const Position &currentPosition,
                                          const PosConfidenceEllipse_t &currentConfidenceEllipse,
                                          const Position &oldPosition,
                                          const PosConfidenceEllipse_t &oldConfidenceEllipse,
                                          double deltaTime) {
        return confidenceIntersectionArea(currentPosition, currentConfidenceEllipse, oldPosition, oldConfidenceEllipse,
                                          detectionParameters->maxPlausibleSpeed * deltaTime);
    }

    double CatchChecks::SpeedConsistencyCheck(const double &currentSpeed, const double &currentSpeedConfidence,
                                              const double &oldSpeed, const double &oldSpeedConfidence,
                                              const double &deltaTime) {
        double f_max = (detectionParameters->maxPlausibleAcceleration - currentSpeed + oldSpeedConfidence) /
                       (4 * oldSpeedConfidence) +
                       (detectionParameters->maxPlausibleAcceleration - currentSpeed + currentSpeedConfidence) /
                       (4 * currentSpeedConfidence);
        double f_min = (currentSpeed - detectionParameters->maxPlausibleDeceleration + oldSpeed) /
                       (4 * oldSpeedConfidence) +
                       (currentSpeed - detectionParameters->maxPlausibleDeceleration + currentSpeedConfidence) /
                       (4 * currentSpeedConfidence);
        if (currentSpeed > oldSpeed) {
            return f_max;
        } else {
            return f_min;
        }
    }

    double CatchChecks::SpeedPlausibilityCheck(const double &currentSpeed, const double &currentSpeedConfidence) {
        return (detectionParameters->maxPlausibleSpeed - currentSpeed + currentSpeedConfidence) /
               (2 * currentSpeedConfidence);
    }



    double CatchChecks::PositionSpeedMaxConsistencyCheck(const Position &currentPosition,
                                                         const PosConfidenceEllipse_t &currentConfidenceEllipse,
                                                         const Position &oldPosition,
                                                         const PosConfidenceEllipse_t &oldConfidenceEllipse,
                                                         double currentSpeed, double currentSpeedConfidence,
                                                         double oldSpeed, double oldSpeedConfidence,
                                                         double deltaTime) {
        if (deltaTime > detectionParameters->maxTimeDelta) {
            return 1;
        }
        double deltaDistance = distance(currentPosition, oldPosition).value();
        double theoreticalSpeed = deltaDistance / deltaTime;
        double maxSpeed = std::max(currentSpeed, oldSpeed);
        double minSpeed = std::min(currentSpeed, oldSpeed);

        double currentRange =
                ((double) currentConfidenceEllipse.semiMajorConfidence / 100) / deltaTime + currentSpeedConfidence;
        double oldRange = ((double) oldConfidenceEllipse.semiMajorConfidence / 100) / deltaTime + oldSpeedConfidence;

        double maxFactor = oneSidedCircleSegmentFactor(maxSpeed - theoreticalSpeed, currentRange, oldRange,
                                                       (detectionParameters->maxPlausibleDeceleration +
                                                        detectionParameters->maxMgtRng) * deltaTime);

        double minFactor = oneSidedCircleSegmentFactor(theoreticalSpeed - minSpeed, currentRange, oldRange,
                                                       (detectionParameters->maxPlausibleAcceleration +
                                                        detectionParameters->maxMgtRng) * deltaTime);
        return std::min(minFactor, maxFactor);
    }


    double CatchChecks::IntersectionCheck(const std::vector<Position> &receiverVehicleOutline,
                                          const std::vector<vanetza::asn1::Cam *> &relevantCams,
                                          const Position &senderPosition, const double &senderLength,
                                          const double &senderWidth, const double &senderHeading,
                                          const double &deltaTime) {
        std::vector<Position> senderOutline = getVehicleOutline(senderPosition, Angle::from_degree(senderHeading),
                                                                senderLength, senderWidth);
        int totalCount = 1;
        double intersectionSum = 1.01 - (intersectionFactor(senderOutline, receiverVehicleOutline) *
                                         ((detectionParameters->maxIntersectionDeltaTime - deltaTime) /
                                          detectionParameters->maxIntersectionDeltaTime));
        for (auto cam : relevantCams) {
            std::vector<Position> outline = getVehicleOutline((*cam), mSimulationBoundary, mTraciAPI);
            totalCount++;
            intersectionSum += intersectionFactor(senderOutline, outline);
        }
        return intersectionSum / totalCount;
    }

    double CatchChecks::SuddenAppearanceCheck(const Position &senderPosition,
                                              const PosConfidenceEllipse_t &senderConfidenceEllipse,
                                              const Position &receiverPosition,
                                              const PosConfidenceEllipse_t &receiverConfidenceEllipse) {
        double deltaDistance = distance(senderPosition, receiverPosition).value();

        double receiverMaxRadius = detectionParameters->maxSuddenAppearanceRange +
                                   (double) receiverConfidenceEllipse.semiMajorConfidence / 100;
        if (senderConfidenceEllipse.semiMajorConfidence == 0 && receiverMaxRadius < deltaDistance) {
            return 0;
        }
        std::vector<Position> receiverCircle = createCircle(receiverPosition, receiverMaxRadius, 36);
        std::vector<Position> senderEllipse = createEllipse(senderPosition, senderConfidenceEllipse);
        double area = getIntersectionArea(receiverCircle, senderEllipse);
        return 1 - (area / PI * pow((double) senderConfidenceEllipse.semiMajorConfidence / 100, 2.0));
    }

    double CatchChecks::PositionPlausibilityCheck(const Position &senderPosition,
                                                  const PosConfidenceEllipse_t &senderPositionConfidence,
                                                  const double &senderSpeed, const double &senderSpeedConfidence) {
        if (std::max(senderSpeed - senderSpeedConfidence, 0.0) < detectionParameters->maxOffroadSpeed) {
            return 1;
        }
        int iterationCount = 5;
        double semiMajorConfidence = (double) senderPositionConfidence.semiMajorConfidence / 100;
        double semiMinorConfidence = (double) senderPositionConfidence.semiMinorConfidence / 100;
        double semiMajorOrientation = (double) senderPositionConfidence.semiMajorOrientation / 10;
        double semiMajorMultiplicator = semiMajorConfidence / iterationCount;
        double semiMinorMultiplicator = semiMinorConfidence / iterationCount;
        int totalCount = 0;
        int failedCount = 0;
        for (int radius = radius; radius <= iterationCount; radius++) {
            int pointCount = int(pow(3 * radius, 1.3));
            for (int i = 0; i < pointCount; i++) {
                double borderX = cos(2 * PI / pointCount * i) * semiMajorMultiplicator * i;
                double borderY = sin(2 * PI / pointCount * i) * semiMinorMultiplicator * i;
                double newAngle = semiMajorOrientation + atan2(-borderY, borderX) * 180 / PI;
                newAngle = std::fmod(newAngle, 360);

                double offsetDistance = sqrt(pow(borderX, 2) + pow(borderY, 2));
                double relativeX = offsetDistance * sin(newAngle * PI / 180);
                double relativeY = offsetDistance * cos(newAngle * PI / 180);
                Position p(senderPosition.x.value() + relativeX, senderPosition.y.value() + relativeY);
                if (getDistanceToNearestRoad(mGlobalEnvironmentModel, p) > detectionParameters->maxDistanceFromRoad) {
                    failedCount++;
                }
                totalCount++;
            }
        }
    }


    double CatchChecks::PositionHeadingConsistencyCheckOld(const double &currentHeading,
                                                           const double &currentHeadingConfidence,
                                                           const Position &currentPosition,
                                                           const PosConfidenceEllipse_t &currentPositionConfidence,
                                                           const Position &oldPosition,
                                                           const PosConfidenceEllipse_t &oldPositionConfidence,
                                                           const double &currentSpeed,
                                                           const double &currentSpeedConfidence,
                                                           const double &deltaTime) {
        double deltaDistance = distance(currentPosition, oldPosition).value();
        if (deltaTime < detectionParameters->positionHeadingTime ||
            deltaDistance < 1 ||
            currentSpeed - currentSpeedConfidence < 1) {
            return 1;
        } else {
            libsumo::TraCIPosition currentPositionTraci = position_cast(mSimulationBoundary, currentPosition);
            libsumo::TraCIPosition oldPositionTraci = position_cast(mSimulationBoundary, oldPosition);
            Position deltaPosition = Position(currentPositionTraci.x - oldPositionTraci.x,
                                              currentPositionTraci.y - oldPositionTraci.y);
            // add 90 degree for traci offset
            double positionAngle = std::fmod(calculateHeadingAngle(deltaPosition) + 90, 360);
            double deltaAngle = calculateHeadingDifference(currentHeading, positionAngle);
            if (deltaAngle > 180) {
                deltaAngle = 360 - deltaAngle;
            }

            double angleLow = std::max(deltaAngle - currentHeadingConfidence, 0.0);
            double angleHigh = std::min(deltaAngle + currentHeadingConfidence, 180.0);
            double xLow = deltaDistance * cos(angleLow * PI / 180);
            double xHigh = deltaDistance * cos(angleHigh * PI / 180);

            double currentFactorLow = 1;
            if (currentPositionConfidence.semiMajorConfidence == 0) {
                currentFactorLow = angleLow < detectionParameters->maxHeadingChange ? 1 : 0;
            } else {
                currentFactorLow = calculateNormedCircleAreaWithoutSegment(
                        (double) currentPositionConfidence.semiMajorConfidence / 100, xLow, true);
            }

            double oldFactorLow = 1;
            if (oldPositionConfidence.semiMajorConfidence == 0) {
                oldFactorLow = angleLow < detectionParameters->maxHeadingChange ? 1 : 0;
            } else {
                oldFactorLow = calculateNormedCircleAreaWithoutSegment(
                        (double) oldPositionConfidence.semiMajorConfidence / 100, -xLow, true);
            }


            double currentFactorHigh = 1;
            if (currentPositionConfidence.semiMajorConfidence == 0) {
                currentFactorHigh = angleHigh < detectionParameters->maxHeadingChange ? 1 : 0;
            } else {
                currentFactorHigh = calculateNormedCircleAreaWithoutSegment(
                        (double) currentPositionConfidence.semiMajorConfidence / 100, xHigh, true);
            }

            double oldFactorHigh = 1;
            if (oldPositionConfidence.semiMajorConfidence == 0) {
                oldFactorHigh = angleHigh < detectionParameters->maxHeadingChange ? 1 : 0;
            } else {
                oldFactorHigh = calculateNormedCircleAreaWithoutSegment(
                        (double) oldPositionConfidence.semiMajorConfidence / 100, -xHigh, true);
            }
            return (currentFactorLow + oldFactorLow + currentFactorHigh + oldFactorHigh);
        }
    }


    CheckResult *
    CatchChecks::checkCAM(const VehicleDataProvider *receiverVDP, const std::vector<Position> &receiverVehicleOutline,
                          TrackedObjectsFilterRange &envModObjects,
                          const vanetza::asn1::Cam &currentCam, const vanetza::asn1::Cam *lastCamPtr) {
        BasicVehicleContainerHighFrequency_t highFrequencyContainer = currentCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;

        Position currentCamPosition = convertCamPosition(
                currentCam->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
        const Position &receiverPosition = receiverVDP->position();
        double currentCamSpeed = (double) highFrequencyContainer.speed.speedValue / 100.0;
        double currentCamSpeedConfidence = (double) highFrequencyContainer.speed.speedConfidence / 100.0;
        double currentCamAcceleration =
                (double) highFrequencyContainer.longitudinalAcceleration.longitudinalAccelerationValue / 10.0;
        double currentCamHeading = angle_cast(
                traci::TraCIAngle((double) highFrequencyContainer.heading.headingValue / 10.0)).value.value();
        Position currentCamSpeedVector = getVector(currentCamSpeed, currentCamHeading);
        Position currentCamAccelerationVector = getVector(currentCamAcceleration, currentCamHeading);
        auto *result = new CheckResult;
        return result;
    }

    CatchChecks::CatchChecks(shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                             DetectionParameters *detectionParameters, const vanetza::asn1::Cam &message) : BaseChecks(
            std::move(traciAPI), globalEnvironmentModel, detectionParameters, message) {

    }


} // namespace artery
