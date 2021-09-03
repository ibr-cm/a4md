//
// Created by bastian on 12.07.21.
//

#include "CatchChecks.h"
#include <utility>
#include <artery/traci/Cast.h>
#include "artery/application/misbehavior/util/HelperFunctions.h"
#include "artery/application/misbehavior/util/DetectionLevels.h"

namespace artery {

    using namespace omnetpp;


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
        for (int radius = 0; radius <= iterationCount; radius++) {
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
        return 1 - (double) failedCount / totalCount;
    }

    double CatchChecks::SpeedPlausibilityCheck(const double &currentSpeed, const double &currentSpeedConfidence) {
//        return (detectionParameters->maxPlausibleSpeed - currentSpeed + currentSpeedConfidence) /
//               (2 * currentSpeedConfidence);
        if (currentSpeed + currentSpeedConfidence / 2 < detectionParameters->maxPlausibleSpeed) {
            return 1;
        } else if (currentSpeed + currentSpeedConfidence / 2 > detectionParameters->maxPlausibleSpeed) {
            return 0;
        } else {
            return (currentSpeedConfidence / 2 + (detectionParameters->maxPlausibleSpeed - currentSpeed)) /
                   currentSpeedConfidence;
        }
    }

    double CatchChecks::ProximityPlausibilityCheck(const Position &senderPosition, const Position &receiverPosition,
                                                   const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &surroundingCamObjects) {
        Position::value_type deltaDistance = distance(senderPosition, receiverPosition);
        double deltaAngle = calculateHeadingAngle(
                Position(senderPosition.x - receiverPosition.x, senderPosition.y - receiverPosition.y));
        if (deltaDistance.value() < detectionParameters->maxProximityRangeL) {
            if (deltaDistance.value() < detectionParameters->maxProximityRangeW * 2 ||
                (deltaAngle < 90 && deltaDistance.value() <
                                    (detectionParameters->maxProximityRangeW / cos((90 - deltaAngle) * PI / 180)))) {
                Position::value_type minimumDistance = Position::value_type::from_value(9999);

                for (const auto &cam: surroundingCamObjects) {
                    Position::value_type currentDistance = distance(senderPosition, convertReferencePosition(
                            (*cam)->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary,
                            mTraciAPI));
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

    double
    CatchChecks::RangePlausibilityCheck(const Position &senderPosition, const std::vector<Position> &senderEllipse,
                                        const double &senderEllipseRadius,
                                        const Position &receiverPosition, const std::vector<Position> &receiverEllipse,
                                        const double &receiverEllipseRadius) {
        return confidenceIntersectionArea(senderPosition, senderEllipse, senderEllipseRadius,
                                          receiverPosition, receiverEllipse, receiverEllipseRadius,
                                          detectionParameters->maxPlausibleRange);
    }

    double CatchChecks::PositionConsistencyCheck(const Position &currentPosition,
                                                 const std::vector<Position> &currentEllipse,
                                                 const double &currentEllipseRadius,
                                                 const Position &oldPosition, const std::vector<Position> &oldEllipse,
                                                 const double &oldEllipseRadius,
                                                 const double &deltaTime) {
        return confidenceIntersectionArea(currentPosition, currentEllipse, currentEllipseRadius,
                                          oldPosition, oldEllipse, oldEllipseRadius,
                                          detectionParameters->maxPlausibleSpeed * deltaTime);
    }

    double CatchChecks::SpeedConsistencyCheck(const double &currentSpeed, const double &currentSpeedConfidence,
                                              const double &oldSpeed, const double &oldSpeedConfidence,
                                              const double &deltaTime) {
//        double f_max = (detectionParameters->maxPlausibleAcceleration - currentSpeed + oldSpeedConfidence) /
//                       (4 * oldSpeedConfidence) +
//                       (detectionParameters->maxPlausibleAcceleration - currentSpeed + currentSpeedConfidence) /
//                       (4 * currentSpeedConfidence);
//        double f_min = (currentSpeed - detectionParameters->maxPlausibleDeceleration + oldSpeed) /
//                       (4 * oldSpeedConfidence) +
//                       (currentSpeed - detectionParameters->maxPlausibleDeceleration + currentSpeedConfidence) /
//                       (4 * currentSpeedConfidence);
//        if (currentSpeed > oldSpeed) {
//            return f_max;
//        } else {
//            return f_min;
//        }
        double deltaSpeed = currentSpeed - oldSpeed;
        double acceleration = deltaSpeed > 0 ? detectionParameters->maxPlausibleAcceleration
                                             : detectionParameters->maxPlausibleDeceleration;
        return segmentSegmentFactor(fabs(deltaSpeed), currentSpeedConfidence, oldSpeedConfidence,
                                    acceleration * deltaTime);
    }

    double CatchChecks::PositionSpeedConsistencyCheck(const Position &currentPosition,
                                                      const std::vector<Position> &currentEllipse,
                                                      const double &currentEllipseRadius,
                                                      const Position &oldPosition,
                                                      const std::vector<Position> &oldEllipse,
                                                      const double &oldEllipseRadius,
                                                      const double &currentSpeed, const double &currentSpeedConfidence,
                                                      const double &oldSpeed, const double &oldSpeedConfidence,
                                                      const double &deltaTime) {
        if (deltaTime > detectionParameters->maxTimeDelta) {
            return 1;
        }
        double deltaDistance = distance(currentPosition, oldPosition).value();

        double currentTest1 = currentSpeed + currentSpeedConfidence;
        double oldTest1 = oldSpeed - oldSpeedConfidence;

        double currentTest2 = currentSpeed - currentSpeedConfidence;
        double oldTest2 = oldSpeed + oldSpeedConfidence;

        if (currentTest2 < oldTest2) {
            currentTest2 = (currentSpeed + oldSpeed) / 2;
            oldTest2 = (currentSpeed + oldSpeed) / 2;
        }

        double returnDistance1[2];
        calculateMaxMinDist(currentTest1, oldTest1, deltaTime, detectionParameters->maxPlausibleAcceleration,
                            detectionParameters->maxPlausibleDeceleration, detectionParameters->maxPlausibleSpeed,
                            returnDistance1);

        double factorMin1 =
                1 - confidenceIntersectionArea(currentPosition, currentEllipse, currentEllipseRadius, oldPosition,
                                               oldEllipse, oldEllipseRadius, returnDistance1[0]);
        double factorMax1 = oneSidedCircleSegmentFactor(deltaDistance,
                                                        currentEllipseRadius,
                                                        oldEllipseRadius,
                                                        returnDistance1[1] + detectionParameters->maxMgtRngUp);

        double returnDistance2[2];
        calculateMaxMinDist(currentTest2, oldTest2, deltaTime, detectionParameters->maxPlausibleAcceleration,
                            detectionParameters->maxPlausibleDeceleration, detectionParameters->maxPlausibleSpeed,
                            returnDistance2);
        double factorMin2 =
                1 - confidenceIntersectionArea(currentPosition, currentEllipse, currentEllipseRadius, oldPosition,
                                               oldEllipse, oldEllipseRadius, returnDistance2[0]);
        double factorMax2 = oneSidedCircleSegmentFactor(deltaDistance,
                                                        currentEllipseRadius,
                                                        oldEllipseRadius,
                                                        returnDistance2[1] + detectionParameters->maxMgtRngUp);

        double returnDistance0[2];
        calculateMaxMinDist(currentSpeed, oldSpeed, deltaTime, detectionParameters->maxPlausibleAcceleration,
                            detectionParameters->maxPlausibleDeceleration, detectionParameters->maxPlausibleSpeed,
                            returnDistance0);
        double factorMin0 =
                1 - confidenceIntersectionArea(currentPosition, currentEllipse, currentEllipseRadius, oldPosition,
                                               oldEllipse, oldEllipseRadius, returnDistance0[0]);
        double factorMax0 = oneSidedCircleSegmentFactor(deltaDistance, currentEllipseRadius, oldEllipseRadius,
                                                        returnDistance0[1] + detectionParameters->maxMgtRngUp);

        double factorMin = (factorMin0 + factorMin1 + factorMin2) / 3;
        double factorMax = (factorMax0 + factorMax1 + factorMax2) / 3;
        return std::min(factorMin, factorMax);

    }

    double CatchChecks::PositionSpeedMaxConsistencyCheck(const Position &currentPosition,
                                                         const PosConfidenceEllipse_t &currentPositionConfidence,
                                                         const Position &oldPosition,
                                                         const PosConfidenceEllipse_t &oldConfidenceEllipse,
                                                         const double &currentSpeed,
                                                         const double &currentSpeedConfidence,
                                                         const double &oldSpeed, const double &oldSpeedConfidence,
                                                         const double &deltaTime) {
        if (deltaTime > detectionParameters->maxTimeDelta) {
            return 1;
        }
        double deltaDistance = distance(currentPosition, oldPosition).value();
        double theoreticalSpeed = deltaDistance / deltaTime;
        double maxSpeed = std::max(currentSpeed, oldSpeed);
        double minSpeed = std::min(currentSpeed, oldSpeed);

        double currentRange =
                ((double) currentPositionConfidence.semiMajorConfidence / 100) / deltaTime + currentSpeedConfidence;
        double oldRange = ((double) oldConfidenceEllipse.semiMajorConfidence / 100) / deltaTime + oldSpeedConfidence;

        double maxFactor = oneSidedCircleSegmentFactor(maxSpeed - theoreticalSpeed, currentRange, oldRange,
                                                       (detectionParameters->maxPlausibleDeceleration +
                                                        detectionParameters->maxMgtRng) * deltaTime);

        double minFactor = oneSidedCircleSegmentFactor(theoreticalSpeed - minSpeed, currentRange, oldRange,
                                                       (detectionParameters->maxPlausibleAcceleration +
                                                        detectionParameters->maxMgtRng) * deltaTime);
        return std::min(minFactor, maxFactor);
    }

    double CatchChecks::PositionHeadingConsistencyCheck(const double &currentHeading,
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

            double currentFactorLow;
            if (currentPositionConfidence.semiMajorConfidence == 0) {
                currentFactorLow = angleLow < detectionParameters->maxHeadingChange ? 1 : 0;
            } else {
                currentFactorLow = calculateNormedCircleAreaWithoutSegment(
                        (double) currentPositionConfidence.semiMajorConfidence / 100, xLow, true);
            }

            double oldFactorLow;
            if (oldPositionConfidence.semiMajorConfidence == 0) {
                oldFactorLow = angleLow < detectionParameters->maxHeadingChange ? 1 : 0;
            } else {
                oldFactorLow = calculateNormedCircleAreaWithoutSegment(
                        (double) oldPositionConfidence.semiMajorConfidence / 100, xLow, true);
            }


            double currentFactorHigh;
            if (currentPositionConfidence.semiMajorConfidence == 0) {
                currentFactorHigh = angleHigh < detectionParameters->maxHeadingChange ? 1 : 0;
            } else {
                currentFactorHigh = calculateNormedCircleAreaWithoutSegment(
                        (double) currentPositionConfidence.semiMajorConfidence / 100, xHigh, true);
            }

            double oldFactorHigh;
            if (oldPositionConfidence.semiMajorConfidence == 0) {
                oldFactorHigh = angleHigh < detectionParameters->maxHeadingChange ? 1 : 0;
            } else {
                oldFactorHigh = calculateNormedCircleAreaWithoutSegment(
                        (double) oldPositionConfidence.semiMajorConfidence / 100, xHigh, true);
            }
            return (currentFactorLow + oldFactorLow + currentFactorHigh + oldFactorHigh) / 4;
        }
    }

    double CatchChecks::IntersectionCheck(const std::vector<Position> &receiverEllipse,
                                          const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &relevantCams,
                                          const std::vector<Position> &senderEllipse,
                                          const double &deltaTime) {
        int totalCount = 1;
        double intersectionSum = 1.0 - (intersectionFactor(senderEllipse, receiverEllipse) *
                                        ((detectionParameters->maxIntersectionDeltaTime - deltaTime) /
                                         detectionParameters->maxIntersectionDeltaTime));
        for (const auto &cam: relevantCams) {
            Position camPosition = convertReferencePosition(
                    (*cam)->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
            PosConfidenceEllipse_t camPositionConfidence =
                    (*cam)->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse;
            std::vector<Position> camPositionEllipse = createEllipse(camPosition, camPositionConfidence);
            totalCount++;
            intersectionSum += intersectionFactor(senderEllipse, camPositionEllipse);
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
        return 1 - area / (PI * pow((double) senderConfidenceEllipse.semiMajorConfidence / 100, 2.0));
    }


    std::shared_ptr<CheckResult> CatchChecks::checkCAM(const VehicleDataProvider *receiverVDP,
                                                       const std::vector<Position> &receiverVehicleOutline,
                                                       const std::shared_ptr<vanetza::asn1::Cam> &currentCam,
                                                       const std::shared_ptr<vanetza::asn1::Cam> &lastCam,
                                                       const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &surroundingCamObjects) {
        const Position &receiverPosition = convertReferencePosition(receiverVDP->approximateReferencePosition(),
                                                                    mSimulationBoundary, mTraciAPI);
        const PosConfidenceEllipse_t &receiverPositionConfidence =
                receiverVDP->approximateReferencePosition().positionConfidenceEllipse;
        const std::vector<Position> receiverPositionEllipse = createEllipse(receiverPosition,
                                                                            receiverPositionConfidence);
        double receiverEllipseRadius = (double) receiverPositionConfidence.semiMajorConfidence / 100;

        Position currentCamPosition = convertReferencePosition(
                (*currentCam)->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
        PosConfidenceEllipse_t currentCamPositionConfidence =
                (*currentCam)->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse;
        std::vector<Position> currentCamPositionEllipse = createEllipse(currentCamPosition,
                                                                        currentCamPositionConfidence);
        double currentCamEllipseRadius = (double) currentCamPositionConfidence.semiMajorConfidence / 100;

        BasicVehicleContainerHighFrequency_t currentHfc =
                (*currentCam)->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        double currentCamSpeed = (double) currentHfc.speed.speedValue / 100.0;
        double currentCamSpeedConfidence = (double) currentHfc.speed.speedConfidence / 100.0;
        double currentCamAcceleration =
                (double) currentHfc.longitudinalAcceleration.longitudinalAccelerationValue / 10.0;
        double currentCamHeading = (double) currentHfc.heading.headingValue / 10.0;
        double currentCamHeadingConfidence = (double) currentHfc.heading.headingConfidence / 10.0;
        Position currentCamSpeedVector = getVector(currentCamSpeed, currentCamHeading);
        Position currentCamAccelerationVector = getVector(currentCamAcceleration, currentCamHeading);

        std::shared_ptr<CheckResult> result = std::make_shared<CheckResult>();
        result->positionPlausibility =
                PositionPlausibilityCheck(currentCamPosition, currentCamPositionConfidence, currentCamSpeed,
                                          currentCamSpeedConfidence);
        result->speedPlausibility =
                SpeedPlausibilityCheck(currentCamSpeed, currentCamSpeedConfidence);
        result->proximityPlausibility =
                ProximityPlausibilityCheck(currentCamPosition, receiverPosition, surroundingCamObjects);
        result->rangePlausibility =
                RangePlausibilityCheck(currentCamPosition, currentCamPositionEllipse, currentCamEllipseRadius,
                                       receiverPosition, receiverPositionEllipse, receiverEllipseRadius);

        if (lastCam != nullptr) {
            auto camDeltaTime = (double) (uint16_t) ((*currentCam)->cam.generationDeltaTime -
                                                     (*lastCam)->cam.generationDeltaTime);
            result->consistencyIsChecked = true;
            result->positionConsistency =
                    PositionConsistencyCheck(currentCamPosition, currentCamPositionEllipse, currentCamEllipseRadius,
                                             mLastCamPosition, mLastCamPositionEllipse, mLastCamEllipseRadius,
                                             camDeltaTime);
            result->speedConsistency =
                    SpeedConsistencyCheck(currentCamSpeed, currentCamSpeedConfidence, mLastCamSpeed,
                                          mLastCamSpeedConfidence, camDeltaTime);


            result->positionSpeedConsistency =
                    PositionSpeedConsistencyCheck(currentCamPosition, currentCamPositionEllipse,
                                                  currentCamEllipseRadius,
                                                  mLastCamPosition, mLastCamPositionEllipse, mLastCamEllipseRadius,
                                                  currentCamSpeed, currentCamSpeedConfidence,
                                                  mLastCamSpeed, mLastCamSpeedConfidence, camDeltaTime);
            result->positionSpeedMaxConsistency =
                    PositionSpeedMaxConsistencyCheck(currentCamPosition, currentCamPositionConfidence, mLastCamPosition,
                                                     mLastCamPositionConfidence, currentCamSpeed,
                                                     currentCamSpeedConfidence, mLastCamSpeed, mLastCamSpeedConfidence,
                                                     camDeltaTime);
            result->positionHeadingConsistency =
                    PositionHeadingConsistencyCheck(currentCamHeading, currentCamHeadingConfidence, currentCamPosition,
                                                    currentCamPositionConfidence, mLastCamPosition,
                                                    mLastCamPositionConfidence, currentCamSpeed,
                                                    currentCamSpeedConfidence, camDeltaTime);
            result->frequency =
                    FrequencyCheck(camDeltaTime);
            result->intersection =
                    IntersectionCheck(receiverPositionEllipse, surroundingCamObjects, currentCamPositionEllipse,
                                      camDeltaTime);
            KalmanChecks(currentCamPosition, currentCamPositionConfidence, currentCamSpeed,
                         currentCamSpeedVector, currentCamSpeedConfidence, currentCamAcceleration,
                         currentCamAccelerationVector, currentCamHeading, mLastCamPosition,
                         mLastCamSpeedVector, camDeltaTime, result);
        } else {
            result->suddenAppearance =
                    SuddenAppearanceCheck(currentCamPosition, currentCamPositionConfidence, receiverPosition,
                                          receiverPositionConfidence);
        }
        mLastCamPosition = currentCamPosition;
        mLastCamPositionConfidence = currentCamPositionConfidence;
        mLastCamPositionEllipse = currentCamPositionEllipse;
        mLastCamEllipseRadius = currentCamEllipseRadius;
        mLastCamSpeed = currentCamSpeed;
        mLastCamSpeedConfidence = currentCamSpeedConfidence;
        mLastCamSpeedVector = currentCamSpeedVector;
        return result;
    }

    std::bitset<16> CatchChecks::checkSemanticLevel1Report(const vanetza::asn1::Cam &currentCam) {
        std::shared_ptr<CheckResult> result = std::make_shared<CheckResult>();

        BasicVehicleContainerHighFrequency_t currentHfc =
                currentCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        double currentCamSpeed = (double) currentHfc.speed.speedValue / 100.0;
        double currentCamSpeedConfidence = (double) currentHfc.speed.speedConfidence / 100.0;

        result->speedPlausibility = SpeedPlausibilityCheck(currentCamSpeed, currentCamSpeedConfidence);
        return mThresholdFusion->checkForReport(*result)[detectionLevels::Level1];
    }

    std::bitset<16> CatchChecks::checkSemanticLevel2Report(const vanetza::asn1::Cam &currentCam,
                                                           const vanetza::asn1::Cam &lastCam) {
        std::shared_ptr<CheckResult> result = std::make_shared<CheckResult>();
        initializeKalmanFilters(lastCam);

        Position currentCamPosition = convertReferencePosition(
                currentCam->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
        PosConfidenceEllipse_t currentCamPositionConfidence =
                currentCam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse;
        std::vector<Position> currentCamPositionEllipse = createEllipse(currentCamPosition,
                                                                        currentCamPositionConfidence);
        double currentCamEllipseRadius = (double) currentCamPositionConfidence.semiMajorConfidence / 100;

        BasicVehicleContainerHighFrequency_t currentCamHfc =
                currentCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        double currentCamSpeed = (double) currentCamHfc.speed.speedValue / 100.0;
        double currentCamSpeedConfidence = (double) currentCamHfc.speed.speedConfidence / 100.0;
        double currentCamAcceleration =
                (double) currentCamHfc.longitudinalAcceleration.longitudinalAccelerationValue / 10.0;
        double currentCamHeading = (double) currentCamHfc.heading.headingValue / 10;
        double currentCamHeadingConfidence = (double) currentCamHfc.heading.headingConfidence / 10.0;
        Position currentCamSpeedVector = getVector(currentCamSpeed, currentCamHeading);
        Position currentCamAccelerationVector = getVector(currentCamAcceleration, currentCamHeading);

        Position lastCamPosition = convertReferencePosition(
                lastCam->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
        PosConfidenceEllipse_t lastCamPositionConfidence =
                lastCam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse;
        std::vector<Position> lastCamPositionEllipse = createEllipse(lastCamPosition,
                                                                     lastCamPositionConfidence);
        double lastCamEllipseRadius = (double) lastCamPositionConfidence.semiMajorConfidence / 100;

        BasicVehicleContainerHighFrequency_t lastCamHfc =
                lastCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        double lastCamSpeed = (double) lastCamHfc.speed.speedValue / 100.0;
        double lastCamSpeedConfidence = (double) lastCamHfc.speed.speedConfidence / 100.0;
        double lastCamAcceleration =
                (double) lastCamHfc.longitudinalAcceleration.longitudinalAccelerationValue / 10.0;
        double lastCamHeading = (double) lastCamHfc.heading.headingValue / 10;
        Position lastCamSpeedVector = getVector(lastCamSpeed, lastCamHeading);
        Position lastCamAccelerationVector = getVector(lastCamAcceleration, currentCamHeading);

        auto camDeltaTime = (double) (uint16_t) (currentCam->cam.generationDeltaTime -
                                                 lastCam->cam.generationDeltaTime);
        result->consistencyIsChecked = true;

        result->positionConsistency =
                PositionConsistencyCheck(currentCamPosition, currentCamPositionEllipse, currentCamEllipseRadius,
                                         lastCamPosition, lastCamPositionEllipse, lastCamEllipseRadius,
                                         camDeltaTime);
        result->speedConsistency =
                SpeedConsistencyCheck(currentCamSpeed, currentCamSpeedConfidence, lastCamSpeed,
                                      lastCamSpeedConfidence, camDeltaTime);


        result->positionSpeedConsistency =
                PositionSpeedConsistencyCheck(currentCamPosition, currentCamPositionEllipse,
                                              currentCamEllipseRadius,
                                              lastCamPosition, lastCamPositionEllipse, lastCamEllipseRadius,
                                              currentCamSpeed, currentCamSpeedConfidence,
                                              lastCamSpeed, lastCamSpeedConfidence, camDeltaTime);
        result->positionSpeedMaxConsistency =
                PositionSpeedMaxConsistencyCheck(currentCamPosition, currentCamPositionConfidence, lastCamPosition,
                                                 lastCamPositionConfidence, currentCamSpeed,
                                                 currentCamSpeedConfidence, lastCamSpeed, lastCamSpeedConfidence,
                                                 camDeltaTime);
        result->positionHeadingConsistency =
                PositionHeadingConsistencyCheck(currentCamHeading, currentCamHeadingConfidence, currentCamPosition,
                                                currentCamPositionConfidence, lastCamPosition,
                                                lastCamPositionConfidence, currentCamSpeed,
                                                currentCamSpeedConfidence, camDeltaTime);
        KalmanChecks(currentCamPosition, currentCamPositionConfidence, currentCamSpeed,
                     currentCamSpeedVector, currentCamSpeedConfidence, currentCamAcceleration,
                     currentCamAccelerationVector, currentCamHeading, lastCamPosition,
                     lastCamSpeedVector, camDeltaTime, result);
        result->frequency = BaseChecks::FrequencyCheck(camDeltaTime);
        std::cout << result->toString(0.5) << std::endl;

        return mThresholdFusion->checkForReport(*result)[detectionLevels::Level2];

    }

    std::bitset<16> CatchChecks::checkSemanticLevel3Report(const vanetza::asn1::Cam &currentCam,
                                                           const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &neighbourCams) {
        std::shared_ptr<CheckResult> result = std::make_shared<CheckResult>();

        Position currentCamPosition = convertReferencePosition(
                currentCam->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
        PosConfidenceEllipse_t currentCamPositionConfidence =
                currentCam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse;
        std::vector<Position> currentCamPositionEllipse = createEllipse(currentCamPosition,
                                                                        currentCamPositionConfidence);

        BasicVehicleContainerHighFrequency_t currentCamHfc =
                currentCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        double currentCamSpeed = (double) currentCamHfc.speed.speedValue / 100.0;
        double currentCamSpeedConfidence = (double) currentCamHfc.speed.speedConfidence / 100.0;
        double currentCamHeading = (double) currentCamHfc.heading.headingValue / 10;
        double currentCamVehicleLength = (double) currentCamHfc.vehicleLength.vehicleLengthValue / 10;
        double currentCamVehicleWidth = (double) currentCamHfc.vehicleWidth / 10;

        std::vector<Position> currentCamOutline = getVehicleOutline(currentCamPosition,
                                                                    Angle::from_degree(currentCamHeading),
                                                                    currentCamVehicleLength, currentCamVehicleWidth);
        auto camDeltaTime = (double) (uint16_t) ((uint16_t) countTaiMilliseconds(mTimer->getTimeFor(simTime())) -
                                                 (*currentCam).cam.generationDeltaTime);
        result->positionPlausibility =
                PositionPlausibilityCheck(currentCamPosition, currentCamPositionConfidence, currentCamSpeed,
                                          currentCamSpeedConfidence);
        result->intersection =
                IntersectionCheck(currentCamOutline, neighbourCams, currentCamPositionEllipse,
                                  camDeltaTime);

        return mThresholdFusion->checkForReport(*result)[detectionLevels::Level3];
    }

    std::bitset<16>
    CatchChecks::checkSemanticLevel4Report(const vanetza::asn1::Cam &currentCam, const Position &receiverPosition,
                                           const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &neighbourCams) {
        std::shared_ptr<CheckResult> result = std::make_shared<CheckResult>();
//        Position currentCamPosition = convertReferencePosition(
//                currentCam->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
//
//        result->proximityPlausibility = ProximityPlausibilityCheck(currentCamPosition, receiverPosition,
//                                                                   neighbourCams);
//        result->rangePlausibility = RangePlausibilityCheck(currentCamPosition, receiverPosition);

        return mThresholdFusion->checkForReport(*result)[detectionLevels::Level4];
    }

    CatchChecks::CatchChecks(std::shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                             DetectionParameters *detectionParameters, const Timer *timer,
                             const std::shared_ptr<vanetza::asn1::Cam> &message)
            : BaseChecks(std::move(traciAPI), globalEnvironmentModel, detectionParameters, timer, message) {
        mThresholdFusion = nullptr;
    }

    CatchChecks::CatchChecks(std::shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                             DetectionParameters *detectionParameters, double misbehaviorThreshold, const Timer *timer)
            : BaseChecks(std::move(traciAPI), globalEnvironmentModel, detectionParameters, timer) {
        mThresholdFusion = new ThresholdFusion(misbehaviorThreshold);
    }


} // namespace artery
