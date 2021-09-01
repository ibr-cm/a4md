#include "LegacyChecks.h"
#include <utility>
#include <artery/traci/Cast.h>
#include "artery/application/misbehavior/util/HelperFunctions.h"
#include "artery/application/misbehavior/util/DetectionLevels.h"

namespace artery {

    using namespace omnetpp;

    double LegacyChecks::PositionPlausibilityCheck(const Position &senderPosition, const double &senderSpeed) const {
        if (senderSpeed < detectionParameters->maxOffroadSpeed ||
            getDistanceToNearestRoad(mGlobalEnvironmentModel, senderPosition) <
            detectionParameters->maxDistanceFromRoad) {
            return 1;
        } else {
            return 0;
        }
    }

    double LegacyChecks::SpeedPlausibilityCheck(const double &speed) const {
        if (fabs(speed) < detectionParameters->maxPlausibleSpeed) {
            return 1;
        } else {
            return 0;
        }
    }

    double LegacyChecks::ProximityPlausibilityCheck(const Position &senderPosition, const Position &receiverPosition,
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
    LegacyChecks::PositionConsistencyCheck(const Position &currentPosition, const Position &lastPosition,
                                           double time) const {
        if (distance(currentPosition, lastPosition).value() < detectionParameters->maxPlausibleSpeed * time) {
            return 1;
        } else {
            return 0;
        }
    }

    double
    LegacyChecks::SpeedConsistencyCheck(const double &currentSpeed, const double &oldSpeed, const double &time) const {
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

    double LegacyChecks::PositionSpeedConsistencyCheck(const Position &currentPosition, const Position &oldPosition,
                                                       const double &currentSpeed, const double &oldSpeed,
                                                       const double &deltaTime) const {
        if (deltaTime < detectionParameters->maxTimeDelta) {
            double deltaDistance = distance(currentPosition, oldPosition).value();
            double minimumSpeed = std::min(currentSpeed, oldSpeed);
            double minimumDistance =
                    oldSpeed * deltaTime - 0.5 * detectionParameters->maxPlausibleDeceleration * pow(deltaTime, 2);
            double maximumDistance =
                    oldSpeed * deltaTime + 0.5 * detectionParameters->maxPlausibleAcceleration * pow(deltaTime, 2);
            double addonMgtRange = detectionParameters->maxMgtRngDown + 0.3571 * minimumSpeed -
                                   0.01694 * pow(minimumSpeed, 2);
            addonMgtRange = (addonMgtRange < 0) ? 0 : addonMgtRange;

            if ((deltaDistance - minimumDistance + addonMgtRange) < 0 ||
                (maximumDistance - deltaDistance + detectionParameters->maxMgtRngUp) < 0) {
                return 0;
            }
        }
        return 1;
    }

    double
    LegacyChecks::PositionSpeedMaxConsistencyCheck(const Position &currentPosition, const Position &oldPosition,
                                                   const double &currentSpeed,
                                                   const double &oldSpeed, const double &deltaTime) const {
        if (deltaTime < detectionParameters->maxTimeDelta) {
            Position::value_type deltaDistance = distance(currentPosition, oldPosition);
            double theoreticalSpeed = deltaDistance.value() / deltaTime;
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

    double
    LegacyChecks::PositionHeadingConsistencyCheck(const double &currentHeading, const Position &currentPosition,
                                                  const Position &oldPosition,
                                                  const double &deltaTime, const double &currentSpeed) const {
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
//            drawTraciPoi(currentPosition, "current", libsumo::TraCIColor(207, 255, 0, 255), mSimulationBoundary,
//                         mTraciAPI);
//            drawTraciPoi(oldPosition, "old", libsumo::TraCIColor(255, 155, 155, 255), mSimulationBoundary, mTraciAPI);
            if (deltaHeading > detectionParameters->maxHeadingChange) {
                return 0;
            }
        }
        return 1;
    }

    double
    LegacyChecks::IntersectionCheck(const std::vector<Position> &receiverVehicleOutline,
                                    const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &relevantCams,
                                    const Position &senderPosition, const double &senderLength,
                                    const double &senderWidth, const double &senderHeading) {
        std::vector<Position> senderOutline = getVehicleOutline(senderPosition, Angle::from_degree(senderHeading),
                                                                senderLength, senderWidth);
//        drawTraciPolygon(receiverVehicleOutline,"receiver",libsumo::TraCIColor(255, 0, 255, 255),mSimulationBoundary,mTraciAPI);
//        drawTraciPolygon(senderOutline,"sender",libsumo::TraCIColor(207, 255, 0, 255),mSimulationBoundary,mTraciAPI);
        if (boost::geometry::intersects(senderOutline, receiverVehicleOutline)) {
            return 0;
        }
        for (const auto &currentCam: relevantCams) {

            auto camDeltaTime = ((double) (uint16_t) ((uint16_t) countTaiMilliseconds(mTimer->getTimeFor(simTime())) -
                                                      (*currentCam)->cam.generationDeltaTime)) / 1000;
            std::vector<Position> outline = getVehicleOutline((*currentCam), mSimulationBoundary, mTraciAPI);
            double iFactor = intersectionFactor(senderOutline, outline);
//            std::cout << "intersectionFactor: " << iFactor << std::endl;
            double factor = iFactor *
                            ((detectionParameters->maxIntersectionDeltaTime - camDeltaTime) /
                             detectionParameters->maxIntersectionDeltaTime);
//            if(factor > 0.5){
//                std::cout << "intersection failed" << std::endl;
//            }
            return factor > 0.5 ? 0 : 1;
        }
        return 1;

    }

    double LegacyChecks::SuddenAppearanceCheck(const Position &senderPosition, const Position &receiverPosition) const {
        if (distance(senderPosition, receiverPosition).value() > detectionParameters->maxSuddenAppearanceRange) {
            return 1;
        } else {
            return 0;
        }
    }

    std::shared_ptr<CheckResult> LegacyChecks::checkCAM(const VehicleDataProvider *receiverVDP,
                                                        const std::vector<Position> &receiverVehicleOutline,
                                                        const std::shared_ptr<vanetza::asn1::Cam> &currentCam,
                                                        const std::shared_ptr<vanetza::asn1::Cam> &lastCam,
                                                        const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &surroundingCamObjects) {
        const Position &receiverPosition = convertReferencePosition(receiverVDP->approximateReferencePosition(),
                                                                    mSimulationBoundary, mTraciAPI);

        Position currentCamPosition = convertReferencePosition(
                (*currentCam)->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
        PosConfidenceEllipse_t currentCamPositionConfidence =
                (*currentCam)->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse;

        BasicVehicleContainerHighFrequency_t currentHfc =
                (*currentCam)->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        double currentCamSpeed = (double) currentHfc.speed.speedValue / 100.0;
        double currentCamSpeedConfidence = (double) currentHfc.speed.speedConfidence / 100.0;
        double currentCamAcceleration =
                (double) currentHfc.longitudinalAcceleration.longitudinalAccelerationValue / 10.0;
        double currentCamHeading = (double) currentHfc.heading.headingValue / 10;
        double currentCamVehicleLength = (double) currentHfc.vehicleLength.vehicleLengthValue / 10;
        double currentCamVehicleWidth = (double) currentHfc.vehicleWidth / 10;
        Position currentCamSpeedVector = getVector(currentCamSpeed, currentCamHeading);
        Position currentCamAccelerationVector = getVector(currentCamAcceleration, currentCamHeading);

        std::shared_ptr<CheckResult> result = std::make_shared<CheckResult>();
        result->positionPlausibility = PositionPlausibilityCheck(currentCamPosition, currentCamSpeed);
        result->speedPlausibility = SpeedPlausibilityCheck(currentCamSpeed);
        result->proximityPlausibility = ProximityPlausibilityCheck(currentCamPosition, receiverPosition,
                                                                   surroundingCamObjects);
        result->rangePlausibility = RangePlausibilityCheck(currentCamPosition, receiverPosition);

        if (lastCam != nullptr) {
            auto camDeltaTime = (double) (uint16_t) ((*currentCam)->cam.generationDeltaTime -
                                                     (*lastCam)->cam.generationDeltaTime);
            result->consistencyIsChecked = true;

            result->positionConsistency = PositionConsistencyCheck(currentCamPosition, mLastCamPosition, camDeltaTime);
            result->speedConsistency = SpeedConsistencyCheck(currentCamSpeed, mLastCamSpeed, camDeltaTime);
            result->positionSpeedConsistency =
                    PositionSpeedConsistencyCheck(currentCamPosition, mLastCamPosition, currentCamSpeed, mLastCamSpeed,
                                                  camDeltaTime);
            result->positionSpeedMaxConsistency =
                    PositionSpeedMaxConsistencyCheck(currentCamPosition, mLastCamPosition, currentCamSpeed,
                                                     mLastCamSpeed,
                                                     camDeltaTime);
            result->positionHeadingConsistency =
                    PositionHeadingConsistencyCheck(currentCamHeading, currentCamPosition, mLastCamPosition,
                                                    camDeltaTime, currentCamSpeed);
            KalmanChecks(currentCamPosition, currentCamPositionConfidence, currentCamSpeed,
                         currentCamSpeedVector, currentCamSpeedConfidence, currentCamAcceleration,
                         currentCamAccelerationVector, currentCamHeading, mLastCamPosition,
                         mLastCamSpeedVector, camDeltaTime, result);
            result->frequency = BaseChecks::FrequencyCheck(camDeltaTime);
            result->intersection =
                    IntersectionCheck(receiverVehicleOutline, surroundingCamObjects, currentCamPosition,
                                      currentCamVehicleLength, currentCamVehicleWidth, currentCamHeading);
        } else {
            result->suddenAppearance = SuddenAppearanceCheck(currentCamPosition, receiverPosition);
        }
        mLastCamPosition = currentCamPosition;
        mLastCamSpeed = currentCamSpeed;
        mLastCamSpeedConfidence = currentCamSpeedConfidence;
        mLastCamSpeedVector = currentCamSpeedVector;
        return result;
    }

    std::bitset<16> LegacyChecks::checkSemanticLevel1Report(const vanetza::asn1::Cam &currentCam) {

        std::shared_ptr<CheckResult> result = std::make_shared<CheckResult>();

        BasicVehicleContainerHighFrequency_t currentHfc =
                currentCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        double currentCamSpeed = (double) currentHfc.speed.speedValue / 100.0;

        result->speedPlausibility = SpeedPlausibilityCheck(currentCamSpeed);
        return mThresholdFusion->checkForReport(*result)[detectionLevels::Level1];
    }

    std::bitset<16> LegacyChecks::checkSemanticLevel2Report(const vanetza::asn1::Cam &currentCam,
                                                            const vanetza::asn1::Cam &lastCam) {
        std::shared_ptr<CheckResult> result = std::make_shared<CheckResult>();
//        initializeKalmanFilters(lastCam);

        Position currentCamPosition = convertReferencePosition(
                currentCam->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
        PosConfidenceEllipse_t currentCamPositionConfidence =
                currentCam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse;

        BasicVehicleContainerHighFrequency_t currentCamHfc =
                currentCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        double currentCamSpeed = (double) currentCamHfc.speed.speedValue / 100.0;
        double currentCamSpeedConfidence = (double) currentCamHfc.speed.speedConfidence / 100.0;
        double currentCamAcceleration =
                (double) currentCamHfc.longitudinalAcceleration.longitudinalAccelerationValue / 10.0;
        double currentCamHeading = (double) currentCamHfc.heading.headingValue / 10;
        Position currentCamSpeedVector = getVector(currentCamSpeed, currentCamHeading);
        Position currentCamAccelerationVector = getVector(currentCamAcceleration, currentCamHeading);

        Position lastCamPosition = convertReferencePosition(
                lastCam->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);

        BasicVehicleContainerHighFrequency_t lastCamHfc =
                lastCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        double lastCamSpeed = (double) lastCamHfc.speed.speedValue / 100.0;
        double lastCamAcceleration =
                (double) lastCamHfc.longitudinalAcceleration.longitudinalAccelerationValue / 10.0;
        double lastCamHeading = (double) lastCamHfc.heading.headingValue / 10;
        Position lastCamSpeedVector = getVector(lastCamSpeed, lastCamHeading);
        Position lastCamAccelerationVector = getVector(lastCamAcceleration, currentCamHeading);

        auto camDeltaTime = (double) (uint16_t) (currentCam->cam.generationDeltaTime -
                                                 lastCam->cam.generationDeltaTime);
        result->consistencyIsChecked = true;

        result->positionConsistency = PositionConsistencyCheck(currentCamPosition, lastCamPosition, camDeltaTime);
        result->speedConsistency = SpeedConsistencyCheck(currentCamSpeed, lastCamSpeed, camDeltaTime);
        result->positionSpeedConsistency =
                PositionSpeedConsistencyCheck(currentCamPosition, lastCamPosition, currentCamSpeed, lastCamSpeed,
                                              camDeltaTime);
        result->positionSpeedMaxConsistency =
                PositionSpeedMaxConsistencyCheck(currentCamPosition, lastCamPosition, currentCamSpeed, lastCamSpeed,
                                                 camDeltaTime);
        result->positionHeadingConsistency =
                PositionHeadingConsistencyCheck(currentCamHeading, currentCamPosition, lastCamPosition,
                                                camDeltaTime, currentCamSpeed);
        KalmanChecks(currentCamPosition, currentCamPositionConfidence, currentCamSpeed,
                     currentCamSpeedVector, currentCamSpeedConfidence, currentCamAcceleration,
                     currentCamAccelerationVector, currentCamHeading, lastCamPosition,
                     lastCamSpeedVector, camDeltaTime, result);
        result->frequency = BaseChecks::FrequencyCheck(camDeltaTime);
        return mThresholdFusion->checkForReport(*result)[detectionLevels::Level2];
    }

    std::bitset<16> LegacyChecks::checkSemanticLevel3Report(const vanetza::asn1::Cam &currentCam,
                                                            const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &neighbourCams) {
        std::shared_ptr<CheckResult> result = std::make_shared<CheckResult>();

        Position currentCamPosition = convertReferencePosition(
                currentCam->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
        BasicVehicleContainerHighFrequency_t currentHfc =
                currentCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        double currentCamSpeed = (double) currentHfc.speed.speedValue / 100.0;
        double currentCamHeading = (double) currentHfc.heading.headingValue / 10;
        double currentCamVehicleLength = (double) currentHfc.vehicleLength.vehicleLengthValue / 10;
        double currentCamVehicleWidth = (double) currentHfc.vehicleWidth / 10;


        std::vector<Position> currentCamOutline = getVehicleOutline(currentCamPosition,
                                                                    Angle::from_degree(currentCamHeading),
                                                                    currentCamVehicleLength, currentCamVehicleWidth);
        result->positionPlausibility = PositionPlausibilityCheck(currentCamPosition, currentCamSpeed);
        result->intersection =
                IntersectionCheck(currentCamOutline, neighbourCams, currentCamPosition,
                                  currentCamVehicleLength, currentCamVehicleWidth, currentCamHeading);
        return mThresholdFusion->checkForReport(*result)[detectionLevels::Level3];
    }

    std::bitset<16>
    LegacyChecks::checkSemanticLevel4Report(const vanetza::asn1::Cam &currentCam, const Position &receiverPosition,
                                            const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &neighbourCams) {
        std::shared_ptr<CheckResult> result = std::make_shared<CheckResult>();
        Position currentCamPosition = convertReferencePosition(
                currentCam->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);

        result->proximityPlausibility = ProximityPlausibilityCheck(currentCamPosition, receiverPosition,
                                                                   neighbourCams);
        result->rangePlausibility = RangePlausibilityCheck(currentCamPosition, receiverPosition);

        return mThresholdFusion->checkForReport(*result)[detectionLevels::Level4];
    }

    LegacyChecks::LegacyChecks(shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                               DetectionParameters *detectionParameters, const Timer *timer,
                               const vanetza::asn1::Cam &message)
            : BaseChecks(std::move(traciAPI), globalEnvironmentModel, detectionParameters, timer, message) {
    }

    LegacyChecks::LegacyChecks(shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                               DetectionParameters *detectionParameters, double misbehaviorThreshold,
                               const Timer *timer)
            : BaseChecks(std::move(traciAPI), globalEnvironmentModel, detectionParameters, timer) {
        mThresholdFusion = new ThresholdFusion(misbehaviorThreshold);
    }
} // namespace artery