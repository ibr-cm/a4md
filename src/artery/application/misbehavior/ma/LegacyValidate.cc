//
// Created by bastian on 06.08.21.
//

#include "LegacyValidate.h"
#include "artery/application/misbehavior/util/HelperFunctions.h"
#include "artery/application/misbehavior/util/DetectionLevels.h"

namespace artery {

    LegacyValidate::LegacyValidate(shared_ptr<const traci::API> traciAPI,
                                   artery::GlobalEnvironmentModel *globalEnvironmentModel,
                                   artery::DetectionParameters *detectionParameters)
            : LegacyChecks(std::move(traciAPI), globalEnvironmentModel, detectionParameters) {
        mThresholdFusion = new ThresholdFusion(detectionParameters->misbehaviorThreshold);
    }

    std::bitset<16> LegacyValidate::checkSemanticLevel2Report(const vanetza::asn1::Cam &currentCam,
                                                              const vanetza::asn1::Cam &lastCam) {
        auto *result = new CheckResult;
        initializeKalmanFilters(lastCam);

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
        double currentCamVehicleLength = (double) currentCamHfc.vehicleLength.vehicleLengthValue / 10;
        double currentCamVehicleWidth = (double) currentCamHfc.vehicleWidth / 10;
        Position currentCamSpeedVector = getVector(currentCamSpeed, currentCamHeading);
        Position currentCamAccelerationVector = getVector(currentCamAcceleration, currentCamHeading);

        Position lastCamPosition = convertReferencePosition(
                lastCam->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
        PosConfidenceEllipse_t lastCamPositionConfidence =
                lastCam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse;

        BasicVehicleContainerHighFrequency_t lastCamHfc =
                lastCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        double lastCamSpeed = (double) lastCamHfc.speed.speedValue / 100.0;
        double lastCamSpeedConfidence = (double) lastCamHfc.speed.speedConfidence / 100.0;
        double lastCamAcceleration =
                (double) lastCamHfc.longitudinalAcceleration.longitudinalAccelerationValue / 10.0;
        double lastCamHeading = (double) lastCamHfc.heading.headingValue / 10;
        double lastCamVehicleLength = (double) lastCamHfc.vehicleLength.vehicleLengthValue / 10;
        double lastCamVehicleWidth = (double) lastCamHfc.vehicleWidth / 10;
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

} // namespace artery
