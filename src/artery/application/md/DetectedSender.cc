//
// Created by bastian on 15.06.21.
//

#include "DetectedSender.h"

namespace artery {
    DetectedSender::DetectedSender(const std::shared_ptr<const traci::API> &traciAPI,
                                   GlobalEnvironmentModel *globalEnvironmentModel,
                                   DetectionParameters *detectionParameters,
                                   const vanetza::asn1::Cam &message)
            : legacyChecks(traciAPI, globalEnvironmentModel, detectionParameters,
                           &kalmanSVI, &kalmanSVSI, &kalmanSI,
                           &kalmanVI) {
        mStationId = message->header.stationID;
        Position position = LegacyChecks::convertCamPosition(
                message->cam.camParameters.basicContainer.referencePosition,
                traci::Boundary{traciAPI->simulation.getNetBoundary()}, traciAPI);
        double speed =
                (double) message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue /
                100.0;
        double heading =
                (double) message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue /
                10.0;
        Position speedVector = LegacyChecks::getVector(speed, heading);
        kalmanSVI.setInitial(position.x.value(), position.y.value(), speedVector.x.value(), speedVector.y.value());
        kalmanSVSI.setInitial(0, speed);
        kalmanSI.setInitial(position.x.value(), position.y.value());
        kalmanVI.setInitial(speedVector.x.value(), speedVector.y.value());
    }


    CheckResult *
    DetectedSender::addAndCheckCam(const vanetza::asn1::Cam &message, const VehicleDataProvider *receiverVDP,
                                   const std::vector<Position> &receiverVehicleOutline,
                                   TrackedObjectsFilterRange &envModObjects) {
        CheckResult *result;
        if (!checkResults.empty()) {
            result = legacyChecks.checkCAM(receiverVDP, receiverVehicleOutline, envModObjects, message,
                                           &checkResults.back()->cam);
        } else {
            result = legacyChecks.checkCAM(receiverVDP, receiverVehicleOutline, envModObjects, message, nullptr);
        }
        result->cam = message;
        checkResults.push_back(result);
        return result;
    }
} //namespace artery
