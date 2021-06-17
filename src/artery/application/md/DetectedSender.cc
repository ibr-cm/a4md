//
// Created by bastian on 15.06.21.
//

#include "DetectedSender.h"

namespace artery {
    DetectedSender::DetectedSender(const std::shared_ptr<const traci::API> &traciAPI,
                                   DetectionParameters *detectionParameters,
                                   const vanetza::asn1::Cam &message) : legacyChecks(traciAPI, detectionParameters,
                                                                                     &kalmanSVI, &kalmanSVSI, &kalmanSI,
                                                                                     &kalmanVI) {
        Position position = legacyChecks.convertCamPosition(message->cam.camParameters.basicContainer.referencePosition);
        double speed =
                (double) message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue /
                100.0;
        double heading =
                (double) message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue /
                10.0;
        Position speedVector = LegacyChecks::getVector(speed,heading);
        kalmanSVI.setInitial(position.x.value(), position.y.value(), speedVector.x.value(), speedVector.y.value());
        kalmanSVSI.setInitial(0, speed);
        kalmanSI.setInitial(position.x.value(), position.y.value());
        kalmanVI.setInitial(speedVector.x.value(), speedVector.y.value());
    }


    CheckResult *
    DetectedSender::addAndCheckCam(const vanetza::asn1::Cam &message, const Position &receiverPosition, TrackedObjectsFilterRange &envModObjects) {
        CheckResult *result;
        if (!checkResults.empty()) {
            result = legacyChecks.checkCAM(receiverPosition, envModObjects, message, &checkResults.back()->cam);
        } else {
            result = legacyChecks.checkCAM(receiverPosition, envModObjects, message, nullptr);
        }
        result->cam = message;
        checkResults.push_back(result);
        return result;
    }
} //namespace artery
