//
// Created by bastian on 15.06.21.
//

#include "DetectedSender.h"

#include <utility>

DetectedSender::DetectedSender(const vanetza::asn1::Cam& message, traci::VehicleController *vehicleController) {

    mVehicleController = vehicleController;
    traci::TraCIGeoPosition traciGeoPosition = {
            (double) message->cam.camParameters.basicContainer.referencePosition.longitude / 10000000.0,
            (double) message->cam.camParameters.basicContainer.referencePosition.latitude / 10000000.0};
    traci::TraCIPosition traciPosition = mVehicleController->getTraCI()->convert2D(traciGeoPosition);
    float speed =
            (float) message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue /
            100.0f;
    float heading =
            (float) message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue /
            10.0f;

    kalmanSVI.setInitial((float) traciPosition.x, (float) traciPosition.y, speed, heading);
    kalmanSVSI.setInitial(0, speed);
    kalmanSI.setInitial((float) traciPosition.x, (float) traciPosition.y);
    kalmanSAI.setInitial((float) traciPosition.x, (float) traciPosition.y);
    kalmanVI.setInitial(speed,heading);

    stationId = message->header.stationID;
//    CheckResult result;
//    result.cam = std::move(message);
//    checkResults.emplace_back(result);
    receivedCams.emplace_back(message);
}

CheckResult DetectedSender::addAndCheckCam(const vanetza::asn1::Cam& message) {
    receivedCams.emplace_back(message);

    return CheckResult();
}
