//
// Created by bastian on 15.06.21.
//

#ifndef ARTERY_DETECTEDSENDER_H
#define ARTERY_DETECTEDSENDER_H


#include <list>
#include <vanetza/asn1/cam.hpp>
#include <artery/application/md/kalman/Kalman_SVI.h>
#include <artery/application/md/kalman/Kalman_SC.h>
#include <artery/application/md/kalman/Kalman_SI.h>
#include <artery/application/md/checks/CheckResult.h>
#include <artery/traci/VehicleController.h>

class DetectedSender {
public:
    DetectedSender(const vanetza::asn1::Cam& message, traci::VehicleController *vehicleController);
    CheckResult addAndCheckCam(const vanetza::asn1::Cam& message);

private:
    const traci::VehicleController *mVehicleController = nullptr;


    std::list<vanetza::asn1::Cam> receivedCams;
//    std::list<CheckResult> checkResults;
    unsigned long stationId;

    Kalman_SVI kalmanSVI;
    Kalman_SC kalmanSVSI;
    Kalman_SI kalmanSI;
    Kalman_SI kalmanVI;
    Kalman_SI kalmanSAI;

};


#endif //ARTERY_DETECTEDSENDER_H
