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
#include <artery/application/md/F2MDParameters.h>
#include <artery/application/md/checks/LegacyChecks.h>

namespace artery {

    class DetectedSender {
    public:
        DetectedSender(const std::shared_ptr<const traci::API> &traciAPI,
                       GlobalEnvironmentModel *globalEnvironmentModel, DetectionParameters *detectionParameters,
                       const vanetza::asn1::Cam &message);

        CheckResult *addAndCheckCam(const vanetza::asn1::Cam &message, const Position &receiverPosition,
                                    TrackedObjectsFilterRange &envModObjects);

    private:
        LegacyChecks legacyChecks;

        std::list<CheckResult *> checkResults;
        Position mPosition;

        Kalman_SVI kalmanSVI;
        Kalman_SC kalmanSVSI;
        Kalman_SI kalmanSI;
        Kalman_SI kalmanVI;

    };

} //namespace artery

#endif //ARTERY_DETECTEDSENDER_H
