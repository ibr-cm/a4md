//
// Created by bastian on 15.06.21.
//

#ifndef ARTERY_DETECTEDSENDER_H
#define ARTERY_DETECTEDSENDER_H


#include <artery/application/md/checks/BaseChecks.h>
#include <list>
#include <vanetza/asn1/cam.hpp>
#include <artery/application/md/checks/kalman/Kalman_SVI.h>
#include <artery/application/md/checks/kalman/Kalman_SC.h>
#include <artery/application/md/checks/kalman/Kalman_SI.h>
#include <artery/application/md/checks/CheckResult.h>
#include <artery/traci/VehicleController.h>
#include <artery/application/md/util/F2MDParameters.h>

namespace artery {

    class DetectedSender {
    public:
        DetectedSender(const std::shared_ptr<const traci::API> &traciAPI,
                       GlobalEnvironmentModel *globalEnvironmentModel, DetectionParameters *detectionParameters,
                       const vanetza::asn1::Cam &message);

        CheckResult *addAndCheckCam(const vanetza::asn1::Cam &message, const VehicleDataProvider *receiverVDP,
                                    const std::vector<Position> &receiverVehicleOutline,
                                    const std::vector<vanetza::asn1::Cam *> &relevantCams);

        StationID_t getStationId() const { return mStationId; };

        std::vector<CheckResult *> getResults() { return checkResults; };

    private:
        BaseChecks *baseChecks;

        std::vector<CheckResult *> checkResults;

        Position mPosition;
        StationID_t mStationId;
    };

} //namespace artery

#endif //ARTERY_DETECTEDSENDER_H
