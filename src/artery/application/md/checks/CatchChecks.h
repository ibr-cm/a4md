//
// Created by bastian on 12.07.21.
//

#ifndef ARTERY_CATCHCHECKS_H
#define ARTERY_CATCHCHECKS_H

#include <artery/application/md/checks/BaseChecks.h>
#include <artery/application/md/checks/CheckResult.h>

namespace artery {


    class CatchChecks : public BaseChecks {
    public:
        CatchChecks() = delete;

        CatchChecks(std::shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                    DetectionParameters *detectionParameters,
                    Kalman_SVI *kalmanSVI, Kalman_SC *kalmanSVSI,
                    Kalman_SI *kalmanSI, Kalman_SI *kalmanVI);

        CheckResult *
        checkCAM(const VehicleDataProvider *receiverVDP, const std::vector<Position> &receiverVehicleOutline,
                 TrackedObjectsFilterRange &envModObjects,
                 const vanetza::asn1::Cam &currentCam, const vanetza::asn1::Cam *lastCamPtr);

    private:
        double ProximityPlausibilityCheck(Position &testPosition, const Position &myPosition,
                                          TrackedObjectsFilterRange &envModObjects);

        double
        RangePlausibilityCheck(const Position &senderPosition, const PosConfidenceEllipse_t &senderConfidenceEllipse,
                               const Position &receiverPosition,
                               const PosConfidenceEllipse_t &receiverConfidenceEllipse);

        double PositionConsistencyCheck(const Position &currentPosition,
                                        const PosConfidenceEllipse_t &currentConfidenceEllipse,
                                        const Position &oldPosition,
                                        const PosConfidenceEllipse_t &oldConfidenceEllipse,
                                        double deltaTime);


        double
        SpeedConsistencyCheck(const double &currentSpeed, const double &currentSpeedConfidence, const double &oldSpeed,
                              const double &oldSpeedConfidence, const double &deltaTime);

        double SpeedPlausibilityCheck(const double &currentSpeed, const double &currentSpeedConfidence);
    };
} // namespace artery

#endif //ARTERY_CATCHCHECKS_H
