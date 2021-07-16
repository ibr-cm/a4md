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

        CatchChecks(shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                    DetectionParameters *detectionParameters, const vanetza::asn1::Cam &message);

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

        double PositionHeadingConsistencyCheckOld(const double &currentHeading,
                                                               const double &currentHeadingConfidence,
                                                               const Position &currentPosition,
                                                               const PosConfidenceEllipse_t &currentPositionConfidence,
                                                               const Position &oldPosition,
                                                               const PosConfidenceEllipse_t &oldPositionConfidence,
                                                               const double &currentSpeed,
                                                               const double &currentSpeedConfidence,
                                                               const double &deltaTime);

        double PositionPlausibilityCheck(const Position &senderPosition,
                                         const PosConfidenceEllipse_t &senderPositionConfidence,
                                         const double &senderSpeed, const double &senderSpeedConfidence);

        double
        SuddenAppearanceCheck(const Position &senderPosition, const PosConfidenceEllipse_t &senderConfidenceEllipse,
                              const Position &receiverPosition,
                              const PosConfidenceEllipse_t &receiverConfidenceEllipse);

        double
        IntersectionCheck(const std::vector<Position> &receiverVehicleOutline,
                          const std::vector<vanetza::asn1::Cam *> &relevantCams,
                          const Position &senderPosition, const double &senderLength,
                          const double &senderWidth, const double &senderHeading, const double &deltaTime);

        double
        PositionSpeedMaxConsistencyCheck(const Position &currentPosition,
                                         const PosConfidenceEllipse_t &currentPositionConfidence,
                                         const Position &oldPosition,
                                         const PosConfidenceEllipse_t &oldConfidenceEllipse,
                                         double currentSpeed, double currentSpeedConfidence, double oldSpeed,
                                         double oldSpeedConfidence, double deltaTime);
    };
} // namespace artery

#endif //ARTERY_CATCHCHECKS_H
