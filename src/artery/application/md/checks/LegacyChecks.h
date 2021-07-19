#ifndef ARTERY_LEGACYCHECK_H_
#define ARTERY_LEGACYCHECK_H_

#include <artery/application/md/checks/BaseChecks.h>
#include <artery/application/md/checks/CheckResult.h>

namespace artery {

    class LegacyChecks : public BaseChecks {
    public:
        LegacyChecks() = delete;

        LegacyChecks(shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                     DetectionParameters *detectionParameters, const vanetza::asn1::Cam &message);

        CheckResult *checkCAM(const VehicleDataProvider *receiverVDP,
                              const std::vector<Position> &receiverVehicleOutline,
                              const vanetza::asn1::Cam &currentCam, const vanetza::asn1::Cam *lastCamPtr,
                              const std::vector<vanetza::asn1::Cam *> &surroundingCamObjects) override;

    private:
        double PositionPlausibilityCheck(const Position &senderPosition, const double &senderSpeed) const;

        double SpeedPlausibilityCheck(const double &speed) const;

        double ProximityPlausibilityCheck(const Position &senderPosition, const Position &receiverPosition,
                                          const vector<vanetza::asn1::Cam *> &surroundingCamObjects);

        double RangePlausibilityCheck(const Position &senderPosition, const Position &receiverPosition) const;

        double
        PositionConsistencyCheck(const Position &currentPosition, const Position &lastPosition, double time) const;

        double SpeedConsistencyCheck(const double &currentSpeed, const double &oldSpeed, const double &time) const;

        double PositionSpeedConsistencyCheck(const Position &currentPosition, const Position &oldPosition,
                                             const double &currentSpeed,
                                             const double &oldSpeed, const double &deltaTime) const;

        double PositionSpeedMaxConsistencyCheck(const Position &currentPosition, const Position &oldPosition,
                                                const double &currentSpeed, const double &oldSpeed,
                                                const double &deltaTime) const;

        double PositionHeadingConsistencyCheck(const double &currentHeading, const Position &currentPosition,
                                               const Position &oldPosition, const double &deltaTime,
                                               const double &currentSpeed) const;

        double IntersectionCheck(const std::vector<Position> &receiverVehicleOutline,
                                 const std::vector<vanetza::asn1::Cam *> &relevantCams,
                                 const Position &senderPosition, const double &senderLength,
                                 const double &senderWidth, const double &senderHeading, const double &deltaTime);

        double SuddenAppearanceCheck(const Position &senderPosition, const Position &receiverPosition) const;


    };
} // namespace artery
#endif