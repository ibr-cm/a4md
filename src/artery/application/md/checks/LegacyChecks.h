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

        CheckResult *
        checkCAM(const VehicleDataProvider *receiverVDP, const std::vector<Position> &receiverVehicleOutline,
                 const vanetza::asn1::Cam &currentCam, const vanetza::asn1::Cam *lastCamPtr,
                 const std::vector<vanetza::asn1::Cam *> &surroundingCamObjects);

    private:
        double ProximityPlausibilityCheck(const Position &senderPosition, const Position &receiverPosition,
                                          const vector<vanetza::asn1::Cam *> &surroundingCamObjects);

        double RangePlausibilityCheck(const Position &senderPosition, const Position &receiverPosition) const;

        double PositionConsistencyCheck(Position &currentPosition, const Position &lastPosition, double time) const;

        double SpeedConsistencyCheck(double currentSpeed, double oldSpeed, double time) const;

        double PositionSpeedConsistencyCheck(Position &currentPosition, Position &oldPosition, double currentSpeed,
                                             double oldSpeed, double deltaTime) const;

        double PositionSpeedMaxConsistencyCheck(Position &currentPosition, Position &oldPosition,
                                                double currentSpeed, double oldSpeed, double deltaTime) const;

        double SpeedPlausibilityCheck(double speed) const;

        double IntersectionCheck(const std::vector<Position> &receiverVehicleOutline,
                                 const std::vector<vanetza::asn1::Cam *> &relevantCams,
                                 const Position &senderPosition, const double &senderLength,
                                 const double &senderWidth, const double &senderHeading, const double &deltaTime);

        double SuddenAppearanceCheck(Position &senderPosition, const Position &receiverPosition) const;

        double PositionPlausibilityCheck(Position &senderPosition, double senderSpeed) const;

//        double FrequencyCheck(long newTime, long oldTime) const;

        double
        PositionHeadingConsistencyCheck(const double &currentHeading, Position &currentPosition,
                                        Position &oldPosition,
                                        double deltaTime, double currentSpeed) const;

    };
} // namespace artery
#endif