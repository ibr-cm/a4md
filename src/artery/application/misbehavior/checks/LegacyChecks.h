#ifndef ARTERY_LEGACYCHECK_H_
#define ARTERY_LEGACYCHECK_H_

#include <artery/application/misbehavior/checks/BaseChecks.h>
#include <artery/application/misbehavior/checks/CheckResult.h>

namespace artery {

    class LegacyChecks : public BaseChecks {
    public:
        LegacyChecks() = delete;

        LegacyChecks(shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                     DetectionParameters *detectionParameters, const Timer *timer, const vanetza::asn1::Cam &message);

        LegacyChecks(shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                     DetectionParameters *detectionParameters,double misbehaviorThreshold, const Timer *timer);

        CheckResult *checkCAM(const VehicleDataProvider *receiverVDP,
                              const std::vector<Position> &receiverVehicleOutline,
                              const vanetza::asn1::Cam &currentCam, const vanetza::asn1::Cam *lastCamPtr,
                              const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &surroundingCamObjects) override;

        std::bitset<16> checkSemanticLevel1Report(const vanetza::asn1::Cam &currentCam) override;

        std::bitset<16> checkSemanticLevel2Report(const vanetza::asn1::Cam &currentCam,
                                                  const vanetza::asn1::Cam &lastCam) override;

        std::bitset<16> checkSemanticLevel3Report(const vanetza::asn1::Cam &currentCam,
                                                  const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &neighbourCams) override;
        std::bitset<16>
        checkSemanticLevel4Report(const vanetza::asn1::Cam &currentCam, const Position &receiverPosition,
                                  const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &neighbourCams) override;

    protected:


        double PositionPlausibilityCheck(const Position &senderPosition, const double &senderSpeed) const;

        double SpeedPlausibilityCheck(const double &speed) const;

        double ProximityPlausibilityCheck(const Position &senderPosition, const Position &receiverPosition,
                                          const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &surroundingCamObjects);

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
                                 const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &relevantCams,
                                 const Position &senderPosition, const double &senderLength,
                                 const double &senderWidth, const double &senderHeading);

        double SuddenAppearanceCheck(const Position &senderPosition, const Position &receiverPosition) const;


    };
} // namespace artery
#endif