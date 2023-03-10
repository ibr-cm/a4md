//
// Created by bastian on 12.07.21.
//

#ifndef ARTERY_CATCHCHECKS_H
#define ARTERY_CATCHCHECKS_H

#include <artery/application/misbehavior/checks/BaseChecks.h>

namespace artery {

    class CatchChecks : public BaseChecks {
    public:
        CatchChecks() = delete;

        ~CatchChecks() override {
            delete kalmanSVI;
            delete kalmanSVSI;
            delete kalmanSI;
            delete kalmanVI;
            delete mThresholdFusion;
        }

        CatchChecks(std::shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                    DetectionParameters *detectionParameters, const Timer *timer,
                    const std::map<detectionLevels::DetectionLevels, bool> &checkableDetectionLevels,
                    const std::shared_ptr<vanetza::asn1::Cam> &message);

        CatchChecks(std::shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                    DetectionParameters *detectionParameters, double misbehaviorThreshold, const Timer *timer);

        std::shared_ptr<CheckResult> checkCAM(const VehicleDataProvider *receiverVDP,
                                              const std::vector<Position> &receiverVehicleOutline,
                                              const std::shared_ptr<vanetza::asn1::Cam> &currentCam,
                                              const std::shared_ptr<vanetza::asn1::Cam> &lastCam,
                                              const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &surroundingCamObjects) override;

        std::bitset<16> checkSemanticLevel1Report(const std::shared_ptr<vanetza::asn1::Cam> &currentCam) override;

        std::bitset<16> checkSemanticLevel2Report(const std::shared_ptr<vanetza::asn1::Cam> &currentCam,
                                                  const std::shared_ptr<vanetza::asn1::Cam> &lastCam) override;

        std::bitset<16> checkSemanticLevel3Report(const std::shared_ptr<vanetza::asn1::Cam> &currentCam,
                                                  const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &neighbourCams) override;

        std::bitset<16>
        checkSemanticLevel4Report(const std::shared_ptr<vanetza::asn1::Cam> &currentCam,
                                  const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &neighbourCams,
                                  const SenderInfoContainer_t &senderInfo) override;

    protected:
        double PositionPlausibilityCheck(const Position &senderPosition,
                                         const PosConfidenceEllipse_t &senderPositionConfidence,
                                         const double &senderSpeed, const double &senderSpeedConfidence);

        double SpeedPlausibilityCheck(const double &currentSpeed, const double &currentSpeedConfidence);

        double ProximityPlausibilityCheck(const Position &senderPosition, const Position &receiverPosition,
                                          const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &surroundingCamObjects,
                                          const StationID_t &senderStationId);

        double RangePlausibilityCheck(const Position &senderPosition, const std::vector<Position> &senderEllipse,
                                      const double &senderEllipseRadius,
                                      const Position &receiverPosition, const std::vector<Position> &receiverEllipse,
                                      const double &receiverEllipseRadius);

        double PositionConsistencyCheck(const Position &currentPosition, const std::vector<Position> &currentEllipse,
                                        const double &currentEllipseRadius,
                                        const Position &oldPosition, const std::vector<Position> &oldEllipse,
                                        const double &oldEllipseRadius,
                                        const double &deltaTime);

        double SpeedConsistencyCheck(const double &currentSpeed, const double &currentSpeedConfidence,
                                     const double &oldSpeed, const double &oldSpeedConfidence, const double &deltaTime);

        double PositionSpeedConsistencyCheck(const Position &currentPosition,
                                             const std::vector<Position> &currentEllipse,
                                             const double &currentEllipseRadius,
                                             const Position &oldPosition,
                                             const std::vector<Position> &oldEllipse,
                                             const double &oldEllipseRadius,
                                             const double &currentSpeed, const double &currentSpeedConfidence,
                                             const double &oldSpeed, const double &oldSpeedConfidence,
                                             const double &deltaTime);

        double PositionSpeedMaxConsistencyCheck(const Position &currentPosition,
                                                const PosConfidenceEllipse_t &currentPositionConfidence,
                                                const Position &oldPosition,
                                                const PosConfidenceEllipse_t &oldConfidenceEllipse,
                                                const double &currentSpeed, const double &currentSpeedConfidence,
                                                const double &oldSpeed, const double &oldSpeedConfidence,
                                                const double &deltaTime);

        double PositionHeadingConsistencyCheck(const double &currentHeading, const double &currentHeadingConfidence,
                                               const Position &currentPosition,
                                               const PosConfidenceEllipse_t &currentPositionConfidence,
                                               const Position &oldPosition,
                                               const PosConfidenceEllipse_t &oldPositionConfidence,
                                               const double &currentSpeed, const double &currentSpeedConfidence,
                                               const double &deltaTime);

        double IntersectionCheck(const std::vector<Position> &receiverEllipse,
                                 const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &relevantCams,
                                 const std::vector<Position> &senderEllipse, const double &deltaTime);

        double SuddenAppearanceCheck(const Position &senderPosition,
                                     const PosConfidenceEllipse_t &senderConfidenceEllipse,
                                     const Position &receiverPosition,
                                     const PosConfidenceEllipse_t &receiverConfidenceEllipse);

    };
} // namespace artery

#endif //ARTERY_CATCHCHECKS_H
