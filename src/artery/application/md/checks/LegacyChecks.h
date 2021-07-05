#ifndef ARTERY_LEGACYCHECK_H_
#define ARTERY_LEGACYCHECK_H_

#include "CheckResult.h"
#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include <artery/application/md/kalman/Kalman_SVI.h>
#include <artery/application/md/kalman/Kalman_SC.h>
#include <artery/application/md/kalman/Kalman_SI.h>
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/GlobalEnvironmentModel.h>
#include <artery/application/VehicleDataProvider.h>
#include <artery/application/md/util/F2MDParameters.h>
#include "artery/traci/VehicleController.h"
#include "artery/utility/Geometry.h"

namespace artery {

    class LegacyChecks {
    public:
        LegacyChecks() = delete;

        LegacyChecks(std::shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                     DetectionParameters *detectionParameters,
                     Kalman_SVI *kalmanSVI, Kalman_SC *kalmanSVSI,
                     Kalman_SI *kalmanSI, Kalman_SI *kalmanVI);

        CheckResult *checkCAM(const VehicleDataProvider *receiverVDP, const std::vector<Position>& receiverVehicleOutline, TrackedObjectsFilterRange &envModObjects,
                              const vanetza::asn1::Cam &currentCam, const vanetza::asn1::Cam *lastCamPtr);

        static Position convertCamPosition(const ReferencePosition_t &referencePosition, traci::Boundary simulationBoundary, std::shared_ptr<const traci::API> traciAPI);

        static Position getVector(const double &value, const double &angle);

        static double calculateHeadingAngle(const Position &position);
        static double getDistanceToNearestRoad(GlobalEnvironmentModel *globalEnvMod, const Position &position);
    private:

        static bool staticInitializationComplete;
        static GlobalEnvironmentModel *mGlobalEnvironmentModel;
        static traci::Boundary mSimulationBoundary;
        static std::shared_ptr<const traci::API> mTraciAPI;
        DetectionParameters *detectionParameters;

        Kalman_SVI *kalmanSVI;
        Kalman_SC *kalmanSVSI;
        Kalman_SI *kalmanSI;
        Kalman_SI *kalmanVI;

        double ProximityPlausibilityCheck(Position &testPosition, const Position &myPosition,
                                          TrackedObjectsFilterRange &envModObjects);

        double RangePlausibilityCheck(const Position &senderPosition, const Position &receiverPosition) const;

        double PositionConsistencyCheck(Position &senderPosition, const Position &receiverPosition, double time) const;

        double SpeedConsistencyCheck(double currentSpeed, double oldSpeed, double time) const;

        double PositionSpeedConsistencyCheck(Position &currentPosition, Position &oldPosition, double currentSpeed,
                                             double oldSpeed, double deltaTime) const;

        double PositionSpeedMaxConsistencyCheck(Position &currentPosition, Position &oldPosition,
                                                double currentSpeed, double oldSpeed, double deltaTime) const;

        double SpeedPlausibilityCheck(double speed) const;

        double IntersectionCheck(const std::vector<Position>& receiverVehicleOutline, const StationID_t &senderStationId,
                                 TrackedObjectsFilterRange &envModObjects);

        double SuddenAppearanceCheck(Position &senderPosition, const Position &receiverPosition) const;

        double PositionPlausibilityCheck(Position &senderPosition, double senderSpeed) const;

        double FrequencyCheck(long newTime, long oldTime) const;


        double
        PositionHeadingConsistencyCheck(const HeadingValue_t &currentHeading, Position &currentPosition,
                                        Position &oldPosition,
                                        double deltaTime, double currentSpeed) const;

        void KalmanPositionSpeedConsistencyCheck(Position &currentPosition,
                                                 const PosConfidenceEllipse_t &currentPositionConfidence,
                                                 const Position &currentSpeed, const Position &currentAcceleration,
                                                 double currentSpeedConfidence,
                                                 double &deltaTime, double *returnValue) const;

        void KalmanPositionSpeedScalarConsistencyCheck(Position &currentPosition, Position &oldPosition,
                                                       const PosConfidenceEllipse_t &currentPositionConfidence,
                                                       double &currentSpeed, double &currentAcceleration,
                                                       double &currentSpeedConfidence, double &deltaTime,
                                                       double *returnValue);

        double KalmanPositionConsistencyCheck(Position &currentPosition, Position &oldPosition,
                                              const PosConfidenceEllipse_t &currentPositionConfidence,
                                              double &deltaTime);

        double
        KalmanPositionAccConsistencyCheck(const Position &currentPosition, const Position &currentSpeed,
                                          const PosConfidenceEllipse_t &currentPositionConfidence,
                                          double &deltaTime);

        double KalmanSpeedConsistencyCheck(const Position &currentSpeed, const Position &oldSpeed,
                                           double &currentSpeedConfidence, const Position &currentAcceleration,
                                           double &deltaTime);

//        InterTest MultipleIntersectionCheck(NodeTable * detectedNodes,
//                                            BasicSafetyMessage * bsm);
    };
} // namespace artery
#endif