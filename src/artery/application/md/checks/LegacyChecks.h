#ifndef ARTERY_LEGACYCHECK_H_
#define ARTERY_LEGACYCHECK_H_

#include "CheckResult.h"
#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include <artery/application/md/kalman/Kalman_SVI.h>
#include <artery/application/md/kalman/Kalman_SC.h>
#include <artery/application/md/kalman/Kalman_SI.h>
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/application/VehicleDataProvider.h>
#include "artery/traci/VehicleController.h"
#include "artery/utility/Geometry.h"

namespace artery {

    class LegacyChecks {
    public:
        LegacyChecks();

        CheckResult checkCAM(const vanetza::asn1::Cam &);

    private:
        double maxPlausibleSpeed;
        double maxPlausibleAcceleration;
        double maxPlausibleDeceleration;

        double maxProximityRangeL;
        double maxProximityRangeW;
        double maxProximityDistance;
        double maxPlausibleRange;
        double maxTimeDelta;
        double maxMgtRng;
        double maxMgtRngDown;
        double maxMgtRngUp;
        double maxSuddenAppearanceRange;
        long maxCamFrequency;
        double maxOffroadSpeed;
        double positionHeadingTime;
        double maxHeadingChange;

        double maxKalmanTime;
        double kalmanMinPosRange;
        double kalmanMinSpeedRange;
        double kalmanMinHeadingRange;
        double kalmanPosRange;
        double kalmanSpeedRange;

        Kalman_SVI *kalmanSVI;
        Kalman_SC *kalmanSVSI;
        Kalman_SI *kalmanSI;
        Kalman_SI *kalmanVI;
        Kalman_SI *kalmanSAI;


        const traci::VehicleController *mVehicleController = nullptr;
        const VehicleDataProvider *mVehicleDataProvider;
        const LocalEnvironmentModel *mLocalEnvironmentModel;
        TrackedObjectsFilterRange envModObjects;


        Position lastCamPosition;
        double lastCamSpeed;
        vanetza::asn1::Cam lastCam;
        double camDeltaTime;
        Position mPosition;

        static double calculateHeadingAngle(const Position &position);

        double ProximityPlausibilityCheck(Position &testPosition, Position &myPosition);

        double RangePlausibilityCheck(Position &senderPosition, Position &receiverPosition) const;

        double PositionConsistencyCheck(Position &senderPosition, Position &receiverPosition, double time) const;

        double SpeedConsistencyCheck(double currentSpeed, double oldSpeed, double time) const;

        double PositionSpeedConsistencyCheck(Position &currentPosition, Position &oldPosition, double currentSpeed,
                                             double oldSpeed, double deltaTime) const;

        double PositionSpeedMaxConsistencyCheck(Position &currentPosition, Position &oldPosition,
                                                double currentSpeed, double oldSpeed, double deltaTime) const;

        double SpeedPlausibilityCheck(double speed) const;

        double IntersectionCheck(Position nodePosition1, Position nodeSize1, Position head1, Position nodePosition2,
                                 Position nodeSize2, Position head2, double deltaTime);

        double SuddenAppearanceCheck(Position &senderPosition, Position &receiverPosition) const;

        double PositionPlausibilityCheck(Position &senderPosition, double senderSpeed) const;

        double FrequencyCheck(long newTime, long oldTime) const;


        double
        PositionHeadingConsistencyCheck(const HeadingValue_t &currentHeading, Position &currentPosition,
                                        Position &oldPosition,
                                        double deltaTime, double currentSpeed) const;

        void KalmanPositionSpeedConsistencyCheck(Position &currentPosition,
                                                 const PosConfidenceEllipse_t &currentPositionConfidence,
                                                 double &currentSpeed, double &currentAcceleration,
                                                 double &currentHeading,
                                                 double &currentSpeedConfidence, double &currentHeadingConfidence,
                                                 double &deltaTime,
                                                 double *returnValue) const;

        void KalmanPositionSpeedScalarConsistencyCheck(Position &currentPosition, Position &oldPosition,
                                                       const PosConfidenceEllipse_t &currentPositionConfidence,
                                                       double &currentSpeed, double &currentAcceleration,
                                                       double &currentSpeedConfidence, double &deltaTime,
                                                       double *returnValue);

        double KalmanPositionConsistencyCheck(Position &currentPosition, Position &oldPosition,
                                              const PosConfidenceEllipse_t &currentPositionConfidence,
                                              double &deltaTime);

        double
        KalmanPositionAccConsistencyCheck(Position &currentPosition, double &currentSpeed, double &currentHeading,
                                          const PosConfidenceEllipse_t &currentPositionConfidence,
                                          double &deltaTime);

        double KalmanSpeedConsistencyCheck(double &currentSpeed, double &oldSpeed, double &currentSpeedConfidence,
                                           double &currentHeading, double &currentHeadingConfidence,
                                           double &currentAcceleration, double &currentAccelerationConfidence,
                                           double &deltaTime);

//        InterTest MultipleIntersectionCheck(NodeTable * detectedNodes,
//                                            BasicSafetyMessage * bsm);
    };
} // namespace artery
#endif