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
        LegacyChecks(double maxPlausibleSpeed, double maxPlausibleAcceleration, double MaxPlausibleDeceleration);

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

        double SuddenAppearenceCheck(Position &senderPosition, Position &receiverPosition);

        double PositionPlausibilityCheck(Position &senderPosition, double senderSpeed);

        double FrequencyCheck(long newTime,long oldTime) const;


        double
        PositionHeadingConsistencyCheck(const HeadingValue_t &currentHeading, Position &currentPosition,
                                        Position &oldPosition,
                                        double deltaTime, double currentSpeed) const;

        void
        KalmanPositionSpeedConsistencyCheck(Position *curPosition, Position *curPositionConfidence, Position *curSpeed,
                                            Position *oldSpeed, Position *curSpeedConfidence, double time,
                                            Kalman_SVI *kalmanSVI, double retVal[]);

        void KalmanPositionSpeedScalarConsistencyCheck(Position *curPosition, Position *oldPosition,
                                                       Position *curPositionConfidence, Position *curSpeed,
                                                       Position *oldSpeed, Position *curSpeedConfidence, double time,
                                                       Kalman_SC *kalmanSC, double retVal[]);

        double KalmanPositionConsistencyCheck(Position *curPosition, Position *oldPosition, Position *curPosConfidence,
                                              double time, Kalman_SI *kalmanSI);

        double KalmanPositionAccConsistencyCheck(Position *curPosition, Position *curSpeed, Position *curPosConfidence,
                                                 double time, Kalman_SI *kalmanSI);

        double KalmanSpeedConsistencyCheck(Position *curSpeed, Position *oldSpeed, Position *curSpeedConfidence,
                                           double time, Kalman_SI *kalmanSI);

//        InterTest MultipleIntersectionCheck(NodeTable * detectedNodes,
//                                            BasicSafetyMessage * bsm);
    };
} // namespace artery
#endif