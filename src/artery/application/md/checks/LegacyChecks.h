#ifndef ARTERY_LEGACYCHECK_H_
#define ARTERY_LEGACYCHECK_H_

#include "artery/application/md/CheckResult.h"
#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include <artery/application/md/kalman/Kalman_SVI.h>
#include <artery/application/md/kalman/Kalman_SC.h>
#include <artery/application/md/kalman/Kalman_SI.h>
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/application/VehicleDataProvider.h>
#include "artery/utility/Geometry.h"

namespace artery {

    class LegacyChecks {
    public:
        LegacyChecks(double maxPlausibleSpeed, double maxPlausibleAcceleration, double MaxPlausibleDeceleration);

        CheckResult checkCAM(vanetza::asn1::Cam);

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
        double maxCamFrequency;
        double maxOffroadSpeed;
        double positionHeadingTime;
        double maxHeadingChange;
        const VehicleDataProvider *mVehicleDataProvider;
        const LocalEnvironmentModel *mLocalEnvironmentModel;
        TrackedObjectsFilterRange envModObjects;
        vanetza::asn1::Cam lastCam;
        vanetza::asn1::Cam currentCam;
//        Position myPosition;

        static double calculateHeadingAngle(const Position position);

        double ProximityPlausibilityCheck(Position *testPosition, Position *myPosition, Position *myHeading);

        double RangePlausibilityCheck(Position *senderPosition, Position *receiverPosition);

        double PositionConsistencyCheck(Position *senderPosition, Position *receiverPostion, double time);

        double SpeedConsistencyCheck(double, double, double);

        double PositionSpeedConsistencyCheck(Position *, Position *, double, double, double);

        double PositionSpeedMaxConsistencyCheck(Position *, Position *, double, double, double);

        double SpeedPlausibilityCheck(double);

        double IntersectionCheck(Position nodePosition1, Position nodeSize1, Position head1, Position nodePosition2,
                                 Position nodeSize2, Position head2, double deltaTime);

        double SuddenAppearenceCheck(Position *, Position *);

        double FrequencyCheck(double, double);

        double PositionPlausibilityCheck(Position *, double);

        double PositionHeadingConsistencyCheck(Position *currentHeading, Position *currentPosition, Position *oldPosition,
                                               double deltaTime, double currentSpeed);

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