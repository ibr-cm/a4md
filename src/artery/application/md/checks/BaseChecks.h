//
// Created by bastian on 15.07.21.
//

#ifndef ARTERY_BASECHECKS_H
#define ARTERY_BASECHECKS_H

#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include <artery/application/md/checks/CheckResult.h>
#include <artery/application/md/checks/kalman/Kalman_SVI.h>
#include <artery/application/md/checks/kalman/Kalman_SC.h>
#include <artery/application/md/checks/kalman/Kalman_SI.h>
#include <artery/application/md/checks/BaseChecks.h>
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/GlobalEnvironmentModel.h>
#include <artery/application/VehicleDataProvider.h>
#include <artery/application/md/util/F2MDParameters.h>
#include "artery/traci/VehicleController.h"
#include "artery/utility/Geometry.h"

namespace artery {

    class BaseChecks {
    public:

    protected:
        BaseChecks(shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                   DetectionParameters *detectionParameters, const vanetza::asn1::Cam &message);

        static bool staticInitializationComplete;
        static GlobalEnvironmentModel *mGlobalEnvironmentModel;
        static traci::Boundary mSimulationBoundary;
        static std::shared_ptr<const traci::API> mTraciAPI;
        DetectionParameters *detectionParameters;

        Position lastCamPosition;
        PosConfidenceEllipse_t lastCamPositionConfidence;
        std::vector<Position> lastCamPositionEllipse;
        double lastCamEllipseRadius;
        double lastCamSpeed;
        double lastCamSpeedConfidence;
        Position lastCamSpeedVector;


        double BaseChecks::FrequencyCheck(const double &deltaTime) const;

        void KalmanChecks(const Position &currentCamPosition,
                          const PosConfidenceEllipse_t &currentCamPositionConfidence,
                          const double &currentCamSpeed,
                          const Position &currentCamSpeedVector, const double &currentCamSpeedConfidence,
                          const double &currentCamAcceleration, const Position &currentCamAccelerationVector,
                          const double &currentCamHeading, const Position &lastCamPosition,
                          const Position &lastCamSpeedVector, const double camDeltaTime,
                          CheckResult *result);

    private:
        Kalman_SVI *kalmanSVI;
        Kalman_SC *kalmanSVSI;
        Kalman_SI *kalmanSI;
        Kalman_SI *kalmanVI;

        void KalmanPositionSpeedConsistencyCheck(const Position &currentPosition,
                                                 const PosConfidenceEllipse_t &currentPositionConfidence,
                                                 const Position &currentSpeed, const Position &currentAcceleration,
                                                 const double &currentSpeedConfidence,
                                                 const double &deltaTime, double *returnValue) const;

        void KalmanPositionSpeedScalarConsistencyCheck(const Position &currentPosition, const Position &oldPosition,
                                                       const PosConfidenceEllipse_t &currentPositionConfidence,
                                                       const double &currentSpeed, const double &currentAcceleration,
                                                       const double &currentSpeedConfidence, const double &deltaTime,
                                                       double *returnValue);

        double KalmanPositionConsistencyCheck(const Position &currentPosition, const Position &oldPosition,
                                              const PosConfidenceEllipse_t &currentPositionConfidence,
                                              const double &deltaTime);

        double KalmanPositionAccConsistencyCheck(const Position &currentPosition, const Position &currentSpeed,
                                                 const PosConfidenceEllipse_t &currentPositionConfidence,
                                                 const double &deltaTime);

        double KalmanSpeedConsistencyCheck(const Position &currentSpeed, const Position &oldSpeed,
                                           const double &currentSpeedConfidence, const Position &currentAcceleration,
                                           const double &deltaTime);

    };

} // namespace artery

#endif //ARTERY_BASECHECKS_H
