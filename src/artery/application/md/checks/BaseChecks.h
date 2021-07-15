//
// Created by bastian on 15.07.21.
//

#ifndef ARTERY_BASECHECKS_H
#define ARTERY_BASECHECKS_H

#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
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
        BaseChecks(std::shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                   DetectionParameters *detectionParameters,
                   Kalman_SVI *kalmanSVI, Kalman_SC *kalmanSVSI,
                   Kalman_SI *kalmanSI, Kalman_SI *kalmanVI);

    protected:
        static bool staticInitializationComplete;
        static GlobalEnvironmentModel *mGlobalEnvironmentModel;
        static traci::Boundary mSimulationBoundary;
        static std::shared_ptr<const traci::API> mTraciAPI;
        DetectionParameters *detectionParameters;

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
    private:
        Kalman_SVI *kalmanSVI;
        Kalman_SC *kalmanSVSI;
        Kalman_SI *kalmanSI;
        Kalman_SI *kalmanVI;
    };

} // namespace artery

#endif //ARTERY_BASECHECKS_H
