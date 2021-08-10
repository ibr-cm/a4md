//
// Created by bastian on 15.07.21.
//

#ifndef ARTERY_BASECHECKS_H
#define ARTERY_BASECHECKS_H

#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include <artery/application/misbehavior/checks/CheckResult.h>
#include <artery/application/misbehavior/checks/kalman/Kalman_SVI.h>
#include <artery/application/misbehavior/checks/kalman/Kalman_SC.h>
#include <artery/application/misbehavior/checks/kalman/Kalman_SI.h>
#include <artery/application/misbehavior/checks/BaseChecks.h>
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/GlobalEnvironmentModel.h>
#include <artery/application/VehicleDataProvider.h>
#include <artery/application/misbehavior/util/F2MDParameters.h>
#include "artery/traci/VehicleController.h"
#include "artery/utility/Geometry.h"
#include "artery/application/misbehavior/fusion/ThresholdFusion.h"


namespace artery {

    class BaseChecks {
    public:
        BaseChecks(shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                   DetectionParameters *detectionParameters, const vanetza::asn1::Cam &message);

        BaseChecks(shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                   DetectionParameters *detectionParameters);


        virtual CheckResult *checkCAM(const VehicleDataProvider *receiverVDP,
                                      const std::vector<Position> &receiverVehicleOutline,
                                      const vanetza::asn1::Cam &currentCam, const vanetza::asn1::Cam *lastCamPtr,
                                      const std::vector<vanetza::asn1::Cam *> &surroundingCamObjects) = 0;

        virtual std::bitset<16> checkSemanticLevel2Report(const vanetza::asn1::Cam &currentCam,
                                                          const vanetza::asn1::Cam &lastCam) = 0;


    protected:
        static bool staticInitializationComplete;
        static GlobalEnvironmentModel *mGlobalEnvironmentModel;
        static traci::Boundary mSimulationBoundary;
        static std::shared_ptr<const traci::API> mTraciAPI;
        DetectionParameters *detectionParameters;
        ThresholdFusion *mThresholdFusion;

        Position mLastCamPosition;
        PosConfidenceEllipse_t mLastCamPositionConfidence;
        std::vector<Position> mLastCamPositionEllipse;
        double mLastCamEllipseRadius;
        double mLastCamSpeed;
        double mLastCamSpeedConfidence;
        Position mLastCamSpeedVector;


        double FrequencyCheck(const double &deltaTime) const;

        void KalmanChecks(const Position &currentCamPosition,
                          const PosConfidenceEllipse_t &currentCamPositionConfidence,
                          const double &currentCamSpeed,
                          const Position &currentCamSpeedVector, const double &currentCamSpeedConfidence,
                          const double &currentCamAcceleration, const Position &currentCamAccelerationVector,
                          const double &currentCamHeading, const Position &lastCamPosition,
                          const Position &lastCamSpeedVector, const double camDeltaTime,
                          CheckResult *result);

        void initializeKalmanFilters(const vanetza::asn1::Cam &message);

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
