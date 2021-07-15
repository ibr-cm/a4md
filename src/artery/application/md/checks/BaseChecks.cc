//
// Created by bastian on 15.07.21.
//

#include "BaseChecks.h"

namespace artery {
    bool BaseChecks::staticInitializationComplete = false;
    GlobalEnvironmentModel *BaseChecks::mGlobalEnvironmentModel;
    std::shared_ptr<const traci::API> BaseChecks::mTraciAPI;
    traci::Boundary BaseChecks::mSimulationBoundary;

    using namespace omnetpp;

    void BaseChecks::KalmanPositionSpeedConsistencyCheck(Position &currentPosition,
                                                         const PosConfidenceEllipse_t &currentPositionConfidence,
                                                         const Position &currentSpeed,
                                                         const Position &currentAcceleration,
                                                         double currentSpeedConfidence,
                                                         double &deltaTime, double *returnValue) const {
        if (!kalmanSVI->isInitialized()) {
            returnValue[0] = 1;
            returnValue[1] = 1;
        } else {
            if (deltaTime < detectionParameters->maxKalmanTime) {
                double deltaPosition[4];

                double currentSemiMajorConfidence = std::max(
                        (double) currentPositionConfidence.semiMajorConfidence / 100.0,
                        detectionParameters->kalmanMinPosRange);
                double currentSemiMinorConfidence = std::max(
                        (double) currentPositionConfidence.semiMinorConfidence / 100.0,
                        detectionParameters->kalmanMinPosRange);
                currentSpeedConfidence = std::max(currentSpeedConfidence, detectionParameters->kalmanMinSpeedRange);

                kalmanSVI->getDeltaPos(deltaTime, currentPosition.x.value(), currentPosition.y.value(),
                                       currentSpeed.x.value(), currentSpeed.y.value(), currentAcceleration.x.value(),
                                       currentAcceleration.y.value(), currentSemiMajorConfidence,
                                       currentSemiMinorConfidence,
                                       currentSpeedConfidence, currentSpeedConfidence, deltaPosition);

                double ret_1 = 1 - sqrt(pow(deltaPosition[0], 2.0) + pow(deltaPosition[2], 2.0)) /
                                   (detectionParameters->kalmanPosRange * currentSemiMajorConfidence * deltaTime);

                double ret_2 = 1 - sqrt(pow(deltaPosition[1], 2.0) + pow(deltaPosition[3], 2.0)) /
                                   (detectionParameters->kalmanSpeedRange * currentSpeedConfidence * deltaTime);
                returnValue[0] = isnan(ret_1) || ret_1 < 0.5 ? 0 : 1;
                returnValue[1] = isnan(ret_2) || ret_2 < 0.5 ? 0 : 1;
            } else {
                returnValue[0] = 1;
                returnValue[1] = 1;
                kalmanSVI->setInitial(currentPosition.x.value(), currentPosition.y.value(), currentSpeed.x.value(),
                                      currentSpeed.y.value());
            }
        }
    }


    void BaseChecks::KalmanPositionSpeedScalarConsistencyCheck(Position &currentPosition, Position &oldPosition,
                                                               const PosConfidenceEllipse_t &currentPositionConfidence,
                                                               double &currentSpeed, double &currentAcceleration,
                                                               double &currentSpeedConfidence, double &deltaTime,
                                                               double *returnValue) {
        if (!kalmanSVSI->isInitialized()) {
            returnValue[0] = 1;
            returnValue[1] = 1;
        } else {
            if (deltaTime < detectionParameters->maxKalmanTime) {
                double deltaPosition[2];

                double deltaDistance = distance(currentPosition, oldPosition).value();
                double curPosConfX = std::max((double) currentPositionConfidence.semiMajorConfidence,
                                              detectionParameters->kalmanMinPosRange);
                double curSpdConfX = std::max(currentSpeedConfidence, detectionParameters->kalmanSpeedRange);

                kalmanSVSI->getDeltaPos(deltaTime, deltaDistance, currentSpeed, currentAcceleration,
                                        currentAcceleration,
                                        curPosConfX, curSpdConfX, deltaPosition);

                double ret_1 = 1 - (deltaPosition[0] / (detectionParameters->kalmanPosRange * curPosConfX * deltaTime));
                double ret_2 =
                        1 - (deltaPosition[1] / (detectionParameters->kalmanSpeedRange * curSpdConfX * deltaTime));
                returnValue[0] = isnan(ret_1) || ret_1 < 0.5 ? 0 : 1;
                returnValue[1] = isnan(ret_2) || ret_2 < 0.5 ? 0 : 1;
            } else {
                returnValue[0] = 1;
                returnValue[1] = 1;
                kalmanSVSI->setInitial(0, currentSpeed);
            }
        }
    }

    double BaseChecks::KalmanPositionConsistencyCheck(Position &currentPosition, Position &oldPosition,
                                                      const PosConfidenceEllipse_t &currentPositionConfidence,
                                                      double &deltaTime) {
        if (!kalmanSI->isInit()) {
            return 1;
        } else {
            if (deltaTime < detectionParameters->maxKalmanTime) {
                double deltaPosition[2];
                double curPosConfX = std::max((double) currentPositionConfidence.semiMajorConfidence,
                                              detectionParameters->kalmanMinPosRange);
                double curPosConfY = std::max((double) currentPositionConfidence.semiMinorConfidence,
                                              detectionParameters->kalmanMinPosRange);

                kalmanSI->getDeltaPos(deltaTime, currentPosition.x.value(), currentPosition.y.value(),
                                      curPosConfX, curPosConfY, deltaPosition);

                double ret_1 = 1 - sqrt(pow(deltaPosition[0], 2.0) + pow(deltaPosition[1], 2.0)) /
                                   (4 * detectionParameters->kalmanPosRange * curPosConfX * deltaTime);
                return isnan(ret_1) || ret_1 < 0.5 ? 0 : 1;
            } else {
                kalmanSI->setInitial(currentPosition.x.value(), currentPosition.y.value());
                return 1;
            }
        }
    }

    double
    BaseChecks::KalmanPositionAccConsistencyCheck(const Position &currentPosition, const Position &currentSpeed,
                                                  const PosConfidenceEllipse_t &currentPositionConfidence,
                                                  double &deltaTime) {
        if (!kalmanSI->isInit()) {
            return 1;
        } else {
            if (deltaTime < detectionParameters->maxKalmanTime) {
                double deltaPosition[2];
                double curPosConfX = std::max((double) currentPositionConfidence.semiMajorConfidence,
                                              detectionParameters->kalmanMinPosRange);
                double curPosConfY = std::max((double) currentPositionConfidence.semiMinorConfidence,
                                              detectionParameters->kalmanMinPosRange);

                kalmanSI->getDeltaPos(deltaTime, currentPosition.x.value(), currentPosition.y.value(),
                                      currentSpeed.x.value(),
                                      currentSpeed.y.value(),
                                      curPosConfX, curPosConfY, deltaPosition);

                double ret_1 = 1 - sqrt(pow(deltaPosition[0], 2.0) + pow(deltaPosition[1], 2.0)) /
                                   (4 * detectionParameters->kalmanPosRange * curPosConfX * deltaTime);
                return isnan(ret_1) || ret_1 < 0.5 ? 0 : 1;
            } else {
                kalmanSI->setInitial(currentPosition.x.value(), currentPosition.x.value());
                return 1;
            }
        }
    }

    double
    BaseChecks::KalmanSpeedConsistencyCheck(const Position &currentSpeed, const Position &oldSpeed,
                                            double &currentSpeedConfidence, const Position &currentAcceleration,
                                            double &deltaTime) {
        if (!kalmanVI->isInit()) {
            return 1;
        } else {
            if (deltaTime < detectionParameters->maxKalmanTime) {
                double deltaPosition[2];

                double curSpdConf = std::max(currentSpeedConfidence, detectionParameters->kalmanMinSpeedRange);

                kalmanVI->getDeltaPos(deltaTime, currentSpeed.x.value(), currentSpeed.y.value(),
                                      currentAcceleration.x.value(),
                                      currentAcceleration.y.value(), curSpdConf, curSpdConf, deltaPosition);

                double ret_1 = 1 - sqrt(pow(deltaPosition[0], 2.0) + pow(deltaPosition[1], 2.0)) /
                                   (detectionParameters->kalmanSpeedRange * curSpdConf * deltaTime);
                return isnan(ret_1) || ret_1 < 0.5 ? 0 : 1;
            } else {
                kalmanVI->setInitial(currentSpeed.x.value(), currentSpeed.y.value());
                return 1;
            }
        }
    }

    BaseChecks::BaseChecks(std::shared_ptr<const traci::API> traciAPI,
                           GlobalEnvironmentModel *globalEnvironmentModel,
                           DetectionParameters *detectionParameters,
                           Kalman_SVI *kalmanSVI, Kalman_SC *kalmanSVSI,
                           Kalman_SI *kalmanSI, Kalman_SI *kalmanVI) :
            detectionParameters(detectionParameters),
            kalmanSVI(kalmanSVI), kalmanSVSI(kalmanSVSI),
            kalmanSI(kalmanSI), kalmanVI(kalmanVI) {
        if (!staticInitializationComplete) {
            staticInitializationComplete = true;
            mGlobalEnvironmentModel = globalEnvironmentModel;
            mTraciAPI = std::move(traciAPI);
            mSimulationBoundary = traci::Boundary{mTraciAPI->simulation.getNetBoundary()};
        }
    }
} // namespace artery