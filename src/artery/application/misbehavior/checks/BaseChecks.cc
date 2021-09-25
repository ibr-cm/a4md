//
// Created by bastian on 15.07.21.
//

#include "BaseChecks.h"

#include <utility>
#include "artery/application/misbehavior/util/HelperFunctions.h"

namespace artery {
    bool BaseChecks::staticInitializationComplete = false;
    GlobalEnvironmentModel *BaseChecks::mGlobalEnvironmentModel;
    std::shared_ptr<const traci::API> BaseChecks::mTraciAPI;
    traci::Boundary BaseChecks::mSimulationBoundary;
    const Timer *BaseChecks::mTimer;

    using namespace omnetpp;

    BaseChecks::BaseChecks(std::shared_ptr<const traci::API> traciAPI,
                           GlobalEnvironmentModel *globalEnvironmentModel,
                           DetectionParameters *detectionParameters,
                           const Timer *timer,
                           std::map<detectionLevels::DetectionLevels, bool> checkableDetectionLevels,
                           const std::shared_ptr<vanetza::asn1::Cam> &message) :
            detectionParameters(detectionParameters),
            mCheckableDetectionLevels(std::move(checkableDetectionLevels)) {
        if (!staticInitializationComplete) {
            staticInitializationComplete = true;
            mGlobalEnvironmentModel = globalEnvironmentModel;
            mTraciAPI = std::move(traciAPI);
            mSimulationBoundary = traci::Boundary{mTraciAPI->simulation.getNetBoundary()};
            mTimer = timer;
        }
        Position position = convertReferencePosition(
                (*message)->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
        double speed =
                (double) (*message)->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue /
                100.0;
        double heading =
                (double) (*message)->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue /
                10.0;
        Position speedVector = getVector(speed, heading);
        kalmanSVI = new Kalman_SVI();
        kalmanSVSI = new Kalman_SC();
        kalmanSI = new Kalman_SI();
        kalmanVI = new Kalman_SI();
        kalmanSVI->setInitial(position.x.value(), position.y.value(), speedVector.x.value(), speedVector.y.value());
        kalmanSVSI->setInitial(0, speed);
        kalmanSI->setInitial(position.x.value(), position.y.value());
        kalmanVI->setInitial(speedVector.x.value(), speedVector.y.value());
    }

    BaseChecks::BaseChecks(std::shared_ptr<const traci::API> traciAPI,
                           GlobalEnvironmentModel *globalEnvironmentModel,
                           DetectionParameters *detectionParameters,
                           const Timer *timer) :
            detectionParameters(detectionParameters) {
        if (!staticInitializationComplete) {
            staticInitializationComplete = true;
            mGlobalEnvironmentModel = globalEnvironmentModel;
            mTraciAPI = std::move(traciAPI);
            mSimulationBoundary = traci::Boundary{mTraciAPI->simulation.getNetBoundary()};
            mTimer = timer;
        }
    }

    void BaseChecks::initializeKalmanFilters(const std::shared_ptr<vanetza::asn1::Cam> &message) {
        Position position = convertReferencePosition(
                (*message)->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
        double speed =
                (double) (*message)->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue /
                100.0;
        double heading =
                (double) (*message)->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue /
                10.0;
        Position speedVector = getVector(speed, heading);
        delete kalmanSVI;
        delete kalmanSVSI;
        delete kalmanSI;
        delete kalmanVI;
        kalmanSVI = new Kalman_SVI();
        kalmanSVSI = new Kalman_SC();
        kalmanSI = new Kalman_SI();
        kalmanVI = new Kalman_SI();
        kalmanSVI->setInitial(position.x.value(), position.y.value(), speedVector.x.value(), speedVector.y.value());
        kalmanSVSI->setInitial(0, speed);
        kalmanSI->setInitial(position.x.value(), position.y.value());
        kalmanVI->setInitial(speedVector.x.value(), speedVector.y.value());
    }

    double BaseChecks::FrequencyCheck(const double &deltaTime) const {
        if (deltaTime > detectionParameters->maxCamFrequency * 1000) {
            return 0;
        } else {
            return 1;
        }
    }

    void BaseChecks::KalmanPositionSpeedConsistencyCheck(const Position &currentPosition,
                                                         const PosConfidenceEllipse_t &currentPositionConfidence,
                                                         const Position &currentSpeed,
                                                         const Position &currentAcceleration,
                                                         const double &currentSpeedConfidence,
                                                         const double &deltaTime, double *returnValue) const {
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
                double currentSpeedConf = std::max(currentSpeedConfidence, detectionParameters->kalmanMinSpeedRange);

                kalmanSVI->getDeltaPos(deltaTime, currentPosition.x.value(), currentPosition.y.value(),
                                       currentSpeed.x.value(), currentSpeed.y.value(), currentAcceleration.x.value(),
                                       currentAcceleration.y.value(), currentSemiMajorConfidence,
                                       currentSemiMinorConfidence,
                                       currentSpeedConf, currentSpeedConf, deltaPosition);

                double ret_1 = 1 - sqrt(pow(deltaPosition[0], 2.0) + pow(deltaPosition[2], 2.0)) /
                                   (detectionParameters->kalmanPosRange * currentSemiMajorConfidence * deltaTime);

                double ret_2 = 1 - sqrt(pow(deltaPosition[1], 2.0) + pow(deltaPosition[3], 2.0)) /
                                   (detectionParameters->kalmanSpeedRange * currentSpeedConf * deltaTime);
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


    void
    BaseChecks::KalmanPositionSpeedScalarConsistencyCheck(const Position &currentPosition, const Position &oldPosition,
                                                          const PosConfidenceEllipse_t &currentPositionConfidence,
                                                          const double &currentSpeed, const double &currentAcceleration,
                                                          const double &currentSpeedConfidence, const double &deltaTime,
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

    double BaseChecks::KalmanPositionConsistencyCheck(const Position &currentPosition, const Position &oldPosition,
                                                      const PosConfidenceEllipse_t &currentPositionConfidence,
                                                      const double &deltaTime) {
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
                                                  const double &deltaTime) {
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
                                            const double &currentSpeedConfidence, const Position &currentAcceleration,
                                            const double &deltaTime) {
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

    void BaseChecks::KalmanChecks(const Position &currentCamPosition,
                                  const PosConfidenceEllipse_t &currentCamPositionConfidence,
                                  const double &currentCamSpeed,
                                  const Position &currentCamSpeedVector, const double &currentCamSpeedConfidence,
                                  const double &currentCamAcceleration, const Position &currentCamAccelerationVector,
                                  const double &currentCamHeading, const Position &lastCamPosition,
                                  const Position &lastCamSpeedVector, const double camDeltaTime,
                                  const std::shared_ptr<CheckResult> &result) {
        double returnValue[2];
        KalmanPositionSpeedConsistencyCheck(currentCamPosition, currentCamPositionConfidence, currentCamSpeedVector,
                                            currentCamAccelerationVector,
                                            currentCamSpeedConfidence, camDeltaTime, returnValue);
        result->kalmanPositionSpeedConsistencyPosition = returnValue[0];
        result->kalmanPositionSpeedConsistencySpeed = returnValue[1];
        KalmanPositionSpeedScalarConsistencyCheck(currentCamPosition, lastCamPosition, currentCamPositionConfidence,
                                                  currentCamSpeed, currentCamAcceleration,
                                                  currentCamSpeedConfidence, camDeltaTime, returnValue);
        result->kalmanPositionSpeedScalarConsistencyPosition = returnValue[0];
        result->kalmanPositionSpeedScalarConsistencySpeed = returnValue[1];
        result->kalmanPositionConsistencyConfidence =
                KalmanPositionConsistencyCheck(currentCamPosition, lastCamPosition, currentCamPositionConfidence,
                                               camDeltaTime);
        result->kalmanPositionAccelerationConsistencySpeed =
                KalmanPositionAccConsistencyCheck(currentCamPosition, currentCamSpeedVector,
                                                  currentCamPositionConfidence,
                                                  camDeltaTime);
        result->kalmanSpeedConsistencyConfidence =
                KalmanSpeedConsistencyCheck(currentCamSpeedVector, lastCamSpeedVector, currentCamSpeedConfidence,
                                            currentCamAccelerationVector,
                                            camDeltaTime);
    }

//    CheckResult *
//    BaseChecks::checkCAM(const VehicleDataProvider *receiverVDP, const vector<Position> &receiverVehicleOutline,
//                         const vanetza::asn1::Cam &currentCam, const vanetza::asn1::Cam *lastCamPtr,
//                         const vector<vanetza::asn1::Cam *> &surroundingCamObjects) {
//        return nullptr;
//    }

} // namespace artery