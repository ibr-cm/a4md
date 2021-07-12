//
// Created by bastian on 12.07.21.
//

#include "CatchChecks.h"
#include <utility>
#include <artery/traci/Cast.h>
#include "artery/envmod/sensor/Sensor.h"
#include "artery/application/md/util/HelperFunctions.h"

namespace artery {
    bool CatchChecks::staticInitializationComplete = false;
    GlobalEnvironmentModel *CatchChecks::mGlobalEnvironmentModel;
    std::shared_ptr<const traci::API> CatchChecks::mTraciAPI;
    traci::Boundary CatchChecks::mSimulationBoundary;

    using namespace omnetpp;


    double CatchChecks::ProximityPlausibilityCheck(Position &testPosition, const Position &myPosition,
                                                   TrackedObjectsFilterRange &envModObjects) {
        Position::value_type deltaDistance = distance(testPosition, myPosition);
        double deltaAngle = calculateHeadingAngle(
                Position(testPosition.x - myPosition.x, testPosition.y - myPosition.y));

        if (deltaDistance.value() < detectionParameters->maxProximityRangeL) {
            if (deltaDistance.value() < detectionParameters->maxProximityRangeW * 2 ||
                (deltaAngle < 90 && deltaDistance.value() <
                                    (detectionParameters->maxProximityRangeW / cos((90 - deltaAngle) * PI / 180)))) {
                Position::value_type minimumDistance = Position::value_type::from_value(9999);

                for (const auto &object: envModObjects) {
                    std::weak_ptr<EnvironmentModelObject> obj_ptr = object.first;
                    if (obj_ptr.expired()) continue; /*< objects remain in tracking briefly after leaving simulation */
                    const auto &vd = obj_ptr.lock()->getVehicleData();
                    Position::value_type currentDistance = distance(testPosition, vd.position());
                    if (currentDistance < minimumDistance) {
                        minimumDistance = currentDistance;
                    }
                }
                if (minimumDistance.value() < 2 * detectionParameters->maxProximityDistance) {
                    return 1 - minimumDistance.value() / (2 * detectionParameters->maxProximityDistance);
                } else {
                    return 0;
                }
            }
        }
        return 1;
    }

    CheckResult *
    CatchChecks::checkCAM(const VehicleDataProvider *receiverVDP, const std::vector<Position> &receiverVehicleOutline,
                          TrackedObjectsFilterRange &envModObjects,
                          const vanetza::asn1::Cam &currentCam, const vanetza::asn1::Cam *lastCamPtr) {
        BasicVehicleContainerHighFrequency_t highFrequencyContainer = currentCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;

        Position currentCamPosition = convertCamPosition(
                currentCam->cam.camParameters.basicContainer.referencePosition, mSimulationBoundary, mTraciAPI);
        const Position &receiverPosition = receiverVDP->position();
        double currentCamSpeed = (double) highFrequencyContainer.speed.speedValue / 100.0;
        double currentCamSpeedConfidence = (double) highFrequencyContainer.speed.speedConfidence / 100.0;
        double currentCamAcceleration =
                (double) highFrequencyContainer.longitudinalAcceleration.longitudinalAccelerationValue / 10.0;
        double currentCamHeading = angle_cast(
                traci::TraCIAngle((double) highFrequencyContainer.heading.headingValue / 10.0)).value.value();
        Position currentCamSpeedVector = getVector(currentCamSpeed, currentCamHeading);
        Position currentCamAccelerationVector = getVector(currentCamAcceleration, currentCamHeading);
        auto *result = new CheckResult;
        return result;
    }


    CatchChecks::CatchChecks(std::shared_ptr<const traci::API> traciAPI,
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
