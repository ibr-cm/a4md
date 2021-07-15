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

    typedef boost::geometry::model::d2::point_xy<double> point;
    typedef boost::geometry::model::polygon<point> polygon;

    polygon createCircle(point center, double radius, int pointCount) {
        boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(radius);
        boost::geometry::strategy::buffer::join_round join_strategy(pointCount);
        boost::geometry::strategy::buffer::end_round end_strategy(pointCount);
        boost::geometry::strategy::buffer::point_circle circle_strategy(pointCount);
        boost::geometry::strategy::buffer::side_straight side_strategy;
        boost::geometry::model::multi_polygon<polygon> circleBuffer;

        boost::geometry::buffer(center, circleBuffer,
                                distance_strategy, side_strategy,
                                join_strategy, end_strategy, circle_strategy);
        return circleBuffer.front();
    }

    polygon createEllipse(point center, double semiMajorLength, double semiMinorLength, double semiMajorOrientation,
                          int pointCount) {
        std::vector<point> points;
        for (int i = 0; i < pointCount; i++) {
            double borderX = cos(2 * PI / pointCount * i) * semiMajorLength / 2;
            double borderY = sin(2 * PI / pointCount * i) * semiMinorLength / 2;

            double newAngle = semiMajorOrientation + atan2(-borderY, borderX) * 180 / PI;
            newAngle = 360 - std::fmod(newAngle, 360);

            double offsetDistance = sqrt(pow(borderX, 2) + pow(borderY, 2));
            double relativeX = offsetDistance * sin(newAngle * PI / 180);
            double relativeY = offsetDistance * cos(newAngle * PI / 180);
            points.emplace_back(point(center.x() + relativeX, center.y() + relativeY));
        }
        polygon ellipseOutline;
        boost::geometry::assign_points(ellipseOutline, points);
        boost::geometry::correct(ellipseOutline);
        return ellipseOutline;
    }

    double getIntersectionArea(const polygon &polygon1, const polygon &polygon2) {
        std::deque<polygon> intersectionGeometry;
        boost::geometry::intersection(polygon1, polygon2, intersectionGeometry);
        if (!intersectionGeometry.empty()) {
            return boost::geometry::area(intersectionGeometry.front());
        } else {
            return 0;
        }
    }

    double CatchChecks::RangePlausibilityCheck(const ReferencePosition_t &senderReferencePosition,
                                               const ReferencePosition_t &receiverReferencePosition) {
        double senderMajorLength = (double) senderReferencePosition.positionConfidenceEllipse.semiMajorConfidence / 100;
        double senderMinorLength = (double) senderReferencePosition.positionConfidenceEllipse.semiMinorConfidence / 100;
        double senderMajorOrientation =
                (double) senderReferencePosition.positionConfidenceEllipse.semiMajorOrientation / 10;
        Position senderPos = convertCamPosition(senderReferencePosition, mSimulationBoundary, mTraciAPI);
        point senderPosition(senderPos.x.value(), senderPos.y.value());

        double receiverMajorLength =
                (double) receiverReferencePosition.positionConfidenceEllipse.semiMajorConfidence / 100;
        double receiverMinorLength =
                (double) receiverReferencePosition.positionConfidenceEllipse.semiMinorConfidence / 100;
        double receiverMajorOrientation =
                (double) receiverReferencePosition.positionConfidenceEllipse.semiMajorOrientation / 10;
        Position receiverPos = convertCamPosition(receiverReferencePosition, mSimulationBoundary, mTraciAPI);
        point receiverPosition(receiverPos.x.value(), receiverPos.y.value());

        double distance = boost::geometry::distance(senderPosition, receiverPosition);
        if (distance > detectionParameters->maxPlausibleRange + senderMajorLength / 2 + receiverMajorLength / 2) {
            return 0;
        } else if (distance <
                   detectionParameters->maxPlausibleRange - senderMajorLength / 2 - receiverMajorLength / 2) {
            return 1;
        } else {
            int pointCount = 36;
            polygon senderEllipse = createEllipse(senderPosition, senderMajorLength, senderMinorLength,
                                                  senderMajorOrientation, pointCount);
            polygon receiverEllipse = createEllipse(receiverPosition, receiverMajorLength, receiverMinorLength,
                                                    receiverMajorOrientation,
                                                    pointCount);

            point rangeCenter = point(senderPosition.x() + fabs(receiverPosition.x() - senderPosition.x()) / 2,
                                      senderPosition.y() + fabs(receiverPosition.y() - senderPosition.y()) / 2);
            polygon range = createCircle(rangeCenter, detectionParameters->maxPlausibleRange / 2, pointCount);


            double senderArea = boost::geometry::area(senderEllipse);
            double receiverArea = boost::geometry::area(receiverEllipse);

            double senderRangeIntersectionArea = getIntersectionArea(senderEllipse, range);
            double receiverRangeIntersectionArea = getIntersectionArea(receiverEllipse, range);

            return (senderRangeIntersectionArea + receiverRangeIntersectionArea) / (senderArea + receiverArea);
        }
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
        result->rangePlausibility = RangePlausibilityCheck(receiverVDP->approximateReferencePosition(),
                                                           currentCam->cam.camParameters.basicContainer.referencePosition);
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
