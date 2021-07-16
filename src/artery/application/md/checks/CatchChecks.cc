//
// Created by bastian on 12.07.21.
//

#include "CatchChecks.h"
#include <utility>
#include <artery/traci/Cast.h>
#include "artery/application/md/util/HelperFunctions.h"

namespace artery {

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

//    std::vector<Position> createCircle(const Position &center, double radius, int pointCount) {
//        boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(radius);
//        boost::geometry::strategy::buffer::join_round join_strategy(pointCount);
//        boost::geometry::strategy::buffer::end_round end_strategy(pointCount);
//        boost::geometry::strategy::buffer::point_circle circle_strategy(pointCount);
//        boost::geometry::strategy::buffer::side_straight side_strategy;
//        boost::geometry::model::multi_polygon<polygon> circleBuffer;
//
//        boost::geometry::buffer(center, circleBuffer,
//                                distance_strategy, side_strategy,
//                                join_strategy, end_strategy, circle_strategy);
//        std::vector<Position> circleOutline;
//        polygon circle = circleBuffer.front();
//        for (auto p : circle.outer()) {
//            circleOutline.emplace_back(Position(p.x(), p.y()));
//        }
//        return circleOutline;
//    }

    std::vector<Position>
    createEllipse(const Position &center, double semiMajorLength, double semiMinorLength, double semiMajorOrientation,
                  int pointCount) {
        std::vector<Position> ellipseOutline;
        for (int i = 0; i < pointCount; i++) {
            double borderX = cos(2 * PI / pointCount * i) * semiMajorLength / 2;
            double borderY = sin(2 * PI / pointCount * i) * semiMinorLength / 2;
            double newAngle = semiMajorOrientation + atan2(-borderY, borderX) * 180 / PI;
//        newAngle = 360 - std::fmod(newAngle, 360);
            newAngle = std::fmod(newAngle, 360);

            double offsetDistance = sqrt(pow(borderX, 2) + pow(borderY, 2));
            double relativeX = offsetDistance * sin(newAngle * PI / 180);
            double relativeY = offsetDistance * cos(newAngle * PI / 180);
//            point p(center.x() + relativeX, center.y() + relativeY);
//            Position p(center.x.value() + relativeX, center.y.value() + relativeY);
            ellipseOutline.emplace_back(Position(center.x.value() + relativeX, center.y.value() + relativeY));
        }
//        polygon ellipseOutline;
//        boost::geometry::assign_points(ellipseOutline, points);
        boost::geometry::correct(ellipseOutline);
        return ellipseOutline;
    }

    std::vector<Position> createEllipse(const Position &position, const PosConfidenceEllipse_t &posConfidenceEllipse) {
        double semiMajorConfidence = (double) posConfidenceEllipse.semiMajorConfidence / 100;
        double semiMinorConfidence = (double) posConfidenceEllipse.semiMinorConfidence / 100;
        double semiMajorOrientation = (double) posConfidenceEllipse.semiMajorOrientation / 10;
//        point p = point(position.x.value(), position.y.value());
        return createEllipse(position, semiMajorConfidence, semiMinorConfidence, semiMajorOrientation, 36);
    }

    double getIntersectionArea(const std::vector<Position> &polygon1, const std::vector<Position> &polygon2) {
        std::deque<std::vector<Position>> intersectionGeometry;
        boost::geometry::intersection(polygon1, polygon2, intersectionGeometry);
        if (!intersectionGeometry.empty()) {
            return boost::geometry::area(intersectionGeometry.front());
        } else {
            return 0;
        }
    }

    double confidenceIntersectionArea(const Position &position1,
                                      const PosConfidenceEllipse_t &confidenceEllipse1,
                                      const Position &position2,
                                      const PosConfidenceEllipse_t &confidenceEllipse2, double range) {
        std::vector<Position> senderEllipse = createEllipse(position1, confidenceEllipse1);
        std::vector<Position> receiverEllipse = createEllipse(position2, confidenceEllipse2);

        Position rangeCenter = Position(
                position1.x.value() + fabs(position2.x.value() - position1.x.value()) / 2,
                position1.y.value() + fabs(position2.y.value() - position1.y.value()) / 2);
        std::vector<Position> rangeCircle = createEllipse(rangeCenter, range, range, 0, 36);
        double distance = boost::geometry::distance(position1, position2);
        if (distance > range * 2 + (double) confidenceEllipse1.semiMajorConfidence / 200 +
                       (double) confidenceEllipse2.semiMajorConfidence / 200) {
            return 0;
        } else if (distance < range * 2 -
                              (double) confidenceEllipse1.semiMajorConfidence / 200 -
                              (double) confidenceEllipse2.semiMajorConfidence / 200) {
            return 1;
        } else {
            double senderArea = boost::geometry::area(senderEllipse);
            double receiverArea = boost::geometry::area(receiverEllipse);

            double senderRangeIntersectionArea = getIntersectionArea(senderEllipse, rangeCircle);
            double receiverRangeIntersectionArea = getIntersectionArea(receiverEllipse, rangeCircle);

            return (senderRangeIntersectionArea + receiverRangeIntersectionArea) / (senderArea + receiverArea);
        }
    }

    double CatchChecks::RangePlausibilityCheck(const Position &senderPosition,
                                               const PosConfidenceEllipse_t &senderConfidenceEllipse,
                                               const Position &receiverPosition,
                                               const PosConfidenceEllipse_t &receiverConfidenceEllipse) {
        return confidenceIntersectionArea(senderPosition, senderConfidenceEllipse, receiverPosition,
                                          receiverConfidenceEllipse,
                                          detectionParameters->maxPlausibleRange);
    }


    double
    CatchChecks::PositionConsistencyCheck(const Position &currentPosition,
                                          const PosConfidenceEllipse_t &currentConfidenceEllipse,
                                          const Position &oldPosition,
                                          const PosConfidenceEllipse_t &oldConfidenceEllipse,
                                          double deltaTime) {
        return confidenceIntersectionArea(currentPosition, currentConfidenceEllipse, oldPosition, oldConfidenceEllipse,
                                          detectionParameters->maxPlausibleSpeed * deltaTime);
    }

    double CatchChecks::SpeedConsistencyCheck(const double &currentSpeed, const double &currentSpeedConfidence,
                                              const double &oldSpeed, const double &oldSpeedConfidence,
                                              const double &deltaTime) {
        double f_max = (detectionParameters->maxPlausibleAcceleration - currentSpeed + oldSpeedConfidence) /
                       (4 * oldSpeedConfidence) +
                       (detectionParameters->maxPlausibleAcceleration - currentSpeed + currentSpeedConfidence) /
                       (4 * currentSpeedConfidence);
        double f_min = (currentSpeed - detectionParameters->maxPlausibleDeceleration + oldSpeed) /
                       (4 * oldSpeedConfidence) +
                       (currentSpeed - detectionParameters->maxPlausibleDeceleration + currentSpeedConfidence) /
                       (4 * currentSpeedConfidence);
        if (currentSpeed > oldSpeed) {
            return f_max;
        } else {
            return f_min;
        }
    }

    double CatchChecks::SpeedPlausibilityCheck(const double &currentSpeed, const double &currentSpeedConfidence) {
        return (detectionParameters->maxPlausibleSpeed - currentSpeed + currentSpeedConfidence) /
               (2 * currentSpeedConfidence);
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

    CatchChecks::CatchChecks(shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                             DetectionParameters *detectionParameters, const vanetza::asn1::Cam &message) : BaseChecks(
            std::move(traciAPI), globalEnvironmentModel, detectionParameters, message) {

    }


} // namespace artery
