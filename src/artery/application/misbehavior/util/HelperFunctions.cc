//
// Created by bastian on 18.06.21.
//

#include "HelperFunctions.h"
#include <artery/traci/Cast.h>
#include <omnetpp.h>

namespace artery {

    typedef boost::geometry::model::d2::point_xy<double> point;
    typedef boost::geometry::model::polygon<point> polygon;

    double calculateHeadingAngle(const Position &position) {
        double angle = atan2(-position.y.value(), position.x.value()) * 180 / PI;
        angle = std::fmod(angle + 360, 360);
        return angle;
    }

    double calculateHeadingDifference(double heading1, double heading2) {
        double headingDiff = fabs(heading1 - heading2);
        headingDiff = std::min(headingDiff, 360 - heading1 + heading2);
        headingDiff = std::min(headingDiff, 360 - heading2 + heading1);
        return headingDiff;
    }

    std::string ia5stringToString(IA5String_t ia5String) {
        char *tmp = new char[ia5String.size + 1]();
        snprintf(tmp, ia5String.size + 1, "%s", (char *) ia5String.buf);
        return std::string(tmp);
    }

    boost::geometry::strategy::transform::matrix_transformer<double, 2, 2>
    transformVehicle(double length, double width, const Position &pos, Angle alpha) {
        using namespace boost::geometry::strategy::transform;

        // scale square to vehicle dimensions
        scale_transformer<double, 2, 2> scaling(length, width);
        // rotate into driving direction
        rotate_transformer<boost::geometry::radian, double, 2, 2> rotation(alpha.radian());
        // move to given front bumper position
        translate_transformer<double, 2, 2> translation(pos.x.value(), pos.y.value());

        return matrix_transformer<double, 2, 2>{translation.matrix() * rotation.matrix() * scaling.matrix()};
    }

    std::vector<Position> getVehicleOutline(const VehicleDataProvider *vehicleDataProvider,
                                            const traci::VehicleController *vehicleController) {
        Angle heading = -1.0 * (vehicleDataProvider->heading() -
                                0.5 * boost::math::double_constants::pi * boost::units::si::radian);
        auto transformationMatrix = transformVehicle(vehicleController->getVehicleType().getLength().value(),
                                                     vehicleController->getVehicleType().getWidth().value(),
                                                     vehicleDataProvider->position(),
                                                     heading);
        std::vector<Position> squareOutline = {
                Position(0.0, 0.5), // front left
                Position(0.0, -0.5), // front right
                Position(-1.0, -0.5), // back right
                Position(-1.0, 0.5) // back left
        };
        std::vector<Position> vehicleOutline;
        boost::geometry::transform(squareOutline, vehicleOutline, transformationMatrix);
        return vehicleOutline;
    }

    std::vector<Position>
    getVehicleOutline(const Position &position, const Angle &heading, const double &length, const double &width) {
        Angle alpha = -1.0 * (vanetza::units::Angle::from_value(heading.radian()) -
                              0.5 * boost::math::double_constants::pi * boost::units::si::radian);
        auto transformationMatrix = transformVehicle(length, width, position, alpha);
        std::vector<Position> squareOutline = {
                Position(0.0, 0.5), // front left
                Position(0.0, -0.5), // front right
                Position(-1.0, -0.5), // back right
                Position(-1.0, 0.5) // back left
        };
        std::vector<Position> vehicleOutline;
        boost::geometry::transform(squareOutline, vehicleOutline, transformationMatrix);
        return vehicleOutline;
    }

    std::vector<Position> getVehicleOutline(const vanetza::asn1::Cam &cam, const traci::Boundary &simulationBoundary,
                                            const std::shared_ptr<const traci::API> &traciAPI) {
        const CamParameters_t &camParameters = cam->cam.camParameters;
        const BasicVehicleContainerHighFrequency_t &hfc = camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        Position position = convertReferencePosition(camParameters.basicContainer.referencePosition,
                                                     simulationBoundary, traciAPI);
        double heading = (double) hfc.heading.headingValue / 10;
        double length = (double) hfc.vehicleLength.vehicleLengthValue / 10;
        double width = (double) hfc.vehicleWidth / 10;
        Angle alpha = -1.0 * (vanetza::units::Angle::from_value(heading * PI / 180) -
                              0.5 * boost::math::double_constants::pi * boost::units::si::radian);
        auto transformationMatrix = transformVehicle(length, width, position, alpha);
        std::vector<Position> squareOutline = {
                Position(0.0, 0.5), // front left
                Position(0.0, -0.5), // front right
                Position(-1.0, -0.5), // back right
                Position(-1.0, 0.5) // back left
        };
        std::vector<Position> vehicleOutline;
        boost::geometry::transform(squareOutline, vehicleOutline, transformationMatrix);
        return vehicleOutline;
    }

    double getDistanceToNearestRoad(GlobalEnvironmentModel *globalEnvMod, const Position &position) {
        double minRoadDistance = 9999;
        std::vector<GeometryRtreeValue> laneResults;
        globalEnvMod->getLaneRTree()->query(
                boost::geometry::index::nearest(position, 10),
                std::back_inserter(laneResults));
        for (const auto &lResult : laneResults) {
            const auto &lane = globalEnvMod->getLane(lResult.second);
            minRoadDistance = std::min(minRoadDistance,
                                       boost::geometry::distance(position, lane->getShape()) - lane->getWidth() / 2);
        }
        std::vector<GeometryRtreeValue> junctionResults;
        globalEnvMod->getJunctionRTree()->query(
                boost::geometry::index::nearest(position, 3),
                std::back_inserter(junctionResults));
        for (const auto &jResult : junctionResults) {
            minRoadDistance = std::min(minRoadDistance, boost::geometry::distance(position,
                                                                                  globalEnvMod->getJunction(
                                                                                          jResult.second)->getOutline()));
        }
        return minRoadDistance;
    }


    Position getVector(const double &value, const double &angle) {
        double x = value * sin(angle * PI / 180);
        double y = sqrt(pow(value, 2) - pow(x, 2));
        if (angle > 90 && angle < 270) {
            y *= -1;
        }
        return Position(x, y);
    }

    Position
    convertReferencePosition(const ReferencePosition_t &referencePosition, const traci::Boundary &simulationBoundary,
                             const std::shared_ptr<const traci::API> &traciAPI) {
        traci::TraCIGeoPosition traciGeoPositionSender = {
                (double) referencePosition.longitude / 10000000.0,
                (double) referencePosition.latitude / 10000000.0};
        return position_cast(simulationBoundary, traciAPI->convert2D(traciGeoPositionSender));
    }

    void drawTraciPolygon(std::vector<Position> outline, const std::string &id, const libsumo::TraCIColor &color,
                          const traci::Boundary &simulationBoundary,
                          const std::shared_ptr<const traci::API> &traciAPI) {
        try {
            traciAPI->polygon.remove(id, 5);
        } catch (libsumo::TraCIException &exception) {

        }
        libsumo::TraCIPositionVector traciOutline;
        for (const Position &p : outline) {
            traciOutline.value.emplace_back(position_cast(simulationBoundary, p));
        }
        traciOutline.value.emplace_back(position_cast(simulationBoundary, outline.front()));
        traciAPI->polygon.add(id, traciOutline, color, false, "helper", 5);
        traciAPI->polygon.setLineWidth(id, 1);
    }

    void drawTraciPoi(const Position &position, const std::string &id, const libsumo::TraCIColor &color,
                      const traci::Boundary &simulationBoundary,
                      const std::shared_ptr<const traci::API> &traciAPI) {
        try {
            traciAPI->poi.remove(id, 5);
        } catch (libsumo::TraCIException &exception) {

        }
        libsumo::TraCIPosition traCiPosition = position_cast(simulationBoundary, position);
        traciAPI->poi.add(id, traCiPosition.x, traCiPosition.y, color, id, 5, "", 0, 0, 0);
    }

    double calculateCircleAreaWithoutSegment(double radius, double distance, bool distanceIsFromCenter) {
        if (distance < 0) {
            return 0;
        }
        if (!distanceIsFromCenter) {
            distance = distance - radius;
        } else {
            distance = fabs(distance);
        }
        double angle = acos(std::min(distance / radius, 1.0)) * 2;
        double area = 0.5 * pow(radius, 2) * (angle - sin(angle));
        return PI * pow(radius, 2) - area;
    }

    double calculateNormedCircleAreaWithoutSegment(double radius, double distance, bool distanceIsFromCenter) {
        return calculateCircleAreaWithoutSegment(radius, distance, distanceIsFromCenter) / (PI * pow(radius, 2));
    }

    std::vector<Position> createCircle(const Position &center, double radius, int pointCount) {
        boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(radius);
        boost::geometry::strategy::buffer::join_round join_strategy(pointCount);
        boost::geometry::strategy::buffer::end_round end_strategy(pointCount);
        boost::geometry::strategy::buffer::point_circle circle_strategy(pointCount);
        boost::geometry::strategy::buffer::side_straight side_strategy;
        boost::geometry::model::multi_polygon<polygon> circleBuffer;

        point centerPoint(center.x.value(), center.y.value());
        boost::geometry::buffer(centerPoint, circleBuffer,
                                distance_strategy, side_strategy,
                                join_strategy, end_strategy, circle_strategy);
        std::vector<Position> circleOutline;
        polygon circle = circleBuffer.front();
        for (auto p : circle.outer()) {
            circleOutline.emplace_back(Position(p.x(), p.y()));
        }
        return circleOutline;
    }

    std::vector<Position>
    createEllipse(const Position &center, double semiMajorLength, double semiMinorLength, double semiMajorOrientation,
                  int pointCount) {
        std::vector<Position> ellipseOutline;
        for (int i = 0; i < pointCount; i++) {
            double borderX = cos(2 * PI / pointCount * i) * semiMajorLength;
            double borderY = sin(2 * PI / pointCount * i) * semiMinorLength;
            double newAngle = semiMajorOrientation + atan2(-borderY, borderX) * 180 / PI;
//        newAngle = 360 - std::fmod(newAngle, 360);
            newAngle = std::fmod(newAngle, 360);

            double offsetDistance = sqrt(pow(borderX, 2) + pow(borderY, 2));
            double relativeX = offsetDistance * sin(newAngle * PI / 180);
            double relativeY = offsetDistance * cos(newAngle * PI / 180);
            ellipseOutline.emplace_back(Position(center.x.value() + relativeX, center.y.value() + relativeY));
        }
        boost::geometry::correct(ellipseOutline);
        return ellipseOutline;
    }

    std::vector<Position> createEllipse(const Position &position, const PosConfidenceEllipse_t &posConfidenceEllipse) {
        double semiMajorConfidence = (double) posConfidenceEllipse.semiMajorConfidence / 100;
        double semiMinorConfidence = (double) posConfidenceEllipse.semiMinorConfidence / 100;
        double semiMajorOrientation = (double) posConfidenceEllipse.semiMajorOrientation / 10;
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

    double confidenceIntersectionArea(const Position &position1, const std::vector<Position> &ellipse1,
                                      const double &ellipse1Radius,
                                      const Position &position2, const std::vector<Position> &ellipse2,
                                      const double &ellipse2Radius,
                                      const double &range) {
        Position rangeCenter = Position(
                position1.x.value() + fabs(position2.x.value() - position1.x.value()) / 2,
                position1.y.value() + fabs(position2.y.value() - position1.y.value()) / 2);
        std::vector<Position> rangeCircle = createEllipse(rangeCenter, range, range, 0, 36);
        double distance = boost::geometry::distance(position1, position2);
        if (distance > range + ellipse1Radius + ellipse2Radius) {
            return 0;
        } else if (distance < range - ellipse1Radius - ellipse2Radius) {
            return 1;
        } else {
            double senderArea = boost::geometry::area(ellipse1);
            double receiverArea = boost::geometry::area(ellipse2);

            double senderRangeIntersectionArea = getIntersectionArea(ellipse1, rangeCircle);
            double receiverRangeIntersectionArea = getIntersectionArea(ellipse2, rangeCircle);

            return (senderRangeIntersectionArea + receiverRangeIntersectionArea) / (senderArea + receiverArea);
        }
    }

    double confidenceIntersectionArea(const Position &position1,
                                      const PosConfidenceEllipse_t &confidenceEllipse1,
                                      const Position &position2,
                                      const PosConfidenceEllipse_t &confidenceEllipse2, double range) {
        std::vector<Position> ellipse1 = createEllipse(position1, confidenceEllipse1);
        std::vector<Position> ellipse2 = createEllipse(position2, confidenceEllipse2);
        return confidenceIntersectionArea(position1, ellipse1, (double) confidenceEllipse1.semiMajorConfidence / 100,
                                          position2, ellipse2, (double) confidenceEllipse2.semiMajorConfidence / 100,
                                          range);
    }

    double intersectionFactor(const std::vector<Position> &polygon1, const std::vector<Position> &polygon2) {
        double area1 = boost::geometry::area(polygon1);
        double area2 = boost::geometry::area(polygon2);

        double intersectionArea = getIntersectionArea(polygon1, polygon2);

        return std::min((intersectionArea) / std::min(area1, area2), 1.0);
    }

    double oneSidedCircleSegmentFactor(double d, double r1, double r2, double range) {

        if (d < 0 || range > d + r1 + r2) {
            return 1;
        } else if (range < d - r1 - r2) {
            return 0;
        } else {
            double d1 = 0;
            double d2 = 0;
            if (d > 0) {
                d1 = (r1 * r1 + d * d - r2 * r2) / (2 * d);
                d2 = (r2 * r2 + d * d - r1 * r1) / (2 * d);
                if ((d1 + r1) < range / 2 && (d2 + r2) > range / 2) {
                    d2 = d2 - (range / 2 - (d1 + r1));
                    d1 = d1 + (range / 2 - (d1 + r1));
                }

                if ((d2 + r2) < range / 2 && (d1 + r1) > range / 2) {
                    d1 = d1 - (range / 2 - (d2 + r2));
                    d2 = d2 + (range / 2 - (d2 + r2));
                }
            }
            if (r1 <= 0) {
                if (range / 2 >= d1) {
                    double intD2 = (range / 2) - (d2 - r2);
                    double area2 = calculateCircleAreaWithoutSegment(r2, intD2, false);

                    double factor = (area2) / (PI * r2 * r2);
                    return factor;
                } else {
                    return 0;
                }
            } else if (r2 <= 0) {
                if (range / 2 >= d2) {
                    double intD1 = (range / 2) - (d1 - r1);
                    double area1 = calculateCircleAreaWithoutSegment(r1, intD1, false);

                    double factor = (area1) / (PI * r1 * r1);
                    return factor;
                } else {
                    return 0;
                }
            } else {

                double intD1 = (range / 2) - (d1 - r1);
                double intD2 = (range / 2) - (d2 - r2);

                double area1 = calculateCircleAreaWithoutSegment(r1, intD1, false);
                double area2 = calculateCircleAreaWithoutSegment(r2, intD2, false);

                double factor = (area1 + area2) / (PI * pow(r1, 2) + PI * pow(r2, 2));

                return factor;
            }
        }
    }

    void calculateMaxMinDist(double curSpeed, double oldspeed, double time,
                             double MAX_PLAUSIBLE_ACCEL, double MAX_PLAUSIBLE_DECEL,
                             double MAX_PLAUSIBLE_SPEED, double *returnDistance) {

        if (curSpeed < 0) {
            curSpeed = 0;
        }
        if (oldspeed < 0) {
            oldspeed = 0;
        }

        double deltaV = curSpeed - oldspeed;

        double T_1 = (deltaV + time * MAX_PLAUSIBLE_DECEL)
                     / (MAX_PLAUSIBLE_ACCEL + MAX_PLAUSIBLE_DECEL);
        double T_2 = time - T_1;

        double maxSpeed = MAX_PLAUSIBLE_ACCEL * T_1 + oldspeed;
        double maxDistance = 0;
        if (maxSpeed > MAX_PLAUSIBLE_SPEED) {
            double newT_1 = (MAX_PLAUSIBLE_SPEED - oldspeed)
                            / MAX_PLAUSIBLE_ACCEL;
            double newT_2 = (MAX_PLAUSIBLE_SPEED - curSpeed)
                            / MAX_PLAUSIBLE_DECEL;
            maxDistance = oldspeed * newT_1
                          + 0.5 * MAX_PLAUSIBLE_ACCEL * newT_1 * newT_1
                          + maxSpeed * newT_2
                          - 0.5 * MAX_PLAUSIBLE_DECEL * newT_2 * newT_2
                          + MAX_PLAUSIBLE_SPEED * (time - newT_1 - newT_2);
        } else {
            maxDistance = oldspeed * T_1 + 0.5 * MAX_PLAUSIBLE_ACCEL * T_1 * T_1
                          + maxSpeed * T_2 - 0.5 * MAX_PLAUSIBLE_DECEL * T_2 * T_2;
        }

        double minSpeed = -MAX_PLAUSIBLE_DECEL * T_2 + oldspeed;
        double minDistance = 0;
        if (minSpeed < 0) {
            double newT_1 = curSpeed / MAX_PLAUSIBLE_ACCEL;
            double newT_2 = oldspeed / MAX_PLAUSIBLE_DECEL;

            minDistance = oldspeed * newT_2
                          - 0.5 * MAX_PLAUSIBLE_DECEL * newT_2 * newT_2
                          + 0.5 * MAX_PLAUSIBLE_ACCEL * newT_1 * newT_1;
        } else {
            minDistance = oldspeed * T_2 - 0.5 * MAX_PLAUSIBLE_DECEL * T_2 * T_2
                          + minSpeed * T_1 + 0.5 * MAX_PLAUSIBLE_ACCEL * T_1 * T_1;
        }

        returnDistance[0] = minDistance;
        returnDistance[1] = maxDistance;
    }

    double segmentSegmentFactor(double d, double r1, double r2, double range) {
        if (r1 == 0 && r2 == 0) {
            if (d > range) {
                return 0;
            } else {
                return 1;
            }
        }
        double d1 = 0;
        double d2 = 0;
        if (d > 0) {
            d1 = (r1 * r1 + d * d - r2 * r2) / (2 * d);
            d2 = (r2 * r2 + d * d - r1 * r1) / (2 * d);

            if ((d1 + r1) < range / 2 && (d2 + r2) > range / 2) {
                d2 = d2 - (range / 2 - (d1 + r1));
                d1 = d1 + (range / 2 - (d1 + r1));
            }

            if ((d2 + r2) < range / 2 && (d1 + r1) > range / 2) {
                d1 = d1 - (range / 2 - (d2 + r2));
                d2 = d2 + (range / 2 - (d2 + r2));
            }
        }

        double overlap1 = 0;
        double overlap2 = 0;

        double addon = 0;

        if ((d1 - range / 2) < r1) {
            if ((d1 - range / 2) > -r1) {
                addon = -(d1 - r1);
                overlap1 = range / 2 + addon;
            } else {
                overlap1 = 2 * r1;
            }
        }
        if ((d2 - range / 2) < r2) {
            if ((d2 - range / 2) > -r2) {
                addon = -(d2 - r2);
                overlap2 = range / 2 + addon;
            } else {
                overlap2 = 2 * r2;
            }
        }
        return (overlap1 + overlap2) / (2 * r1 + 2 * r2);
    }

    void setPositionWithJitter(ReferencePosition_t &referencePosition, const Position &originalPosition,
                               const traci::Boundary &simulationBoundary,
                               const std::shared_ptr<const traci::API> &traciAPI, omnetpp::cRNG *rng) {
        Position newPosition;
        if (referencePosition.positionConfidenceEllipse.semiMajorConfidence == SemiAxisLength_unavailable ||
            referencePosition.positionConfidenceEllipse.semiMinorConfidence == SemiAxisLength_unavailable) {
            newPosition = originalPosition;
        } else {
            double phi = uniform(rng, 0, 2 * PI);
            double rho = sqrt(uniform(rng, 0, 1));
            double offsetX =
                    rho * cos(phi) * (double) referencePosition.positionConfidenceEllipse.semiMajorConfidence / 200;
            double offsetY =
                    rho * sin(phi) * (double) referencePosition.positionConfidenceEllipse.semiMinorConfidence / 200;
            double newAngle = (double) referencePosition.positionConfidenceEllipse.semiMajorOrientation / 10 +
                              calculateHeadingAngle(Position(offsetX, offsetY));
            newAngle = 360 - std::fmod(newAngle, 360);

            double offsetDistance = sqrt(pow(offsetX, 2) + pow(offsetY, 2));
            double relativeX = offsetDistance * sin(newAngle * PI / 180);
            double relativeY = offsetDistance * cos(newAngle * PI / 180);
            newPosition = Position(originalPosition.x.value() + relativeX,
                                   originalPosition.y.value() + relativeY);
        }
        traci::TraCIGeoPosition traciGeoPos = traciAPI->convertGeo(
                position_cast(simulationBoundary, newPosition));
        referencePosition.longitude = (long) (traciGeoPos.longitude * 10000000);
        referencePosition.latitude = (long) (traciGeoPos.latitude * 10000000);
    }

    void setPositionWithJitter(ReferencePosition_t &referencePosition, const Position &originalPosition,
                               const HeadingValue_t &heading, const HeadingValue_t &semiMajorOrientationOffset,
                               const traci::Boundary &simulationBoundary,
                               const std::shared_ptr<const traci::API> &traciAPI, omnetpp::cRNG *rng) {
        Position newPosition;
        if (referencePosition.positionConfidenceEllipse.semiMajorConfidence == SemiAxisLength_unavailable ||
            referencePosition.positionConfidenceEllipse.semiMinorConfidence == SemiAxisLength_unavailable) {
            newPosition = originalPosition;
        } else {
            referencePosition.positionConfidenceEllipse.semiMajorOrientation =
                    (heading + semiMajorOrientationOffset) % 3600;

            double phi = uniform(rng, 0, 2 * PI);
            double rho = sqrt(uniform(rng, 0, 1));
            double offsetX =
                    rho * cos(phi) * (double) referencePosition.positionConfidenceEllipse.semiMajorConfidence / 200;
            double offsetY =
                    rho * sin(phi) * (double) referencePosition.positionConfidenceEllipse.semiMinorConfidence / 200;
            double newAngle = (double) referencePosition.positionConfidenceEllipse.semiMajorOrientation / 10 +
                              calculateHeadingAngle(Position(offsetX, offsetY));
            newAngle = 360 - std::fmod(newAngle, 360);

            double offsetDistance = sqrt(pow(offsetX, 2) + pow(offsetY, 2));
            double relativeX = offsetDistance * sin(newAngle * PI / 180);
            double relativeY = offsetDistance * cos(newAngle * PI / 180);
            newPosition = Position(originalPosition.x.value() + relativeX,
                                   originalPosition.y.value() + relativeY);
        }
        traci::TraCIGeoPosition traciGeoPos = traciAPI->convertGeo(
                position_cast(simulationBoundary, newPosition));
        referencePosition.longitude = (long) (traciGeoPos.longitude * 10000000);
        referencePosition.latitude = (long) (traciGeoPos.latitude * 10000000);
    }


    bool camComp(const vanetza::asn1::Cam &message1, const vanetza::asn1::Cam &message2) {
        {
            ItsPduHeader header1 = message1->header;
            ItsPduHeader header2 = message2->header;
            if (header1.protocolVersion != header2.protocolVersion) {
                return header1.protocolVersion < header2.protocolVersion;
            } else if (header1.messageID != header2.messageID) {
                return header1.messageID < header2.messageID;
            } else if (header1.stationID != header2.stationID) {
                return header1.stationID < header2.stationID;
            }
        }
        {
            CoopAwareness cam1 = message1->cam;
            CoopAwareness cam2 = message2->cam;
            if (cam1.generationDeltaTime != cam2.generationDeltaTime) {
                return cam1.generationDeltaTime < cam2.generationDeltaTime;
            }
            {
                BasicContainer bc1 = cam1.camParameters.basicContainer;
                BasicContainer bc2 = cam2.camParameters.basicContainer;
                if (bc1.stationType != bc2.stationType) {
                    return bc1.stationType < bc2.stationType;
                }
                {
                    ReferencePosition referencePosition1 = bc1.referencePosition;
                    ReferencePosition referencePosition2 = bc2.referencePosition;
                    if (referencePosition1.longitude != referencePosition2.longitude) {
                        return referencePosition1.longitude < referencePosition2.longitude;
                    } else if (referencePosition1.latitude != referencePosition2.latitude) {
                        return referencePosition1.latitude < referencePosition2.latitude;
                    }
                    {
                        Altitude altitude1 = referencePosition1.altitude;
                        Altitude altitude2 = referencePosition2.altitude;
                        if (altitude1.altitudeConfidence != altitude2.altitudeConfidence) {
                            return altitude1.altitudeConfidence < altitude2.altitudeConfidence;
                        } else if (altitude1.altitudeValue != altitude2.altitudeValue) {
                            return altitude1.altitudeValue < altitude2.altitudeValue;
                        }
                    }
                    {
                        PosConfidenceEllipse posConfidenceEllipse1 = referencePosition1.positionConfidenceEllipse;
                        PosConfidenceEllipse posConfidenceEllipse2 = referencePosition2.positionConfidenceEllipse;
                        if (posConfidenceEllipse1.semiMajorConfidence != posConfidenceEllipse2.semiMajorConfidence) {
                            return posConfidenceEllipse1.semiMajorConfidence <
                                   posConfidenceEllipse2.semiMajorConfidence;
                        } else if (
                                posConfidenceEllipse1.semiMinorConfidence !=
                                posConfidenceEllipse2.semiMinorConfidence) {
                            return posConfidenceEllipse1.semiMinorConfidence <
                                   posConfidenceEllipse2.semiMinorConfidence;
                        } else if (
                                posConfidenceEllipse1.semiMajorOrientation !=
                                posConfidenceEllipse2.semiMajorOrientation) {
                            return posConfidenceEllipse1.semiMajorOrientation <
                                   posConfidenceEllipse2.semiMajorOrientation;
                        }
                    }
                }
            }
            HighFrequencyContainer highFrequencyContainer1 = cam1.camParameters.highFrequencyContainer;
            HighFrequencyContainer highFrequencyContainer2 = cam2.camParameters.highFrequencyContainer;
            if (highFrequencyContainer1.present != highFrequencyContainer2.present) {
                return highFrequencyContainer1.present < highFrequencyContainer2.present;
            }
            if (cam1.camParameters.highFrequencyContainer.present ==
                HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
                BasicVehicleContainerHighFrequency hfc1 = cam1.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
                BasicVehicleContainerHighFrequency hfc2 = cam2.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
                if (hfc1.vehicleWidth != hfc2.vehicleWidth) {
                    return hfc1.vehicleWidth < hfc2.vehicleWidth;
                } else if (hfc1.driveDirection != hfc2.driveDirection) {
                    return hfc1.driveDirection < hfc2.driveDirection;
                } else if (hfc1.curvatureCalculationMode != hfc2.curvatureCalculationMode) {
                    return hfc1.curvatureCalculationMode < hfc2.curvatureCalculationMode;
                }
                {
                    Speed speed1 = hfc1.speed;
                    Speed speed2 = hfc2.speed;
                    if (speed1.speedValue != speed2.speedValue) {
                        return speed1.speedValue < speed2.speedValue;
                    } else if (speed1.speedConfidence != speed2.speedConfidence) {
                        return speed1.speedConfidence < speed2.speedConfidence;
                    }
                }
                {
                    Heading heading1 = hfc1.heading;
                    Heading heading2 = hfc2.heading;
                    if (heading1.headingValue != heading2.headingValue) {
                        return heading1.headingValue < heading2.headingValue;
                    } else if (heading1.headingConfidence != heading2.headingConfidence) {
                        return heading1.headingConfidence < heading2.headingConfidence;
                    }
                }
                {
                    VehicleLength vehicleLength1 = hfc1.vehicleLength;
                    VehicleLength vehicleLength2 = hfc2.vehicleLength;
                    if (vehicleLength1.vehicleLengthValue != vehicleLength2.vehicleLengthValue) {
                        return vehicleLength1.vehicleLengthValue < vehicleLength2.vehicleLengthValue;
                    } else if (vehicleLength1.vehicleLengthConfidenceIndication !=
                               vehicleLength2.vehicleLengthConfidenceIndication) {
                        return vehicleLength1.vehicleLengthConfidenceIndication <
                               vehicleLength2.vehicleLengthConfidenceIndication;
                    }
                }
                {
                    LongitudinalAcceleration la1 = hfc1.longitudinalAcceleration;
                    LongitudinalAcceleration la2 = hfc2.longitudinalAcceleration;
                    if (la1.longitudinalAccelerationValue != la2.longitudinalAccelerationValue) {
                        return la1.longitudinalAccelerationValue < la2.longitudinalAccelerationValue;
                    } else if (la1.longitudinalAccelerationConfidence != la2.longitudinalAccelerationConfidence) {
                        return la1.longitudinalAccelerationConfidence < la2.longitudinalAccelerationConfidence;
                    }
                }
                {
                    Curvature curvature1 = hfc1.curvature;
                    Curvature curvature2 = hfc2.curvature;
                    if (curvature1.curvatureValue != curvature2.curvatureValue) {
                        return curvature1.curvatureValue < curvature2.curvatureValue;
                    } else if (curvature1.curvatureConfidence != curvature2.curvatureConfidence) {
                        return curvature1.curvatureConfidence < curvature2.curvatureConfidence;
                    }
                }
                {
                    YawRate yawRate1 = hfc1.yawRate;
                    YawRate yawRate2 = hfc2.yawRate;
                    if (yawRate1.yawRateValue != yawRate2.yawRateValue) {
                        return yawRate1.yawRateValue < yawRate2.yawRateValue;
                    } else if (yawRate1.yawRateConfidence != yawRate2.yawRateConfidence) {
                        return yawRate1.yawRateConfidence < yawRate2.yawRateConfidence;
                    }
                }
                {
                    AccelerationControl_t *ac1 = hfc1.accelerationControl;
                    AccelerationControl_t *ac2 = hfc2.accelerationControl;
                    if (ac1 == nullptr && ac2 != nullptr ||
                        ac1 != nullptr && ac2 == nullptr) {
                        return ac1 < ac2;
                    }
                    if (ac1 != nullptr) {
                        if (ac1->buf != ac2->buf) {
                            return ac1->buf < ac2->buf;
                        }
                    }
                }
                {
                    LanePosition_t *lanePosition1 = hfc1.lanePosition;
                    LanePosition_t *lanePosition2 = hfc2.lanePosition;
                    if (lanePosition1 == nullptr && lanePosition2 != nullptr ||
                        lanePosition1 != nullptr && lanePosition2 == nullptr) {
                        return lanePosition1 < lanePosition2;
                    }
                    if (lanePosition1 != nullptr) {
                        if (*lanePosition1 != *lanePosition2) {
                            return *lanePosition1 < *lanePosition2;
                        }
                    }
                }
                {
                    SteeringWheelAngle *swa1 = hfc1.steeringWheelAngle;
                    SteeringWheelAngle *swa2 = hfc2.steeringWheelAngle;
                    if (swa1 == nullptr && swa2 != nullptr ||
                        swa1 != nullptr && swa2 == nullptr) {
                        return swa1 < swa2;
                    }
                    if (swa1 != nullptr) {
                        if (swa1->steeringWheelAngleConfidence != swa2->steeringWheelAngleConfidence) {
                            return swa1->steeringWheelAngleConfidence < swa2->steeringWheelAngleConfidence;
                        } else if (swa1->steeringWheelAngleValue != swa2->steeringWheelAngleValue) {
                            return swa1->steeringWheelAngleValue < swa2->steeringWheelAngleValue;
                        }
                    }
                }
                {
                    LateralAcceleration *la1 = hfc1.lateralAcceleration;
                    LateralAcceleration *la2 = hfc2.lateralAcceleration;
                    if (la1 == nullptr && la2 != nullptr ||
                        la1 != nullptr && la2 == nullptr) {
                        return la1 < la2;
                    }
                    if (la1 != nullptr) {
                        if (la1->lateralAccelerationConfidence != la2->lateralAccelerationConfidence) {
                            return la1->lateralAccelerationConfidence < la2->lateralAccelerationConfidence;
                        } else if (la1->lateralAccelerationValue != la2->lateralAccelerationValue) {
                            return la1->lateralAccelerationValue < la2->lateralAccelerationValue;
                        }
                    }
                }
                {
                    VerticalAcceleration *va1 = hfc1.verticalAcceleration;
                    VerticalAcceleration *va2 = hfc2.verticalAcceleration;
                    if (va1 == nullptr && va2 != nullptr ||
                        va1 != nullptr && va2 == nullptr) {
                        return va1 < va2;
                    }
                    if (va1 != nullptr) {
                        if (va1->verticalAccelerationConfidence != va2->verticalAccelerationConfidence) {
                            return va1->verticalAccelerationConfidence < va2->verticalAccelerationConfidence;
                        } else if (va1->verticalAccelerationValue != va2->verticalAccelerationValue) {
                            return va1->verticalAccelerationValue < va2->verticalAccelerationValue;
                        }
                    }
                }
                {
                    PerformanceClass_t *pc1 = hfc1.performanceClass;
                    PerformanceClass_t *pc2 = hfc2.performanceClass;
                    if (pc1 == nullptr && pc2 != nullptr ||
                        pc1 != nullptr && pc2 == nullptr) {
                        return pc1 < pc2;
                    }
                    if (pc1 != nullptr) {
                        if (*pc1 != *pc2) {
                            return *pc1 < *pc2;
                        }
                    }
                }
                {
                    CenDsrcTollingZone *cdtz1 = hfc1.cenDsrcTollingZone;
                    CenDsrcTollingZone *cdtz2 = hfc2.cenDsrcTollingZone;
                    if (cdtz1 == nullptr && cdtz2 != nullptr ||
                        cdtz1 != nullptr && cdtz2 == nullptr) {
                        return cdtz1 < cdtz2;
                    }
                    if (cdtz1 != nullptr) {
                        if (cdtz1->protectedZoneLatitude != cdtz2->protectedZoneLatitude) {
                            return cdtz1->protectedZoneLatitude < cdtz2->protectedZoneLatitude;
                        } else if (cdtz1->protectedZoneLongitude != cdtz2->protectedZoneLongitude) {
                            return cdtz1->protectedZoneLongitude < cdtz2->protectedZoneLongitude;
                        }
                        {
                            CenDsrcTollingZoneID_t *cid1 = cdtz1->cenDsrcTollingZoneID;
                            CenDsrcTollingZoneID_t *cid2 = cdtz2->cenDsrcTollingZoneID;
                            if (cid1 == nullptr && cid2 != nullptr ||
                                cid1 != nullptr && cid2 == nullptr) {
                                return cid1 < cid2;
                            }
                            if (*cid1 != *cid2) {
                                return *cid1 < *cid2;
                            }
                        }
                    }
                }
            }
            {
                LowFrequencyContainer *lowFrequencyContainer1 = cam1.camParameters.lowFrequencyContainer;
                LowFrequencyContainer *lowFrequencyContainer2 = cam2.camParameters.lowFrequencyContainer;
                if (lowFrequencyContainer1 == nullptr && lowFrequencyContainer2 != nullptr ||
                    lowFrequencyContainer1 != nullptr && lowFrequencyContainer2 == nullptr) {
                    return lowFrequencyContainer1 < lowFrequencyContainer2;
                }
                if (lowFrequencyContainer1 != nullptr) {
                    if (lowFrequencyContainer1->present != lowFrequencyContainer2->present) {
                        return lowFrequencyContainer1->present < lowFrequencyContainer2->present;
                    }
                    if (lowFrequencyContainer1->present == LowFrequencyContainer_PR_basicVehicleContainerLowFrequency) {
                        BasicVehicleContainerLowFrequency_t lfc1 = lowFrequencyContainer1->choice.basicVehicleContainerLowFrequency;
                        BasicVehicleContainerLowFrequency_t lfc2 = lowFrequencyContainer2->choice.basicVehicleContainerLowFrequency;
                        if (lfc1.vehicleRole != lfc2.vehicleRole ||
                            *lfc1.exteriorLights.buf != *lfc2.exteriorLights.buf) {
                            return *lfc1.exteriorLights.buf < *lfc2.exteriorLights.buf;
                        }
                        if (lfc1.pathHistory.list.count != lfc2.pathHistory.list.count) {
                            return lfc1.pathHistory.list.count < lfc2.pathHistory.list.count;
                        }
                        for (int i = 0; i < lfc1.pathHistory.list.count; i++) {
                            PathPoint *pathPoint1 = lfc1.pathHistory.list.array[i];
                            PathPoint *pathPoint2 = lfc1.pathHistory.list.array[i];
                            {
                                DeltaReferencePosition drp1 = pathPoint1->pathPosition;
                                DeltaReferencePosition drp2 = pathPoint2->pathPosition;
                                if (drp1.deltaAltitude != drp2.deltaAltitude) {
                                    return drp1.deltaAltitude < drp2.deltaAltitude;
                                } else if (drp1.deltaLatitude != drp2.deltaLatitude) {
                                    return drp1.deltaLatitude < drp2.deltaLatitude;
                                } else if (drp1.deltaLongitude != drp2.deltaLongitude) {
                                    return drp1.deltaLongitude < drp2.deltaLongitude;
                                }
                            }
                            {
                                PathDeltaTime_t *pathDeltaTime1 = pathPoint1->pathDeltaTime;
                                PathDeltaTime_t *pathDeltaTime2 = pathPoint2->pathDeltaTime;
                                if (pathDeltaTime1 == nullptr && pathDeltaTime2 != nullptr ||
                                    pathDeltaTime1 != nullptr && pathDeltaTime2 == nullptr) {
                                    return pathDeltaTime1 < pathDeltaTime2;
                                }
                                if (*pathDeltaTime1 != *pathDeltaTime2) {
                                    return *pathDeltaTime1 < *pathDeltaTime2;
                                }
                            }
                        }
                    }
                }
            }
            {
                SpecialVehicleContainer *specialVehicleContainer1 = cam1.camParameters.specialVehicleContainer;
                SpecialVehicleContainer *specialVehicleContainer2 = cam2.camParameters.specialVehicleContainer;
                if (specialVehicleContainer1 == nullptr && specialVehicleContainer2 != nullptr ||
                    specialVehicleContainer1 != nullptr && specialVehicleContainer2 == nullptr) {
                    return specialVehicleContainer1 < specialVehicleContainer2;
                }
                if (specialVehicleContainer1 != nullptr) {
                    if (specialVehicleContainer1->present != specialVehicleContainer2->present) {
                        return specialVehicleContainer1->present < specialVehicleContainer2->present;
                    }
                }
            }
        }
        return false;
    }

    bool camCompPtr(const std::shared_ptr<vanetza::asn1::Cam> &ptr1, const std::shared_ptr<vanetza::asn1::Cam> &ptr2){
        return camComp((*ptr1),(*ptr2));
    }

    bool camEquiv(const vanetza::asn1::Cam &message1, const vanetza::asn1::Cam &message2){
        return !camComp(message1,message2) && !camComp(message1,message2);
    }


    double normalizeValue(double value, double min, double max) {
        return (value - min) / (max - min);
    }

    std::vector<std::string> split(const std::string &s, char delimiter) {
        std::vector<std::string> result;
        std::stringstream ss(s);
        std::string item;

        while (getline(ss, item, delimiter)) {
            result.push_back(item);
        }

        return result;
    }


}
