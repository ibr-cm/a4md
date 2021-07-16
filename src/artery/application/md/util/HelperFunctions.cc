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

    double calculateHeadingDifference(double heading1, double heading2){
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
        Position position = convertCamPosition(camParameters.basicContainer.referencePosition,
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

    Position convertCamPosition(const ReferencePosition_t &referencePosition, const traci::Boundary &simulationBoundary,
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
        if (!distanceIsFromCenter) {
            distance = distance - radius;
        }
        double angle = acos(std::min(distance / radius, 1.0)) * 2;
        double area = 0.5 * pow(radius, 2) * (angle - sin(angle));
        return PI * pow(radius, 2) - area;
    }

    double calculateNormedCircleAreaWithoutSegment(double radius, double distance, bool distanceIsFromCenter) {
        return calculateCircleAreaWithoutSegment(radius, distance, false) / (PI * pow(radius, 2));
    }

    std::vector<Position> createCircle(const Position &center, double radius, int pointCount) {
        boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(radius);
        boost::geometry::strategy::buffer::join_round join_strategy(pointCount);
        boost::geometry::strategy::buffer::end_round end_strategy(pointCount);
        boost::geometry::strategy::buffer::point_circle circle_strategy(pointCount);
        boost::geometry::strategy::buffer::side_straight side_strategy;
        boost::geometry::model::multi_polygon<polygon> circleBuffer;

        boost::geometry::buffer(center, circleBuffer,
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

    double intersectionFactor(const std::vector<Position> &polygon1, const std::vector<Position> &polygon2){
        double area1 = boost::geometry::area(polygon1);
        double area2 = boost::geometry::area(polygon2);

        double intersectionArea = getIntersectionArea(polygon1, polygon2);

        return (intersectionArea) / std::min(area1,area2);
    }

}
