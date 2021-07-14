//
// Created by bastian on 18.06.21.
//

#include "HelperFunctions.h"
#include <artery/traci/Cast.h>
#include <omnetpp.h>

namespace artery {

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

}
