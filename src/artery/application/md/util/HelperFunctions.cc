//
// Created by bastian on 18.06.21.
//

#include "HelperFunctions.h"
#include <artery/traci/Cast.h>
#include <omnetpp.h>

namespace artery{

    double calculateHeadingAngle(const Position &position) {
        double angle = atan2(-position.y.value(), position.x.value()) * 180 / PI;
        return angle;
    }

    std::string ia5stringToString (IA5String_t ia5String){
        char *tmp = new char[ia5String.size+1]();
        snprintf(tmp,ia5String.size+1,"%s",(char*) ia5String.buf);
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

    std::vector<Position> getVehicleOutline(const VehicleDataProvider *vehicleDataProvider, const traci::VehicleController *vehicleController) {
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

    double getDistanceToNearestRoad(GlobalEnvironmentModel *globalEnvMod, const Position &position){
        double minRoadDistance = 9999;
        std::vector<GeometryRtreeValue> laneResults;
        globalEnvMod->getLaneRTree()->query(
                boost::geometry::index::nearest(position, 10),
                std::back_inserter(laneResults));
        for (const auto &lResult : laneResults) {
            const auto &lane = globalEnvMod->getLane(lResult.second);
            minRoadDistance = std::min(minRoadDistance,boost::geometry::distance(position, lane->getShape()) - lane->getWidth() / 2);
        }
        std::vector<GeometryRtreeValue> junctionResults;
        globalEnvMod->getJunctionRTree()->query(
                boost::geometry::index::nearest(position, 3),
                std::back_inserter(junctionResults));
        for (const auto &jResult : junctionResults) {
            minRoadDistance = std::min(minRoadDistance,boost::geometry::distance(position,
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

    Position convertCamPosition(const ReferencePosition_t &referencePosition, const traci::Boundary& simulationBoundary, const std::shared_ptr<const traci::API>& traciAPI) {
        traci::TraCIGeoPosition traciGeoPositionSender = {
                (double) referencePosition.longitude / 10000000.0,
                (double) referencePosition.latitude / 10000000.0};
        return position_cast(simulationBoundary, traciAPI->convert2D(traciGeoPositionSender));
    }

}
