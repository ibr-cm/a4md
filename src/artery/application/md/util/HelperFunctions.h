//
// Created by bastian on 18.06.21.
//

#ifndef ARTERY_HELPERFUNCTIONS_H
#define ARTERY_HELPERFUNCTIONS_H

#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/Geometry.h"
#include "artery/traci/VehicleController.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "vanetza/asn1/support/IA5String.h"
#include "vanetza/asn1/its/ReferencePosition.h"
#include "traci/API.h"
#include "traci/Boundary.h"

namespace artery {

    double calculateHeadingAngle(const Position &position);

    std::string ia5stringToString(IA5String_t ia5String);

    boost::geometry::strategy::transform::matrix_transformer<double, 2, 2>
    transformVehicle(double length, double width, const Position &pos, Angle alpha);

    std::vector<Position> getVehicleOutline(const VehicleDataProvider *vehicleDataProvider,
                                            const traci::VehicleController *vehicleController);

    std::vector<Position>
    getVehicleOutline(const Position &position, const Angle &heading, const double &length, const double &width);

    double getDistanceToNearestRoad(GlobalEnvironmentModel *globalEnvMod, const Position &position);

    Position getVector(const double &value, const double &angle);

    Position convertCamPosition(const ReferencePosition_t &referencePosition, const traci::Boundary &simulationBoundary,
                                const std::shared_ptr<const traci::API> &traciAPI);

}

#endif //ARTERY_HELPERFUNCTIONS_H
