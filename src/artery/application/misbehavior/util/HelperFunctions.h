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
#include <vanetza/asn1/cam.hpp>
#include "vanetza/asn1/its/ReferencePosition.h"
#include "traci/API.h"
#include "traci/Boundary.h"

namespace artery {


    double calculateHeadingAngle(const Position &position);

    double calculateHeadingDifference(double heading1, double heading2);

    std::string ia5stringToString(IA5String_t ia5String);

    boost::geometry::strategy::transform::matrix_transformer<double, 2, 2>
    transformVehicle(double length, double width, const Position &pos, Angle alpha);

    std::vector<Position> getVehicleOutline(const VehicleDataProvider *vehicleDataProvider,
                                            const traci::VehicleController *vehicleController);

    std::vector<Position>
    getVehicleOutline(const Position &position, const Angle &heading, const double &length, const double &width);

    std::vector<Position> getVehicleOutline(const vanetza::asn1::Cam &cam, const traci::Boundary &simulationBoundary,
                                            const std::shared_ptr<const traci::API> &traciAPI);

    double getDistanceToNearestRoad(GlobalEnvironmentModel *globalEnvMod, const Position &position);

    Position getVector(const double &value, const double &angle);

    Position convertReferencePosition(const ReferencePosition_t &referencePosition, const traci::Boundary &simulationBoundary,
                                      const std::shared_ptr<const traci::API> &traciAPI);

    void drawTraciPolygon(std::vector<Position> outline, const std::string &id, const libsumo::TraCIColor &color,
                          const traci::Boundary &simulationBoundary,
                          const std::shared_ptr<const traci::API> &traciAPI);

    void drawTraciPoi(const Position &position, const std::string &id, const libsumo::TraCIColor &color,
                      const traci::Boundary &simulationBoundary, const std::shared_ptr<const traci::API> &traciAPI);

    double calculateCircleAreaWithoutSegment(double radius, double distance, bool distanceIsFromCenter);

    double calculateNormedCircleAreaWithoutSegment(double radius, double distance, bool distanceIsFromCenter);

    std::vector<Position> createCircle(const Position &center, double radius, int pointCount);

    std::vector<Position>
    createEllipse(const Position &center, double semiMajorLength, double semiMinorLength, double semiMajorOrientation,
                  int pointCount);

    std::vector<Position> createEllipse(const Position &position, const PosConfidenceEllipse_t &posConfidenceEllipse);

    double getIntersectionArea(const std::vector<Position> &polygon1, const std::vector<Position> &polygon2);

    double confidenceIntersectionArea(const Position &position1, const std::vector<Position> &ellipse1,
                                      const double &ellipse1Radius,
                                      const Position &position2, const std::vector<Position> &ellipse2,
                                      const double &ellipse2Radius,
                                      const double &range);

    double confidenceIntersectionArea(const Position &position1,
                                      const PosConfidenceEllipse_t &confidenceEllipse1,
                                      const Position &position2,
                                      const PosConfidenceEllipse_t &confidenceEllipse2, double range);

    double intersectionFactor(const std::vector<Position> &polygon1, const std::vector<Position> &polygon2);

    double oneSidedCircleSegmentFactor(double d, double r1, double r2, double range);

    void calculateMaxMinDist(double curSpeed, double oldspeed, double time,
                        double MAX_PLAUSIBLE_ACCEL, double MAX_PLAUSIBLE_DECEL,
                        double MAX_PLAUSIBLE_SPEED, double *returnDistance);

    double segmentSegmentFactor(double d, double r1, double r2, double range);

    void setPositionWithJitter(ReferencePosition_t &referencePosition, const Position &originalPosition,
                               const traci::Boundary &simulationBoundary,
                               const std::shared_ptr<const traci::API> &traciAPI, omnetpp::cRNG *rng);

    void setPositionWithJitter(ReferencePosition_t &referencePosition, const Position &originalPosition,
                               const HeadingValue_t &heading, const HeadingValue_t &semiMajorOrientationOffset,
                               const traci::Boundary &simulationBoundary,
                               const std::shared_ptr<const traci::API> &traciAPI, omnetpp::cRNG *rng);

    bool camComp(const vanetza::asn1::Cam &message1, const vanetza::asn1::Cam &message2);

    bool camCompPtr(const std::shared_ptr<vanetza::asn1::Cam> &ptr1, const std::shared_ptr<vanetza::asn1::Cam> &ptr2);

    bool camEquiv(const vanetza::asn1::Cam &message1, const vanetza::asn1::Cam &message2);

    struct CamCompare {
        bool operator()(const std::shared_ptr<vanetza::asn1::Cam> &ptr1,
                        const std::shared_ptr<vanetza::asn1::Cam> &ptr2) {
            return camCompPtr(ptr1, ptr2);
        }
    };

    double normalizeValue(double value, double min, double max);

    std::vector<std::string> split(const std::string &s, char delimiter);

    std::string generateReportId(const StationID_t &senderStationId,
                                 const StationID_t &receiverStationId,
                                 omnetpp::cRNG *rng);
}

#endif //ARTERY_HELPERFUNCTIONS_H
