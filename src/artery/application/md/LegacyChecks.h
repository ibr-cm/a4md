#ifndef ARTERY_LEGACYCHECK_H_
#define ARTERY_LEGACYCHECK_H_
#include "artery/application/md/CheckResult.h"
#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include "veins/base/utils/Coord.h"
namespace artery
{

    class LegacyChecks
    {
    public:
        LegacyChecks(double maxPlausibleSpeed, double maxPlausibleAcceleration, double MaxPlausibleDeceleration);
        CheckResult checkCAM(const vanetza::asn1::Cam);

    private:
        double maxPlausibleSpeed;
        double maxPlausibleAcceleration;
        double MaxPlausibleDeceleration;

        double maxProximityRangeL;
        double maxProximityRangeW;
        double maxPlausibleRange;

        double calculateHeadingAngle(const veins::Coord *position);
        double calculateDistance2D(const veins::Coord *position1, const veins::Coord *position2);

        // double checkProximityPlausibility(const veins::Coord *referencePosition,const veins::Coord *testPosition);
        double checkRangePlausibility(const veins::Coord *receiverPosition, const veins::Coord *senderPosition);
        double checkPositionConsistency(const veins::Coord *oldPosition, const veins::Coord *newPosition, double deltaTime);
        double checkSpeedConsistency(double oldSpeed, double newSpeed, double deltaTime);
        double checkPositionSpeedConsistency(const veins::Coord *oldPosition, const veins::Coord *newPosition, double oldSpeed, double newSpeed, double deltaTime);
        double checkPositionSpeedMaxConsistency(const veins::Coord *oldPosition, const veins::Coord *newPosition, double oldSpeed, double newSpeed, double deltaTime);
        double checkIntersection(veins::Coord nodePosition1, veins::Coord nodeSize1, veins::Coord head1, veins::Coord nodePosition2, veins::Coord head2, double deltaTime);

        double checkSpeedPlausibility(double speed);
        double checkPositionPlausibility(const veins::Coord *position);
    };
} // namespace artery
#endif