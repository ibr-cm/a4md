#include "artery/application/md/LegacyChecks.h"

namespace artery
{

    using namespace omnetpp;
    double LegacyChecks::calculateHeadingAngle(const veins::Coord *position)
    {
        double angle = atan2(-position->y, position->x) * 180 / PI;
        // if (angle < 0)
        // {
        //     angle += 360;
        // }
        return angle;
    }

    double LegacyChecks::calculateDistance2D(const veins::Coord *position1, const veins::Coord *position2)
    {
        return sqrt(pow(position1->x - position2->x, 2.0) + pow(position1->y - position2->y, 2.0));
    }

    // double LegacyChecks::checkProximityPlausibility(const veins::Coord *receiverPosition, const veins::Coord *senderPosition)
    // {
    //     double deltaDistance = calculateDistance2D(receiverPosition, senderPosition);
    //     double deltaPosition = veins::Coord(
    //         senderPosition->x - receiverPosition->x,
    //         senderPosition->y - receiverPosition->y,
    //         senderPosition->z - receiverPosition->z, );
    //     double deltaAngle = calculateHeadingAngle(deltaPosition);

    //     if (deltaDistance < maxProximityRangeL)
    //     {
    //         if (deltaDistance < maxProximityRangeW * 2 || (deltaAngle < 90 && distance < (maxProximityRangeW / cos((90 - deltaAngle) * PI / 180))))
    //         {
    //         }
    //     }
    // }

    double LegacyChecks::checkRangePlausibility(const veins::Coord *receiverPosition, const veins::Coord *senderPosition){
        if(calculateDistance2D(senderPosition,receiverPosition) > maxPlausibleRange){
            return 0;
        } else {
            return 1;
        }
    }

    double LegacyChecks::checkSpeedPlausibility(double speed){
        if(fabs(speed) > maxPlausibleSpeed){
            return 0;
        } else {
            return 1;
        }
    }

}