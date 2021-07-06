//
// Created by bastian on 18.06.21.
//

#include "HelperFunctions.h"
#include <omnetpp.h>

namespace artery{

    double calculateHeadingAngle(const Position &position) {
        double angle = atan2(-position.y.value(), position.x.value()) * 180 / PI;
        return angle;
    }

}
