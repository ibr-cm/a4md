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

    std::string ia5stringToString (IA5String_t ia5String){
        char *tmp = new char[ia5String.size+1]();
        snprintf(tmp,ia5String.size+1,"%s",(char*) ia5String.buf);
        return std::string(tmp);
    }

}
