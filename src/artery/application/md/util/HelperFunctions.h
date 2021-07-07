//
// Created by bastian on 18.06.21.
//

#ifndef ARTERY_HELPERFUNCTIONS_H
#define ARTERY_HELPERFUNCTIONS_H

#include "artery/utility/Geometry.h"
#include "vanetza/asn1/support/IA5String.h"

namespace artery{

    double calculateHeadingAngle(const Position &position);

    std::string ia5stringToString (IA5String_t ia5String);
}

#endif //ARTERY_HELPERFUNCTIONS_H
