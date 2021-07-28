//
// Created by bastian on 07.07.21.
//

#ifndef ARTERY_BASEFUSION_H
#define ARTERY_BASEFUSION_H

#include "artery/application/misbehavior/checks/CheckResult.h"
#include "vanetza/asn1/md/DetectionReferenceCAM.h"
#include <bitset>

namespace artery {

    class BaseFusion {
    public:
        BaseFusion() = default;
        virtual std::vector<std::bitset<16>> checkForReport(const CheckResult &checkResult) = 0;
    };

} // namespace artery

#endif //ARTERY_BASEFUSION_H
