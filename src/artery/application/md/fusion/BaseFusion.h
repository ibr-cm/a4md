//
// Created by bastian on 07.07.21.
//

#ifndef ARTERY_BASEFUSION_H
#define ARTERY_BASEFUSION_H

#include "artery/application/md/checks/CheckResult.h"
#include "vanetza/asn1/md/DetectionReferenceCAM.h"
#include <bitset>

namespace artery {

    namespace detectionLevels{


    enum DetectionLevels {
        Level1 = 0,
        Level2 = 1,
        Level3 = 2,
        Level4 = 3,
        SIZE_OF_ENUM
    };
    }
    class BaseFusion {
    public:
        BaseFusion();
//        virtual DetectionReferenceCAM_t *checkForReport(CheckResult &checkResult) = 0;

        virtual std::vector<std::bitset<16>> checkForReport(const CheckResult &checkResult) = 0;
    };

} // namespace artery

#endif //ARTERY_BASEFUSION_H
