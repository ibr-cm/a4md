//
// Created by bastian on 07.07.21.
//

#ifndef ARTERY_BASEFUSION_H
#define ARTERY_BASEFUSION_H

#include "artery/application/md/checks/CheckResult.h"
#include "vanetza/asn1/md/DetectionReferenceCAM.h"

namespace artery {

    class BaseFusion {
    public:
        BaseFusion();
        virtual DetectionReferenceCAM_t *checkForReport(CheckResult &checkResult) = 0;
    };

} // namespace artery

#endif //ARTERY_BASEFUSION_H
