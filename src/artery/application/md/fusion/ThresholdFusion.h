//
// Created by bastian on 07.07.21.
//

#ifndef ARTERY_THRESHOLDFUSION_H
#define ARTERY_THRESHOLDFUSION_H

#include "artery/application/md/checks/CheckResult.h"
#include "artery/application/md/fusion/BaseFusion.h"

namespace artery {

    class ThresholdFusion : public BaseFusion {
    public:
        explicit ThresholdFusion(double threshold);
//        DetectionReferenceCAM_t *checkForReport(CheckResult &checkResult) override;
        std::vector<std::bitset<16>> checkForReport(const CheckResult &checkResult) override;

    private:
        double threshold;

    };

} // namespace artery

#endif //ARTERY_THRESHOLDFUSION_H
