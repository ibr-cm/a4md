//
// Created by bastian on 06.08.21.
//

#ifndef ARTERY_LEGACYVALIDATE_H
#define ARTERY_LEGACYVALIDATE_H

#include "artery/application/misbehavior/checks/LegacyChecks.h"
#include "artery/application/misbehavior/fusion/ThresholdFusion.h"

namespace artery {

    class LegacyValidate : public artery::LegacyChecks {

    public:
        LegacyValidate(shared_ptr<const traci::API> traciAPI,
                       artery::GlobalEnvironmentModel *globalEnvironmentModel,
                       artery::DetectionParameters *detectionParameters);

        std::bitset<16> checkSemanticLevel2Report(const vanetza::asn1::Cam &currentCam,
                                                  const vanetza::asn1::Cam &lastCam);

    private:
        ThresholdFusion *mThresholdFusion;
    };
}


#endif //ARTERY_LEGACYVALIDATE_H
