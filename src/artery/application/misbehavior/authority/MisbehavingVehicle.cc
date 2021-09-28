//
// Created by bastian on 28.09.21.
//

#include "MisbehavingVehicle.h"

namespace artery {
    artery::MisbehavingVehicle::MisbehavingVehicle(StationID_t stationId,
                                                   artery::misbehaviorTypes::MisbehaviorTypes misbehaviorType,
                                                   artery::attackTypes::AttackTypes attackType) :
            mStationId(stationId),
            mMisbehaviorType(misbehaviorType),
            mAttackType(attackType) {

    }

    void MisbehavingVehicle::addPseudonym(std::shared_ptr<MisbehavingPseudonym> &misbehavingPseudonym) {
        mPseudonyms.emplace_back(misbehavingPseudonym);
    }
} // namespace artery