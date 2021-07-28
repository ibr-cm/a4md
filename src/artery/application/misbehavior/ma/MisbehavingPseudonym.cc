//
// Created by bastian on 28.07.21.
//

#include "MisbehavingPseudonym.h"

namespace artery {


    StationID_t MisbehavingPseudonym::getMStationId() const {
        return mStationId;
    }

    misbehaviorTypes::MisbehaviorTypes artery::MisbehavingPseudonym::getMMisbehaviorType() const {
        return mMisbehaviorType;
    }

    attackTypes::AttackTypes artery::MisbehavingPseudonym::getMAttackType() const {
        return mAttackType;
    }

    MisbehavingPseudonym::MisbehavingPseudonym(StationID_t stationId,
                                               misbehaviorTypes::MisbehaviorTypes misbehaviorType,
                                               attackTypes::AttackTypes attackType) :
            mStationId(stationId),
            mMisbehaviorType(misbehaviorType),
            mAttackType(attackType) {

    }
} // namespace artery
