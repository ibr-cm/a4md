//
// Created by bastian on 28.07.21.
//

#include "MisbehavingPseudonym.h"

#include <utility>

namespace artery {

    using namespace omnetpp;
    MisbehavingPseudonym::MisbehavingPseudonym(StationID_t stationId,
                                               misbehaviorTypes::MisbehaviorTypes misbehaviorType,
                                               attackTypes::AttackTypes attackType,
                                               std::shared_ptr<MisbehavingVehicle> misbehavingVehicle) :
            mStationId(stationId),
            mVehicle(std::move(misbehavingVehicle)),
            mMisbehaviorType(misbehaviorType),
            mAttackType(attackType) {
    }

    StationID_t MisbehavingPseudonym::getStationId() const {
        return mStationId;
    }

    misbehaviorTypes::MisbehaviorTypes artery::MisbehavingPseudonym::getMisbehaviorType() const {
        return mMisbehaviorType;
    }

    attackTypes::AttackTypes artery::MisbehavingPseudonym::getAttackType() const {
        return mAttackType;
    }

    const std::shared_ptr<MisbehavingVehicle> &MisbehavingPseudonym::getVehicle() const {
        return mVehicle;
    }
} // namespace artery
