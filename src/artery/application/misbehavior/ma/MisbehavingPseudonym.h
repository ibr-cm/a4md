//
// Created by bastian on 28.07.21.
//

#ifndef ARTERY_MISBEHAVINGPSEUDONYM_H
#define ARTERY_MISBEHAVINGPSEUDONYM_H


#include "vanetza/asn1/md/StationID.h"
#include "artery/application/misbehavior/util/MisbehaviorTypes.h"
#include "artery/application/misbehavior/util/AttackTypes.h"

namespace artery {

    class MisbehavingPseudonym {
    public:
        MisbehavingPseudonym(StationID_t stationId, misbehaviorTypes::MisbehaviorTypes misbehaviorType, attackTypes::AttackTypes attackType);

        StationID_t getMStationId() const;

        misbehaviorTypes::MisbehaviorTypes getMMisbehaviorType() const;

        attackTypes::AttackTypes getMAttackType() const;

    private:
        StationID_t mStationId;
        misbehaviorTypes::MisbehaviorTypes mMisbehaviorType;
        attackTypes::AttackTypes mAttackType;
    };

}// namespace artery
#endif //ARTERY_MISBEHAVINGPSEUDONYM_H
