//
// Created by bastian on 28.07.21.
//

#ifndef ARTERY_MISBEHAVINGPSEUDONYM_H
#define ARTERY_MISBEHAVINGPSEUDONYM_H


#include "vanetza/asn1/md/StationID.h"
#include "artery/application/misbehavior/util/MisbehaviorTypes.h"
#include "artery/application/misbehavior/util/AttackTypes.h"
#include "MisbehavingVehicle.h"
#include <omnetpp.h>
#include <memory>

namespace artery {

    class MisbehavingPseudonym {
    public:
        MisbehavingPseudonym(StationID_t stationId, misbehaviorTypes::MisbehaviorTypes misbehaviorType,
                             attackTypes::AttackTypes attackType,
                             std::shared_ptr<MisbehavingVehicle> misbehavingVehicle);

        StationID_t getStationId() const;

        misbehaviorTypes::MisbehaviorTypes getMisbehaviorType() const;

        attackTypes::AttackTypes getAttackType() const;

    private:
        StationID_t mStationId;
    public:
        const std::shared_ptr<MisbehavingVehicle> &getVehicle() const;

    private:
        std::shared_ptr<MisbehavingVehicle> mVehicle;
        misbehaviorTypes::MisbehaviorTypes mMisbehaviorType;
        attackTypes::AttackTypes mAttackType;
    };

}// namespace artery
#endif //ARTERY_MISBEHAVINGPSEUDONYM_H
