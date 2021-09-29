//
// Created by bastian on 28.09.21.
//

#ifndef ARTERY_MISBEHAVINGVEHICLE_H
#define ARTERY_MISBEHAVINGVEHICLE_H

#include "vanetza/asn1/md/StationID.h"
#include "artery/application/misbehavior/util/MisbehaviorTypes.h"
#include "artery/application/misbehavior/util/AttackTypes.h"
#include <omnetpp.h>
#include <memory>

namespace artery {
    class MisbehavingPseudonym;

    class MisbehavingVehicle {
    public:
        MisbehavingVehicle(StationID_t stationId, misbehaviorTypes::MisbehaviorTypes misbehaviorType,
                           attackTypes::AttackTypes attackType);

        StationID_t getStationId() const {return mStationId;};

        misbehaviorTypes::MisbehaviorTypes getMisbehaviorType() const{return mMisbehaviorType;};

        attackTypes::AttackTypes getAttackType() const{return mAttackType;};

        void addPseudonym(std::shared_ptr<MisbehavingPseudonym> &misbehavingPseudonym);

        std::vector<std::shared_ptr<MisbehavingPseudonym>> getPseudonyms() const {return mPseudonyms;};

    private:
        StationID_t mStationId;
        std::vector<std::shared_ptr<MisbehavingPseudonym>> mPseudonyms;
        misbehaviorTypes::MisbehaviorTypes mMisbehaviorType;
        attackTypes::AttackTypes mAttackType;
    };

}// namespace artery
#endif //ARTERY_MISBEHAVINGVEHICLE_H
