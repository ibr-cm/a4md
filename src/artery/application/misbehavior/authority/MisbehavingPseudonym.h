//
// Created by bastian on 28.07.21.
//

#ifndef ARTERY_MISBEHAVINGPSEUDONYM_H
#define ARTERY_MISBEHAVINGPSEUDONYM_H


#include "vanetza/asn1/md/StationID.h"
#include "artery/application/misbehavior/util/MisbehaviorTypes.h"
#include "artery/application/misbehavior/util/AttackTypes.h"
#include <omnetpp.h>

namespace artery {

    class MisbehavingPseudonym {
    public:
        MisbehavingPseudonym(StationID_t stationId, misbehaviorTypes::MisbehaviorTypes misbehaviorType, attackTypes::AttackTypes attackType);

        StationID_t getStationId() const;

        misbehaviorTypes::MisbehaviorTypes getMisbehaviorType() const;

        attackTypes::AttackTypes getAttackType() const;

        omnetpp::simsignal_t signalPseudonym;
        omnetpp::simsignal_t signalMisbehaviorType;
        omnetpp::simsignal_t signalAttackType;


    private:
        StationID_t mStationId;
        misbehaviorTypes::MisbehaviorTypes mMisbehaviorType;
        attackTypes::AttackTypes mAttackType;
    };

}// namespace artery
#endif //ARTERY_MISBEHAVINGPSEUDONYM_H
