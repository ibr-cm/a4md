//
// Created by bastian on 28.07.21.
//

#ifndef ARTERY_REPORTEDPSEUDONYM_H
#define ARTERY_REPORTEDPSEUDONYM_H

#include <vanetza/asn1/misbehavior_report.hpp>
#include "vanetza/asn1/md/StationID.h"
#include "artery/application/misbehavior/util/MisbehaviorTypes.h"
#include "artery/application/misbehavior/util/AttackTypes.h"
#include "artery/application/misbehavior/util/ReactionTypes.h"
#include <map>

namespace artery {
    class ReportedPseudonym {
    public:
        ReportedPseudonym(const StationID_t &stationId, const vanetza::asn1::MisbehaviorReport &report);

        void addReport(const vanetza::asn1::MisbehaviorReport &report);

        void updateReactionType();

    private:
        StationID_t mStationId;
        std::vector<vanetza::asn1::MisbehaviorReport> mReportList;
        std::map<misbehaviorTypes::MisbehaviorTypes, int> mPredictedMisbehaviorTypeCount;
        misbehaviorTypes::MisbehaviorTypes mActualMisbehaviorType;
        attackTypes::AttackTypes mActualAttackType;
        reactionTypes::ReactionTypes mReactionType;

    };
} //namespace artery

#endif //ARTERY_REPORTEDPSEUDONYM_H
