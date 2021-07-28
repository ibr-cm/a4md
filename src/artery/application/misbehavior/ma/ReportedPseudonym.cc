//
// Created by bastian on 28.07.21.
//

#include "ReportedPseudonym.h"

namespace artery {

    ReportedPseudonym::ReportedPseudonym(const StationID_t &stationId, const vanetza::asn1::MisbehaviorReport &report) :
            mStationId(stationId),
            mReactionType(reactionTypes::Nothing),
            mActualAttackType(attackTypes::Benign),
            mActualMisbehaviorType(misbehaviorTypes::Benign) {
        for (int misbehaviorType = 0; misbehaviorType < misbehaviorTypes::SIZE_OF_ENUM; misbehaviorType++) {
            mPredictedMisbehaviorTypeCount[static_cast<misbehaviorTypes::MisbehaviorTypes>(misbehaviorType)] = 0;
        }
        addReport(report);
    }

    void ReportedPseudonym::addReport(const vanetza::asn1::MisbehaviorReport &report) {
        mReportList.emplace_back(report);
        updateReactionType();
    }

    void ReportedPseudonym::updateReactionType() {
        if (mReportList.size() > 20) {
            mReactionType = reactionTypes::Warning;
        } else if (mReportList.size() > 50) {
            mReactionType = reactionTypes::Ticket;
        } else if (mReportList.size() > 250) {
            mReactionType = reactionTypes::PassiveRevocation;
        } else if (mReportList.size() > 1250) {
            mReactionType = reactionTypes::ActiveRevocation;
        }

    }

} // namespace artery
