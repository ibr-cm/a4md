//
// Created by bastian on 28.07.21.
//

#include "ReportedPseudonym.h"
#include "artery/application/misbehavior/util/F2MDParameters.h"
#include <algorithm>

namespace artery {

    ReportedPseudonym::ReportedPseudonym(const std::shared_ptr<ma::Report> &report) :
            mStationId((*report->reportedMessage)->header.stationID),
            mReactionType(reactionTypes::Nothing),
            mActualAttackType(attackTypes::Benign) {
        for (int misbehaviorType = 0; misbehaviorType < misbehaviorTypes::SIZE_OF_ENUM; misbehaviorType++) {
            mPredictedMisbehaviorTypeCount[static_cast<misbehaviorTypes::MisbehaviorTypes>(misbehaviorType)] = 0;
        }
        addReport(report);
    }

    void ReportedPseudonym::addReport(const std::shared_ptr<ma::Report> &report) {
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

    misbehaviorTypes::MisbehaviorTypes ReportedPseudonym::predictMisbehaviorType() {
        misbehaviorTypes::MisbehaviorTypes predictedMisbehaviorType;
        if (mReportList.size() > F2MDParameters::misbehaviorAuthorityParameters.reportCountThreshold) {
            predictedMisbehaviorType = misbehaviorTypes::LocalAttacker;
        } else {
            predictedMisbehaviorType = misbehaviorTypes::Benign;
        }
        mPredictedMisbehaviorTypeCount[predictedMisbehaviorType]++;
        return predictedMisbehaviorType;
    }

    misbehaviorTypes::MisbehaviorTypes ReportedPseudonym::predictMisbehaviorTypeAggregate() {
        auto maxElement =
                std::max_element(mPredictedMisbehaviorTypeCount.begin(), mPredictedMisbehaviorTypeCount.end(),
                                 [](const std::pair<misbehaviorTypes::MisbehaviorTypes, int> &a,
                                    const std::pair<misbehaviorTypes::MisbehaviorTypes, int> &b) -> bool {
                                     return a.second < b.second;
                                 });
        return maxElement->first;
    }

} // namespace artery
