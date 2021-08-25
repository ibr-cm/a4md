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
    }

    void ReportedPseudonym::addReport(const std::shared_ptr<ma::ReportSummary> &reportSummary, uint64_t generationTime) {
        mTotalScore += reportSummary->score;
        if(reportSummary->score > 0){
            mValidReportCount++;
        }
        mReports.emplace(reportSummary->id, reportSummary);
        mLastReportGenerationTime = generationTime;
    }

    void ReportedPseudonym::setReactionType(reactionTypes::ReactionTypes reactionType) {
        mReactionType = reactionType;
    }

    misbehaviorTypes::MisbehaviorTypes ReportedPseudonym::predictMisbehaviorType() {
        misbehaviorTypes::MisbehaviorTypes predictedMisbehaviorType;
        if (mValidReportCount > F2MDParameters::misbehaviorAuthorityParameters.reportCountThreshold) {
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
