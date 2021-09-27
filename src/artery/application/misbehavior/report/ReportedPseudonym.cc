//
// Created by bastian on 28.07.21.
//

#include "ReportedPseudonym.h"
#include "artery/application/misbehavior/util/F2MDParameters.h"
#include <algorithm>
#include <string>
#include <boost/shared_ptr.hpp>

namespace artery {

    ReportedPseudonym::ReportedPseudonym(const std::shared_ptr<Report> &report) :
            mStationId((*report->reportedMessage)->header.stationID),
            mReactionType(reactionTypes::Nothing),
            mActualAttackType(attackTypes::Benign),
            mLastReportGenerationTime(report->generationTime) {
        for (int misbehaviorType = 0; misbehaviorType < misbehaviorTypes::SIZE_OF_ENUM; misbehaviorType++) {
            mPredictedMisbehaviorTypeCount[static_cast<misbehaviorTypes::MisbehaviorTypes>(misbehaviorType)] = 0;
        }
        std::string prefix = {"reportedPseudonym_" + std::to_string(mStationId) + "_"};
        vectorReportingPseudonym.setName((prefix + "reportingPseudonym").c_str());
        statsValidity.setName((prefix + "validity").c_str());
        statsScore.setName((prefix + "score").c_str());
        statsDetectionType.setName((prefix + "detectionType").c_str());
        statsDetectionLevel.setName((prefix + "detectionLevel").c_str());
        statsErrorCode.setName((prefix + "errorCode").c_str());
    }

    void ReportedPseudonym::addReport(const std::shared_ptr<Report> &report) {
        mTotalScore += report->score;
        if (report->score > 0) {
            mValidReportCount++;
        }
        mTotalReportCount++;
        mLastReportGenerationTime = report->generationTime;
        vectorReportingPseudonym.record(report->reportingPseudonym->getStationId());
        statsValidity.collect(report->isValid);
        statsScore.collect(report->score);
        statsDetectionType.collect(report->detectionType.present);
        statsDetectionLevel.collect(report->detectionType.semantic->detectionLevel);
        statsErrorCode.collect(report->detectionType.semantic->errorCode.to_ulong());
    }

    void ReportedPseudonym::recordStatistics() {
        statsValidity.record();
        statsScore.record();
        statsDetectionType.record();
        statsDetectionLevel.record();
        statsErrorCode.record();
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
