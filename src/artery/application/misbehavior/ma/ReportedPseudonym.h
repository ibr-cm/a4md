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
#include "artery/application/misbehavior/ma/Report.h"
#include <map>

namespace artery {

    struct ReportSummary {
        std::string id;
        double score;
        ma::DetectionType detectionType;
        std::shared_ptr<ReportedPseudonym> reportedPseudonym;
        uint64_t generationTime;

        ReportSummary(const std::string &id, double score, const ma::DetectionType &detectionType,
                      const std::shared_ptr<ReportedPseudonym> &reportedPseudonym, uint64_t generationTime)
                : id(id),
                  score(score),
                  detectionType(detectionType),
                  reportedPseudonym(reportedPseudonym),
                  generationTime(generationTime) {}
    };

    class ReportedPseudonym {
    public:
        explicit ReportedPseudonym(const std::shared_ptr<ma::Report> &report);

        StationID_t getStationId() const { return mStationId; };

        void addReport(const std::shared_ptr<ReportSummary> &reportSummary);

        int getReportCount() { return (int) mReports.size(); };

        int getTotalScore() { return mTotalScore; };

        reactionTypes::ReactionTypes getReactionType() { return mReactionType; };

        void setReactionType(reactionTypes::ReactionTypes reactionType);

        misbehaviorTypes::MisbehaviorTypes predictMisbehaviorType();

        misbehaviorTypes::MisbehaviorTypes predictMisbehaviorTypeAggregate();


    private:
        StationID_t mStationId;
        int mValidReportCount = 0;
        double mTotalScore = 0;
        std::map<misbehaviorTypes::MisbehaviorTypes, int> mPredictedMisbehaviorTypeCount;
        attackTypes::AttackTypes mActualAttackType;
        reactionTypes::ReactionTypes mReactionType;
        std::map<std::string, std::shared_ptr<ReportSummary>> mReports;

    };
} //namespace artery

#endif //ARTERY_REPORTEDPSEUDONYM_H
