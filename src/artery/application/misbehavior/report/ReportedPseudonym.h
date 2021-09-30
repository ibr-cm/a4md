//
// Created by bastian on 28.07.21.
//

#ifndef ARTERY_REPORTEDPSEUDONYM_H
#define ARTERY_REPORTEDPSEUDONYM_H

#include "vanetza/asn1/md/StationID.h"
#include "artery/application/misbehavior/util/MisbehaviorTypes.h"
#include "artery/application/misbehavior/util/AttackTypes.h"
#include "artery/application/misbehavior/util/ReactionTypes.h"
#include "artery/application/misbehavior/report/Report.h"
#include <map>
#include <omnetpp.h>
#include <omnetpp/ccomponent.h>
#include <boost/shared_ptr.hpp>

namespace artery {

    class Report;

    class ReportedPseudonym {
    public:
        explicit ReportedPseudonym(const std::shared_ptr<Report> &report);

        void addReport(const std::shared_ptr<Report> &report);

        void recordStatistics();

        StationID_t getStationId() const { return mStationId; };

        int getTotalReportCount() { return mTotalReportCount; };

        int getTotalScore() { return mTotalScore; };

        int getValidReportCount() { return mValidReportCount; };


        uint64_t getPreviousReportGenerationTime() { return mLastReportGenerationTime; };

        reactionTypes::ReactionTypes getReactionType() { return mReactionType; };

        void setReactionType(reactionTypes::ReactionTypes reactionType);

        misbehaviorTypes::MisbehaviorTypes predictMisbehaviorType();

        misbehaviorTypes::MisbehaviorTypes predictMisbehaviorTypeAggregate();

    public:
        int falsePositiveCount = 0;
        int truePositiveCount = 0;
        double falsePositiveScoreSum = 0;

    private:
        StationID_t mStationId;
        int mTotalReportCount = 0;
        int mValidReportCount = 0;
        double mTotalScore = 0;
        std::map<misbehaviorTypes::MisbehaviorTypes, int> mPredictedMisbehaviorTypeCount;
        attackTypes::AttackTypes mActualAttackType;
        reactionTypes::ReactionTypes mReactionType;
        uint64_t mLastReportGenerationTime;

        omnetpp::cOutVector vectorReportingPseudonym;
        omnetpp::cStdDev statsValidity;
        omnetpp::cStdDev statsScore;
        omnetpp::cHistogram statsDetectionType;
        omnetpp::cHistogram statsDetectionLevel;
        omnetpp::cHistogram statsErrorCode;
    };
} //namespace artery

#endif //ARTERY_REPORTEDPSEUDONYM_H
