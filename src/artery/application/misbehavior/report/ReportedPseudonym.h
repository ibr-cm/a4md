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
#include "artery/application/misbehavior/report/Report.h"
#include "artery/application/misbehavior/report/ReportSummary.h"
#include <map>
#include <omnetpp.h>

namespace artery {

    class Report;

    class ReportedPseudonym {
    public:
        explicit ReportedPseudonym(const std::shared_ptr<Report> &report);

        StationID_t getStationId() const { return mStationId; };

        void addReport(const std::shared_ptr<ma::ReportSummary> &reportSummary, uint64_t generationTime);

        int getReportCount() { return (int) mReports.size(); };

        int getTotalScore() { return mTotalScore; };

        int getValidReportCount() { return mValidReportCount; };

        uint64_t getPreviousReportGenerationTime() { return mLastReportGenerationTime; };

        reactionTypes::ReactionTypes getReactionType() { return mReactionType; };

        void setReactionType(reactionTypes::ReactionTypes reactionType);

        misbehaviorTypes::MisbehaviorTypes predictMisbehaviorType();

        misbehaviorTypes::MisbehaviorTypes predictMisbehaviorTypeAggregate();

        omnetpp::simsignal_t signalReportReportingPseudonym;
        omnetpp::simsignal_t signalReportValidity;
        omnetpp::simsignal_t signalReportScore;
        omnetpp::simsignal_t signalReportDetectionType;
        omnetpp::simsignal_t signalReportDetectionLevel;
        omnetpp::simsignal_t signalReportErrorCode;

    private:
        StationID_t mStationId;
        int mValidReportCount = 0;
        double mTotalScore = 0;
        std::map<misbehaviorTypes::MisbehaviorTypes, int> mPredictedMisbehaviorTypeCount;
        attackTypes::AttackTypes mActualAttackType;
        reactionTypes::ReactionTypes mReactionType;
        uint64_t mLastReportGenerationTime;
        std::map<std::string, std::shared_ptr<ma::ReportSummary>> mReports;

    };
} //namespace artery

#endif //ARTERY_REPORTEDPSEUDONYM_H
