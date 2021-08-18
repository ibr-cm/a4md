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
#include "artery/application/misbehavior/ma/ReportedPseudonym.h"
#include <map>

namespace artery {
    class ReportedPseudonym {
    public:
        explicit ReportedPseudonym(const std::shared_ptr<ma::Report> &report);

        StationID_t getStationId() const { return mStationId; };

        void addReport(const std::shared_ptr<ma::Report> &report);

        void removeReport(const std::shared_ptr<ma::Report> &report);

        std::vector<std::shared_ptr<ma::Report>> getReports() { return mCurrentReportList; };

        int getReportCount() { return (int) mCurrentReportList.size(); };

        int getValidReportCount() {return mValidReportCount;};

        std::shared_ptr<ma::Report> getLastReport(){return mCurrentReportList.back();};

        reactionTypes::ReactionTypes getReactionType(){return mReactionType;};

        void setReactionType(reactionTypes::ReactionTypes reactionType);

        misbehaviorTypes::MisbehaviorTypes predictMisbehaviorType();

        misbehaviorTypes::MisbehaviorTypes predictMisbehaviorTypeAggregate();


    private:
        StationID_t mStationId;
        std::vector<std::shared_ptr<ma::Report>> mCurrentReportList;
        int mValidReportCount = 0;
        std::map<misbehaviorTypes::MisbehaviorTypes, int> mPredictedMisbehaviorTypeCount;
        attackTypes::AttackTypes mActualAttackType;
        reactionTypes::ReactionTypes mReactionType;
        std::map<std::string,int> mReportScores;

    };
} //namespace artery

#endif //ARTERY_REPORTEDPSEUDONYM_H
