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
    class ReportedPseudonym {
    public:
        explicit ReportedPseudonym(const std::shared_ptr<ma::Report> &report);

        StationID_t getStationId() const { return mStationId; };

        void addReport(const std::shared_ptr<ma::Report> &report);

        std::vector<std::shared_ptr<ma::Report>> getReports() { return mReportList; };

        size_t getReportCount() { return mReportList.size(); };

        std::shared_ptr<ma::Report> getLastReport(){return mReportList.back();};

        reactionTypes::ReactionTypes getReactionType(){return mReactionType;};

        void updateReactionType();

        misbehaviorTypes::MisbehaviorTypes predictMisbehaviorType();

        misbehaviorTypes::MisbehaviorTypes predictMisbehaviorTypeAggregate();


    private:
        StationID_t mStationId;
        std::vector<std::shared_ptr<ma::Report>> mReportList;
        std::map<misbehaviorTypes::MisbehaviorTypes, int> mPredictedMisbehaviorTypeCount;
        attackTypes::AttackTypes mActualAttackType;
        reactionTypes::ReactionTypes mReactionType;

    };
} //namespace artery

#endif //ARTERY_REPORTEDPSEUDONYM_H
