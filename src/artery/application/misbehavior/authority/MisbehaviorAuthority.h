//
// Created by bastian on 05.07.21.
//

#ifndef ARTERY_MISBEHAVIORAUTHORITY_H
#define ARTERY_MISBEHAVIORAUTHORITY_H

#include <curl/curl.h>
#include <omnetpp.h>
#include <rapidjson/document.h>
#include <vanetza/asn1/misbehavior_report.hpp>
#include <vanetza/asn1/cam.hpp>
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/application/Timer.h"
#include "artery/application/misbehavior/util/MisbehaviorTypes.h"
#include "artery/application/misbehavior/util/AttackTypes.h"
#include "artery/application/misbehavior/authority/MisbehavingPseudonym.h"
#include "artery/application/misbehavior/report/ReportedPseudonym.h"
#include "artery/application/misbehavior/report/ReportingPseudonym.h"
#include "artery/application/misbehavior/report/Report.h"
#include "artery/application/misbehavior/report/ReportSummary.h"
#include "artery/application/misbehavior/util/HelperFunctions.h"
#include "artery/application/misbehavior/fusion/BaseFusion.h"
#include <artery/application/misbehavior/checks/BaseChecks.h>

namespace artery {

    namespace {

        struct RecentReported {
            StationID_t stationId;
            int reportCount;
            uint64_t lastGenerationTime;
        };

        struct RecentReportedCompare {
            bool operator()(const RecentReported &a, const RecentReported &b) const {
                return a.lastGenerationTime < b.lastGenerationTime;
            }
        };
    }


    class MisbehaviorAuthority : public omnetpp::cSimpleModule, public omnetpp::cListener {
    public:
        MisbehaviorAuthority();

        ~MisbehaviorAuthority();

        void initialize() override;

        void handleMessage(omnetpp::cMessage *) override;

        void receiveSignal(omnetpp::cComponent *source, omnetpp::simsignal_t signalId, const omnetpp::SimTime &t,
                           omnetpp::cObject *) override;

        void receiveSignal(omnetpp::cComponent *source, omnetpp::simsignal_t signal, cObject *obj,
                           omnetpp::cObject *) override;

    private:

        void clear();

        omnetpp::simsignal_t traciInitSignal;
        omnetpp::simsignal_t traciCloseSignal;
        omnetpp::simsignal_t maNewReport;
        omnetpp::simsignal_t maMisbehaviorAnnouncement;

        omnetpp::cMessage *mMsgGuiUpdate;
        omnetpp::cMessage *mMsgReportCleanup;
        CURL *curl;
        GlobalEnvironmentModel *mGlobalEnvironmentModel;
        traci::Boundary mSimulationBoundary;
        std::shared_ptr<const traci::API> mTraciAPI;
        Timer mTimer;
        BaseFusion *mFusionApplication;
        BaseChecks *mBaseChecks;
        MisbehaviorAuthorityParameters *mParameters;

        std::map<StationID_t, std::shared_ptr<ReportedPseudonym>> mReportedPseudonyms;
        std::map<StationID_t, std::shared_ptr<MisbehavingPseudonym>> mMisbehavingPseudonyms;
        std::map<StationID_t, std::shared_ptr<ReportingPseudonym>> mReportingPseudonyms;

        std::map<std::string, std::shared_ptr<Report>> mCurrentReports;
        std::set<std::shared_ptr<vanetza::asn1::Cam>, CamCompare> mCams;
        std::map<std::string, std::shared_ptr<ma::ReportSummary>> mReports;


        std::list<std::string> mDetectionAccuracyLabels;
        std::list<std::tuple<int, int, double>> mDetectionAccuracyData;
        std::map<reactionTypes::ReactionTypes, std::set<StationID_t>> mReactionsList;

        int mTotalReportCount = 0;
        bool mNewReport = false;
        uint64_t mLastUpdateTime = 0;

        int mTrueDetectionCount = 0;
        int mFalseDetectionCount = 0;
        double mDetectionRate = 0;
        int mTrueDetectionAggregateCount = 0;
        int mFalseDetectionAggregateCount = 0;
        double mDetectionRateAggregate = 0;
        int mTrueDetectionCountInst = 0;
        int mFalseDetectionCountInst = 0;
        int mDetectionRateCur = 0;

        void updateDetectionRates(ReportedPseudonym &reportedPseudonym, const Report &report);

        misbehaviorTypes::MisbehaviorTypes getActualMisbehaviorType(const StationID_t &stationId);

        void updateReactionType(ReportedPseudonym &reportedPseudonym);

        bool validateSemanticLevel1Report(const Report &report);

        bool validateSemanticLevel2Report(const Report &report);

        bool validateSemanticLevel3Report(const Report &report);

        bool validateSemanticLevel4Report(const Report &report);

        bool validateReportReason(const Report &report);

        void removeOldReports();

        std::vector<RecentReported> getRecentReported();

        rapidjson::Value getRadarData(rapidjson::Document::AllocatorType &allocator);

        rapidjson::Value getRecentReported(rapidjson::Document::AllocatorType &allocator);

        rapidjson::Value getDetectionRates(rapidjson::Document::AllocatorType &allocator);

        void createGuiJsonData();

        void printReportsPerPseudonym();

        void processReport(const shared_ptr<Report>& report);

        double scoreReport(const shared_ptr<Report> &report);
    };
} // namespace artery

#endif //ARTERY_MISBEHAVIORAUTHORITY_H
