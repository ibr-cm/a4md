//
// Created by bastian on 05.07.21.
//

#ifndef ARTERY_MISBEHAVIORAUTHORITY_H
#define ARTERY_MISBEHAVIORAUTHORITY_H

#include <curl/curl.h>
#include <omnetpp.h>
#include <vanetza/asn1/misbehavior_report.hpp>
#include <vanetza/asn1/cam.hpp>
#include <rapidjson/document.h>
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/application/Timer.h"
#include "artery/application/misbehavior/util/MisbehaviorTypes.h"
#include "artery/application/misbehavior/util/AttackTypes.h"
#include "artery/application/misbehavior/ma/ReportedPseudonym.h"
#include "artery/application/misbehavior/ma/MisbehavingPseudonym.h"
#include "artery/application/misbehavior/ma/Report.h"
#include "artery/application/misbehavior/ma/ReportSummary.h"
#include "artery/application/misbehavior/util/HelperFunctions.h"
#include <rapidjson/document.h>
#include "artery/application/misbehavior/fusion/BaseFusion.h"
#include <artery/application/misbehavior/checks/BaseChecks.h>

namespace artery {

    namespace {
        struct CamCompare {
            bool operator()(const std::shared_ptr<vanetza::asn1::Cam> &ptr1,
                            const std::shared_ptr<vanetza::asn1::Cam> &ptr2) {
                return camCompPtr(ptr1, ptr2);
            }
        };

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

        std::map<StationID_t, std::shared_ptr<ReportedPseudonym>> mReportedPseudonyms;
        std::map<StationID_t, MisbehavingPseudonym *> mMisbehavingPseudonyms;
        std::map<std::string, std::shared_ptr<ma::Report>> mCurrentReports;
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


        std::shared_ptr<ma::Report> parseReport(const vanetza::asn1::MisbehaviorReport &misbehaviorReport);

        void parseMessageEvidenceContainer(const MessageEvidenceContainer &messageEvidenceContainer,
                                           std::vector<std::shared_ptr<vanetza::asn1::Cam>> &messages);

        void updateDetectionRates(ReportedPseudonym &reportedPseudonym, const ma::Report &report);

        misbehaviorTypes::MisbehaviorTypes getActualMisbehaviorType(const StationID_t &stationId);


        void updateReactionType(ReportedPseudonym &reportedPseudonym);

        bool validateSemanticLevel1Report(const ma::Report &report);

        bool validateSemanticLevel2Report(const ma::Report &report);

        bool validateSemanticLevel3Report(const ma::Report &report);

        bool validateSemanticLevel4Report(const ma::Report &report);

        bool validateReportReason(const ma::Report &report);

        void removeOldReports();

        shared_ptr<vanetza::asn1::Cam> getCamFromOpaque(const Opaque_t &data);


        std::vector<RecentReported> getRecentReported();

        rapidjson::Value getRadarData(rapidjson::Document::AllocatorType &allocator);

        rapidjson::Value getRecentReported(rapidjson::Document::AllocatorType &allocator);

        rapidjson::Value getDetectionRates(rapidjson::Document::AllocatorType &allocator);

        void createGuiJsonData();

        void printReportsPerPseudonym();
    };
} // namespace artery

#endif //ARTERY_MISBEHAVIORAUTHORITY_H
