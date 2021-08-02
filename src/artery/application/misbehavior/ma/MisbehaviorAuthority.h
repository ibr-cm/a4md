//
// Created by bastian on 05.07.21.
//

#ifndef ARTERY_MISBEHAVIORAUTHORITY_H
#define ARTERY_MISBEHAVIORAUTHORITY_H

#include <omnetpp.h>
#include <vanetza/asn1/misbehavior_report.hpp>
#include <vanetza/asn1/cam.hpp>
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/application/Timer.h"
#include "artery/application/misbehavior/util/MisbehaviorTypes.h"
#include "artery/application/misbehavior/util/AttackTypes.h"
#include "artery/application/misbehavior/ma/ReportedPseudonym.h"
#include "artery/application/misbehavior/ma/MisbehavingPseudonym.h"
#include "artery/application/misbehavior/ma/Report.h"
#include "artery/application/misbehavior/util/HelperFunctions.h"

namespace artery {

    namespace {
        struct CamCompare {
            bool operator()(const std::shared_ptr<vanetza::asn1::Cam> &ptr1,
                            const std::shared_ptr<vanetza::asn1::Cam> &ptr2) {
                return camCompPtr(ptr1, ptr2);
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

        omnetpp::cMessage* mSelfMsg;

        GlobalEnvironmentModel *mGlobalEnvironmentModel;
        std::shared_ptr<const traci::API> mTraciAPI;
        Timer mTimer;

        std::map<StationID_t, ReportedPseudonym *> mReportedPseudonyms;
        std::map<StationID_t, MisbehavingPseudonym *> mMisbehavingPseudonyms;
        std::map<std::string, std::shared_ptr<ma::Report>> mReports;
        std::set<std::shared_ptr<vanetza::asn1::Cam>, CamCompare> mCams;


        std::list<std::string> mDetectionAccuracyLabels;
        std::list<std::tuple<int,int,double>> mDetectionAccuracyData;

        int mTotalReportCount;
        int mNewReport;
        uint64_t mLastUpdateTime;

        int mTrueDetectionCount;
        int mFalseDetectionCount;
        double mDetectionRate;
        int mTrueDetectionAggregateCount;
        int mFalseDetectionAggregateCount;
        double mDetectionRateAggregate;
        int mTrueDetectionCountInst;
        int mFalseDetectionCountInst;
        int mDetectionRateCur;


        ma::Report *parseReport(const vanetza::asn1::MisbehaviorReport &misbehaviorReport);

        void parseMessageEvidenceContainer(const MessageEvidenceContainer &messageEvidenceContainer,
                                           std::vector<std::shared_ptr<vanetza::asn1::Cam>> &messages);

        void updateDetectionRates(ReportedPseudonym &reportedPseudonym, const ma::Report &report);

        std::list<std::tuple<StationID_t ,int,uint64_t>> getRecentReported();
    };
} // namespace artery

#endif //ARTERY_MISBEHAVIORAUTHORITY_H
