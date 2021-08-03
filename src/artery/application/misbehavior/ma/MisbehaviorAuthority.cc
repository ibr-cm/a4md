//
// Created by bastian on 05.07.21.
//

#include <artery/application/CaObject.h>
#include "MisbehaviorAuthority.h"
#include "traci/Core.h"
#include "artery/traci/Cast.h"
#include "artery/application/misbehavior/MisbehaviorReportObject.h"
#include "artery/application/misbehavior/MisbehaviorCaService.h"
#include "artery/application/misbehavior/util/DetectionLevels.h"
#include <bitset>
#include <chrono>
#include <numeric>

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/writer.h>


namespace artery {

    using namespace omnetpp;

    Define_Module(MisbehaviorAuthority)

    MisbehaviorAuthority::MisbehaviorAuthority() {
        curl = curl_easy_init();
        traciInitSignal = cComponent::registerSignal("traci.init");
        traciCloseSignal = cComponent::registerSignal("traci.close");
        maNewReport = cComponent::registerSignal("misbehaviorAuthority.newReport");
        maMisbehaviorAnnouncement = cComponent::registerSignal("misbehaviorAuthority.MisbehaviorAnnouncement");
    }

    MisbehaviorAuthority::~MisbehaviorAuthority() {
        curl_easy_cleanup(curl);
        this->clear();
    }

    void MisbehaviorAuthority::clear() {
    }

    void MisbehaviorAuthority::initialize() {
        cModule *traci = this->getParentModule()->getSubmodule("traci");
        traci->subscribe(traciInitSignal, this);
        traci->subscribe(traciCloseSignal, this);
        getSimulation()->getSystemModule()->subscribe(maNewReport, this);
        getSimulation()->getSystemModule()->subscribe(maMisbehaviorAnnouncement, this);

        mTimer.setTimebase(par("datetime"));
        cModule *globalEnvMod = this->getParentModule()->getSubmodule("environmentModel");
        if (globalEnvMod == nullptr) {
            throw cRuntimeError("globalEnvMod not found");
        }
        mGlobalEnvironmentModel = dynamic_cast<GlobalEnvironmentModel *>(globalEnvMod);

        F2MDParameters::misbehaviorAuthorityParameters.maxReportAge = par("maxReportAge");
        F2MDParameters::misbehaviorAuthorityParameters.reportCountThreshold = par("reportCountThreshold");
        F2MDParameters::misbehaviorAuthorityParameters.updateTimeStep = par("updateTimeStep");
        F2MDParameters::misbehaviorAuthorityParameters.displaySteps = par("displaySteps");
        F2MDParameters::misbehaviorAuthorityParameters.recentReportedCount = par("recentReportedCount");

        mSelfMsg = new cMessage("getDataScheduleMessage");
        scheduleAt(simTime() + 2.0, mSelfMsg);
    }

    void MisbehaviorAuthority::handleMessage(omnetpp::cMessage *msg) {
        Enter_Method("handleMessage");
        if (msg == mSelfMsg) {
            std::vector<RecentReported> recentReported = getRecentReported();

            rapidjson::Document d;
            d.SetObject();
            rapidjson::Document::AllocatorType &allocator = d.GetAllocator();
            d.AddMember("newReport", mNewReport, allocator);
            d.AddMember("totalReports", mTotalReportCount, allocator);
            d.AddMember("cumulativeDetectionRate", mDetectionRate, allocator);

            // getRadarData
            rapidjson::Value reactionsData(rapidjson::kObjectType);
            std::vector<int> reactionsBenign(5, 0);
            std::vector<int> reactionsMalicious(5, 0);

            for (auto reportedPseudonym : mReportedPseudonyms) {
                misbehaviorTypes::MisbehaviorTypes misbehaviorType = getActualMisbehaviorType(reportedPseudonym.second->getStationId());
                if (misbehaviorType == misbehaviorTypes::Benign) {
                    reactionsBenign[static_cast<int>(reportedPseudonym.second->getReactionType())]++;
                } else if (misbehaviorType == misbehaviorTypes::LocalAttacker ||
                           misbehaviorType == misbehaviorTypes::GlobalAttacker) {
                    reactionsMalicious[static_cast<int>(reportedPseudonym.second->getReactionType())]++;
                }
            }
            rapidjson::Value reactionsBenignJson;
            rapidjson::Value reactionsMaliciousJson;
            reactionsBenignJson.SetArray();
            reactionsMaliciousJson.SetArray();

            int sumReactionsBenign = std::accumulate(reactionsBenign.begin(), reactionsBenign.end(), 0);
            int sumReactionsMalicious = std::accumulate(reactionsMalicious.begin(), reactionsMalicious.end(), 0);
            if (sumReactionsBenign > 0) {
                for (auto &reaction : reactionsBenign) {
                    reactionsBenignJson.PushBack(100.0 * reaction / sumReactionsBenign, allocator);
                }
            } else {
                for (auto &reaction : reactionsBenign) {
                    reactionsBenignJson.PushBack(0, allocator);
                }
            }
            if (sumReactionsMalicious > 0) {
                for (auto &reaction : reactionsMalicious) {
                    reactionsMaliciousJson.PushBack(100 * reaction / sumReactionsMalicious, allocator);
                }
            } else {
                for (auto &reaction : reactionsMalicious) {
                    reactionsMaliciousJson.PushBack(0, allocator);
                }
            }
            reactionsData.AddMember("benign", reactionsBenignJson, allocator);
            reactionsData.AddMember("malicious", reactionsMaliciousJson, allocator);
            d.AddMember("reactionsData", reactionsData, allocator);


            rapidjson::Value recentlyReportedData(rapidjson::kObjectType);
            rapidjson::Value labels;
            rapidjson::Value data;
            labels.SetArray();
            data.SetArray();
            for (auto r : recentReported) {
                labels.PushBack(r.stationId, allocator);
                data.PushBack(r.reportCount, allocator);
            }
            recentlyReportedData.AddMember("labels", labels, allocator);
            recentlyReportedData.AddMember("data", data, allocator);
            d.AddMember("recentlyReportedData", recentlyReportedData, allocator);

            rapidjson::Value detectionRatesData(rapidjson::kObjectType);
            rapidjson::Value detectionRatesLabels;
            rapidjson::Value accurate;
            rapidjson::Value notAccurate;
            rapidjson::Value rate;
            detectionRatesLabels.SetArray();
            accurate.SetArray();
            notAccurate.SetArray();
            rate.SetArray();
            for (const auto &label : mDetectionAccuracyLabels) {
                rapidjson::Value v;
                v.SetString(label.c_str(), label.length(), allocator);
                detectionRatesLabels.PushBack(v, allocator);
            }
            for (const auto &dAData : mDetectionAccuracyData) {
                accurate.PushBack(std::get<0>(dAData), allocator);
                notAccurate.PushBack(std::get<1>(dAData) * -1, allocator);
                rate.PushBack(std::get<2>(dAData), allocator);
            }
            detectionRatesData.AddMember("labels", detectionRatesLabels, allocator);
            detectionRatesData.AddMember("accurate", accurate, allocator);
            detectionRatesData.AddMember("notAccurate", notAccurate, allocator);
            detectionRatesData.AddMember("rate", rate, allocator);
            d.AddMember("detectionRatesData", detectionRatesData, allocator);

            rapidjson::StringBuffer strBuf;
            rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(strBuf);
            d.Accept(writer);
            std::string jsonString = strBuf.GetString();
            if (mNewReport) {
                std::cout << jsonString << std::endl;
            }
            if (mTotalReportCount == 45) {
                std::cout << "";
            }

            mNewReport = false;
            scheduleAt(simTime() + 2.0, mSelfMsg);
        }
    }


    bool sortByGenerationTime(RecentReported &a,
                              RecentReported &b) {
        return a.lastGenerationTime < b.lastGenerationTime;
    }

    bool sortByStationId(RecentReported &a, RecentReported &b) {
        return a.stationId > b.stationId;
    }

    std::vector<RecentReported> MisbehaviorAuthority::getRecentReported() {
        std::vector<RecentReported> recentReported;
        if (mTotalReportCount == 45) {
            std::cout << "";
        }
        for (auto r : mReportedPseudonyms) {
            auto reportedPseudonym = *r.second;
            if (getActualMisbehaviorType(reportedPseudonym.getStationId()) != misbehaviorTypes::Benign) {
                std::sort(recentReported.begin(), recentReported.end(), sortByGenerationTime);
                if (recentReported.size() < F2MDParameters::misbehaviorAuthorityParameters.recentReportedCount) {
                    recentReported.emplace_back(
                            RecentReported{reportedPseudonym.getStationId(), reportedPseudonym.getReportCount(),
                                           reportedPseudonym.getLastReport()->generationTime});
                } else {
                    if ((*recentReported.begin()).lastGenerationTime <
                        reportedPseudonym.getLastReport()->generationTime) {
                        recentReported.erase(recentReported.begin());
                        recentReported.emplace_back(RecentReported{reportedPseudonym.getStationId(),
                                                                   reportedPseudonym.getReportCount(),
                                                                   reportedPseudonym.getLastReport()->generationTime});
                    }
                }
            }
        }
        std::sort(recentReported.begin(), recentReported.end(), sortByStationId);
        return recentReported;

    }


    void MisbehaviorAuthority::receiveSignal(cComponent *source, simsignal_t signal, const SimTime &,
                                             cObject *) {
        if (signal == traciInitSignal) {
            auto core = check_and_cast<traci::Core *>(source);
            mTraciAPI = core->getAPI();
        } else if (signal == traciCloseSignal) {
            clear();
        } else if (signal == maMisbehaviorAnnouncement) {
            auto misbehaviorCaService = check_and_cast<MisbehaviorCaService *>(source);
            StationID_t stationId = misbehaviorCaService->getStationId();
            mMisbehavingPseudonyms[stationId] =
                    new MisbehavingPseudonym(stationId, misbehaviorCaService->getMisbehaviorType(),
                                             misbehaviorCaService->getAttackType());
        }
    }

    void MisbehaviorAuthority::receiveSignal(cComponent *source, omnetpp::simsignal_t signal, cObject *obj,
                                             cObject *) {
        if (signal == maNewReport) {
            auto *reportObject = dynamic_cast<MisbehaviorReportObject *>(obj);
            const vanetza::asn1::MisbehaviorReport &misbehaviorReport = reportObject->shared_ptr().operator*();
            std::shared_ptr<ma::Report> reportPtr(parseReport(misbehaviorReport));
            if (reportPtr != nullptr) {
                mTotalReportCount++;
                mNewReport = true;

                mReports.emplace(reportPtr->reportId, reportPtr);
                StationID_t reportedStationId;
                if (reportPtr->reportedMessage == nullptr) {
                    reportedStationId = (*reportPtr->relatedReport->referencedReport->reportedMessage)->header.stationID;
                } else {
                    reportedStationId = (*reportPtr->reportedMessage)->header.stationID;
                }
                ReportedPseudonym *reportedPseudonym;
                auto it = mReportedPseudonyms.find(reportedStationId);
                if (it != mReportedPseudonyms.end()) {
                    mReportedPseudonyms[reportedStationId]->addReport(reportPtr);
                    reportedPseudonym = it->second;
                } else {
                    reportedPseudonym = new ReportedPseudonym(reportPtr);
                    mReportedPseudonyms.emplace(reportedStationId, reportedPseudonym);
                }
                updateDetectionRates(*reportedPseudonym, *reportPtr);
            }
        }
    }

    misbehaviorTypes::MisbehaviorTypes MisbehaviorAuthority::getActualMisbehaviorType(const StationID_t &stationId) {
        auto it = mMisbehavingPseudonyms.find(stationId);
        if (it != mMisbehavingPseudonyms.end()) {
            return (*it).second->getMisbehaviorType();
        } else {
            return misbehaviorTypes::Benign;
        }
    }

    void MisbehaviorAuthority::updateDetectionRates(ReportedPseudonym &reportedPseudonym, const ma::Report &report) {

        misbehaviorTypes::MisbehaviorTypes predictedMisbehaviorType =
                reportedPseudonym.predictMisbehaviorType();
        misbehaviorTypes::MisbehaviorTypes predictedMisbehaviorTypeAggregated =
                reportedPseudonym.predictMisbehaviorTypeAggregate();

        misbehaviorTypes::MisbehaviorTypes actualMisbehaviorType;
        if (predictedMisbehaviorType == getActualMisbehaviorType(reportedPseudonym.getStationId())) {
            mTrueDetectionCount++;
        } else {
            mFalseDetectionCount++;
        }
        mDetectionRate = 100 * mTrueDetectionCount / (double) (mTrueDetectionCount + mFalseDetectionCount);
        if (predictedMisbehaviorTypeAggregated == getActualMisbehaviorType(reportedPseudonym.getStationId())) {
            mTrueDetectionAggregateCount++;
        } else {
            mFalseDetectionAggregateCount++;
        }

        mDetectionRateAggregate = 100 * mTrueDetectionAggregateCount /
                                  ((double) mTrueDetectionAggregateCount + mFalseDetectionAggregateCount);
        if (report.generationTime - mLastUpdateTime >
            (long) F2MDParameters::misbehaviorAuthorityParameters.updateTimeStep * 1000) {
            mLastUpdateTime = report.generationTime;
            auto time = (std::time_t) (report.generationTime / 1000 + 1072915200 - 5);
            std::tm t_tm = *std::gmtime(&time);
            std::stringstream ss;
            ss << std::put_time(&t_tm, "%d-%m-%Y_%H:%M:%S");
            std::string timeString = ss.str();
//            std::cout << timeString << std::endl;
            mDetectionAccuracyLabels.emplace_back(timeString);
            int trueDetectionCurrent = mTrueDetectionCount - mTrueDetectionCountInst;
            int falseDetectionCurrent = mFalseDetectionCount - mFalseDetectionCountInst;
            if (trueDetectionCurrent + falseDetectionCurrent > 0) {
                mDetectionRateCur = 100 * trueDetectionCurrent / (trueDetectionCurrent + falseDetectionCurrent);
            }
            mTrueDetectionCountInst = mTrueDetectionCount;
            mFalseDetectionCountInst = mFalseDetectionCount;
            auto data = std::make_tuple(trueDetectionCurrent, falseDetectionCurrent, mDetectionRateCur);
            mDetectionAccuracyData.emplace_back(data);
            if (mDetectionAccuracyData.size() > F2MDParameters::misbehaviorAuthorityParameters.displaySteps) {
                mDetectionAccuracyData.pop_front();
                mDetectionAccuracyLabels.pop_front();
            }

        }
    }

    ma::Report *MisbehaviorAuthority::parseReport(const vanetza::asn1::MisbehaviorReport &misbehaviorReport) {
        auto *report = new ma::Report();
        ReportMetadataContainer reportMetadataContainer = misbehaviorReport->reportMetadataContainer;
        long generationTime;
        asn_INTEGER2long(&reportMetadataContainer.generationTime, &generationTime);
        uint64_t currentTime = countTaiMilliseconds(mTimer.getTimeFor(simTime()));
        if (currentTime - generationTime >
            (uint64_t) F2MDParameters::misbehaviorAuthorityParameters.maxReportAge * 1000) {
            return nullptr;
        }
        std::string reportId = ia5stringToString(reportMetadataContainer.reportID);
        std::cout << "received report: " << reportId << " " << generationTime << std::endl;
        if (reportId.empty()) {
            return nullptr;
        }
        report->reportId = reportId;
        report->generationTime = generationTime;

        if (reportMetadataContainer.relatedReportContainer != nullptr) {
            RelatedReportContainer_t relatedReportContainer = *reportMetadataContainer.relatedReportContainer;
            std::string relatedReportId = ia5stringToString(relatedReportContainer.relatedReportID);
            std::cout << "  relatedReportId: " << relatedReportId << " " << std::endl;
            auto it = mReports.find(relatedReportId);
            if (it != mReports.end()) {
                report->relatedReport = new ma::RelatedReport;
                report->relatedReport->referencedReport = it->second;
                report->relatedReport->omittedReportsNumber = relatedReportContainer.omittedReportsNumber;
            } else {
                return nullptr;
            }
        }

        ReportContainer reportContainer = misbehaviorReport->reportContainer;
        if (reportContainer.reportedMessageContainer.present ==
            ReportedMessageContainer_PR_certificateIncludedContainer) {
            EtsiTs103097Data_t reportedMessage = reportContainer.reportedMessageContainer.choice.certificateIncludedContainer.reportedMessage;
            if (reportedMessage.content != nullptr) {
                Ieee1609Dot2Content ieee1609Dot2Content = *reportedMessage.content;
                if (ieee1609Dot2Content.present == Ieee1609Dot2Content_PR_unsecuredData) {
                    auto *cam = (vanetza::asn1::Cam *) ieee1609Dot2Content.choice.unsecuredData.buf;
                    std::cout << "  reported StationID: " << (*cam)->header.stationID << std::endl;
                    std::cout << "  genDeltaTime: " << (*cam)->cam.generationDeltaTime << std::endl;
                    auto camPtr = std::make_shared<vanetza::asn1::Cam>(*cam);
                    auto it = mCams.find(camPtr);
                    if (it == mCams.end()) {
                        mCams.insert(camPtr);
                    } else {
                        camPtr = (*it);
                    }
                    report->reportedMessage = camPtr;
                } else if (report->relatedReport == nullptr) {
                    return nullptr;
                }
            }
        } else {
            std::cout << "ignoring report, only CertificateIncludedContainer is implemented" << std::endl;
            return nullptr;
        }
        if (reportContainer.misbehaviorTypeContainer.present == MisbehaviorTypeContainer_PR_semanticDetection) {
            SemanticDetection_t semanticDetection = reportContainer.misbehaviorTypeContainer.choice.semanticDetection;
            if (semanticDetection.present == SemanticDetection_PR_semanticDetectionReferenceCAM) {
                report->detectionType.present = ma::detectionTypes::SemanticType;
                auto semantic = new ma::SemanticDetection;
                report->detectionType.semantic = semantic;
                semantic->detectionLevel = (int) semanticDetection.choice.semanticDetectionReferenceCAM.detectionLevelCAM;
                semantic->errorCode = (std::bitset<16>) semanticDetection.choice.semanticDetectionReferenceCAM.semanticDetectionErrorCodeCAM.buf;

                std::cout << "  detection level: "
                          << detectionLevels::DetectionLevelStrings[semantic->detectionLevel]
                          << std::endl;
                std::cout << "  " << semantic->errorCode.to_string() << std::endl;

                switch (semantic->detectionLevel) {
                    case detectionLevels::Level1: {
                        break;
                    }
                    case detectionLevels::Level2: {
                        if (reportContainer.evidenceContainer == nullptr) {
                            std::cout << "invalid report, evidenceContainer missing!" << std::endl;
                            return nullptr;
                        }
                        if (reportContainer.evidenceContainer->reportedMessageContainer == nullptr) {
                            std::cout
                                    << "invalid report, reportedMessageContainer (messageEvidenceContainer) missing!"
                                    << std::endl;
                            return nullptr;
                        }
                        auto *reportedMessageContainer = reportContainer.evidenceContainer->reportedMessageContainer;
                        if (reportedMessageContainer->list.count == 0) {
                            std::cout << "invalid report, reportedMessageContainer is empty" << std::endl;
                            return nullptr;
                        }
                        std::cout << "  evidenceCams: " << std::endl;
                        parseMessageEvidenceContainer(*reportedMessageContainer, report->evidence.reportedMessages);
                        break;
                    }
                    case detectionLevels::Level3: {
                        if (reportContainer.evidenceContainer == nullptr) {
                            std::cout << "invalid report, evidenceContainer missing!" << std::endl;
                            return nullptr;
                        }
                        if (reportContainer.evidenceContainer->neighbourMessageContainer != nullptr) {
                            auto *neighbourMessageContainer = reportContainer.evidenceContainer->neighbourMessageContainer;
                            if (neighbourMessageContainer->list.count == 0) {
                                std::cout << "neighbourMessageContainer is empty" << std::endl;
                            } else {
                                std::cout << "  neighbourCams: " << std::endl;
                                parseMessageEvidenceContainer(*neighbourMessageContainer,
                                                              report->evidence.neighbourMessages);
                            }
//                            std::cout
//                                    << "invalid report, neighbourMessageContainer (messageEvidenceContainer) missing!"
//                                    << std::endl;
//                            return nullptr;
                        }
                    }
                    case detectionLevels::Level4: {
                        std::cout << "Nothing to do, DetectionLevel 4 not implemented" << std::endl;
//                        return nullptr;
                    }
                }
            }
        } else {
            std::cout << "Nothing to do, only SemanticDetection is implemented" << std::endl;
//            return nullptr;

        }
        return report;
    }

    void MisbehaviorAuthority::parseMessageEvidenceContainer(const MessageEvidenceContainer &messageEvidenceContainer,
                                                             std::vector<std::shared_ptr<vanetza::asn1::Cam>> &messages) {
        for (int i = 0; i < messageEvidenceContainer.list.count; i++) {
            auto *ieee1609Dot2Content = ((EtsiTs103097Data_t *) messageEvidenceContainer.list.array[i])->content;
            if (ieee1609Dot2Content->present == Ieee1609Dot2Content_PR_unsecuredData) {
                auto *evidenceCam = (vanetza::asn1::Cam *) ieee1609Dot2Content->choice.unsecuredData.buf;
                std::cout << "    previous genDeltaTime: " << (*evidenceCam)->cam.generationDeltaTime
                          << std::endl;

                auto camPtr = std::make_shared<vanetza::asn1::Cam>(*evidenceCam);
                auto it = mCams.find(camPtr);
                if (it == mCams.end()) {
                    mCams.insert(camPtr);
                } else {
                    camPtr = (*it);
                }
                messages.emplace_back(camPtr);
            } else {
                std::cout << "ignoring CAM, only unsigned CAMs can be processed" << std::endl;
            }
        }
    }

} // namespace artery