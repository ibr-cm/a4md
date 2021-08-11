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
#include "artery/application/misbehavior/util/CheckTypes.h"
#include "artery/application/misbehavior/fusion/ThresholdFusion.h"
#include <artery/application/misbehavior/checks/LegacyChecks.h>
#include <artery/application/misbehavior/checks/CatchChecks.h>
#include <bitset>
#include <chrono>
#include <numeric>

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

        mFusionApplication = new ThresholdFusion(0.5);

        F2MDParameters::misbehaviorAuthorityParameters.maxReportAge = par("maxReportAge");
        F2MDParameters::misbehaviorAuthorityParameters.reportCountThreshold = par("reportCountThreshold");
        F2MDParameters::misbehaviorAuthorityParameters.checkType = par("checkType");
        F2MDParameters::misbehaviorAuthorityParameters.updateTimeStep = par("updateTimeStep");
        F2MDParameters::misbehaviorAuthorityParameters.enableWebGui = par("enableWebGui");
        F2MDParameters::misbehaviorAuthorityParameters.webGuiDataUrl = par("webGuiDataUrl").stdstringValue();
        F2MDParameters::misbehaviorAuthorityParameters.guiJsonDataUpdateInterval = par("guiJsonDataUpdateInterval");
        F2MDParameters::misbehaviorAuthorityParameters.displaySteps = par("displaySteps");
        F2MDParameters::misbehaviorAuthorityParameters.recentReportedCount = par("recentReportedCount");

        if (F2MDParameters::misbehaviorAuthorityParameters.enableWebGui) {
            mSelfMsg = new cMessage("getDataScheduleMessage");
            scheduleAt(simTime() + F2MDParameters::misbehaviorAuthorityParameters.guiJsonDataUpdateInterval, mSelfMsg);
        }
    }

    void MisbehaviorAuthority::handleMessage(omnetpp::cMessage *msg) {
        Enter_Method("handleMessage");
        if (msg == mSelfMsg) {
            createGuiJsonData();
            scheduleAt(simTime() + F2MDParameters::misbehaviorAuthorityParameters.guiJsonDataUpdateInterval, mSelfMsg);
        }
    }

    void MisbehaviorAuthority::receiveSignal(cComponent *source, simsignal_t signal, const SimTime &,
                                             cObject *) {
        if (signal == traciInitSignal) {
            auto core = check_and_cast<traci::Core *>(source);
            mTraciAPI = core->getAPI();
            switch (F2MDParameters::misbehaviorAuthorityParameters.checkType) {
                case checkTypes::LegacyChecks:
                    mBaseChecks = new LegacyChecks(mTraciAPI, mGlobalEnvironmentModel,
                                                   &F2MDParameters::detectionParameters, &mTimer);
                    break;
                case checkTypes::CatchChecks:
                    mBaseChecks = new CatchChecks(mTraciAPI, mGlobalEnvironmentModel,
                                                  &F2MDParameters::detectionParameters, &mTimer);
            }
        } else if (signal == traciCloseSignal) {
            clear();
        }
    }

    void MisbehaviorAuthority::updateReactionType(ReportedPseudonym &reportedPseudonym) {
        size_t reportCount = reportedPseudonym.getReportCount();
        reactionTypes::ReactionTypes newReactionType = reactionTypes::Nothing;
        if (reportCount > 20) {
            newReactionType = reactionTypes::Warning;
        } else if (reportCount > 50) {
            newReactionType = reactionTypes::Ticket;
        } else if (reportCount > 250) {
            newReactionType = reactionTypes::PassiveRevocation;
        } else if (reportCount > 1250) {
            newReactionType = reactionTypes::ActiveRevocation;
        }
        reactionTypes::ReactionTypes oldReactionType = reportedPseudonym.getReactionType();
        if (newReactionType != oldReactionType) {
            mReactionsList[oldReactionType].erase(reportedPseudonym.getStationId());
            mReactionsList[newReactionType].insert(reportedPseudonym.getStationId());
            reportedPseudonym.setReactionType(newReactionType);
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
                if (validateReport(*reportPtr)) {
                    updateReactionType(*reportedPseudonym);
                    updateDetectionRates(*reportedPseudonym, *reportPtr);
                } else {
                    std::cout << "######### report validation failed" << std::endl;
                }
            }
        } else if (signal == maMisbehaviorAnnouncement) {
            std::vector<StationID_t> stationIds = *reinterpret_cast<std::vector<StationID_t> *>(obj);
            auto misbehaviorCaService = check_and_cast<MisbehaviorCaService *>(source);
            for (const auto &stationId : stationIds) {
                mMisbehavingPseudonyms[stationId] = new MisbehavingPseudonym(stationId,
                                                                             misbehaviorCaService->getMisbehaviorType(),
                                                                             misbehaviorCaService->getAttackType());
            }
        }
    }

    bool compareErrorCodes(std::bitset<16> reportedErrorCodes, std::bitset<16> actualErrorCodes) {
        for (int i = 0; i < actualErrorCodes.size(); i++) {
            if (reportedErrorCodes[i] == 1 && actualErrorCodes[i] == 0) {
                return false;
            }
        }
        return true;
    }

    bool MisbehaviorAuthority::validateSemanticLevel1Report(const ma::Report &report) {
        std::bitset<16> actualErrorCodes = mBaseChecks->checkSemanticLevel1Report(*report.reportedMessage);
        std::bitset<16> reportedErrorCodes = report.detectionType.semantic->errorCode;
        return compareErrorCodes(reportedErrorCodes, actualErrorCodes);
    }

    bool MisbehaviorAuthority::validateSemanticLevel2Report(const ma::Report &report) {
        std::bitset<16> actualErrorCodes;
        std::bitset<16> reportedErrorCodes = report.detectionType.semantic->errorCode;
        std::vector<std::shared_ptr<vanetza::asn1::Cam>> reportedMessages = report.evidence.reportedMessages;
        for (int i = 0; i < reportedMessages.size() - 1; i++) {
            actualErrorCodes |= mBaseChecks->checkSemanticLevel2Report(*reportedMessages[i],
                                                                       *reportedMessages[i + 1]);
        }
        actualErrorCodes |= mBaseChecks->checkSemanticLevel2Report(*reportedMessages.back(),
                                                                   *report.reportedMessage);
        return compareErrorCodes(reportedErrorCodes, actualErrorCodes);
    }

    bool MisbehaviorAuthority::validateSemanticLevel3Report(const ma::Report &report) {
        std::bitset<16> actualErrorCodes = mBaseChecks->checkSemanticLevel3Report(*report.reportedMessage,
                                                                                  report.evidence.neighbourMessages);
        std::bitset<16> reportedErrorCodes = report.detectionType.semantic->errorCode;
        std::vector<std::shared_ptr<vanetza::asn1::Cam>> neighbourMessages = report.evidence.neighbourMessages;

        return compareErrorCodes(reportedErrorCodes, actualErrorCodes);
    }

    bool MisbehaviorAuthority::validateReport(const ma::Report &report) {
        if (report.detectionType.semantic != nullptr) {
            switch (report.detectionType.semantic->detectionLevel) {
                case detectionLevels::Level1:
                    return validateSemanticLevel1Report(report);
                case detectionLevels::Level2:
                    return validateSemanticLevel2Report(report);
                case detectionLevels::Level3:
                    return validateSemanticLevel3Report(report);
                case detectionLevels::Level4:
                    break;
                default:
                    break;
            }
        }
        return false;
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
            ss << std::put_time(&t_tm, "%H:%M:%S");
            std::string timeString = ss.str();
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
                semantic->detectionLevel = static_cast<detectionLevels::DetectionLevels>(semanticDetection.choice.semanticDetectionReferenceCAM.detectionLevelCAM);
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

    rapidjson::Value MisbehaviorAuthority::getRadarData(rapidjson::Document::AllocatorType &allocator) {
        rapidjson::Value reactionsData(rapidjson::kObjectType);
        std::vector<int> reactionsBenign(5, 0);
        std::vector<int> reactionsMalicious(5, 0);

        for (auto reportedPseudonym : mReportedPseudonyms) {
            misbehaviorTypes::MisbehaviorTypes misbehaviorType = getActualMisbehaviorType(
                    reportedPseudonym.second->getStationId());
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
        return reactionsData;
    }

    rapidjson::Value MisbehaviorAuthority::getRecentReported(rapidjson::Document::AllocatorType &allocator) {
        rapidjson::Value recentlyReportedData(rapidjson::kObjectType);
        rapidjson::Value labels;
        rapidjson::Value data;
        labels.SetArray();
        data.SetArray();
        for (auto r : getRecentReported()) {
            labels.PushBack(r.stationId, allocator);
            data.PushBack(r.reportCount, allocator);
        }
        recentlyReportedData.AddMember("labels", labels, allocator);
        recentlyReportedData.AddMember("data", data, allocator);
        return recentlyReportedData;
    }

    rapidjson::Value MisbehaviorAuthority::getDetectionRates(rapidjson::Document::AllocatorType &allocator) {
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
        return detectionRatesData;
    }

    void MisbehaviorAuthority::createGuiJsonData() {
        rapidjson::Document d;
        d.SetObject();
        rapidjson::Document::AllocatorType &allocator = d.GetAllocator();

        d.AddMember("newReport", mNewReport, allocator);
        d.AddMember("totalReports", mTotalReportCount, allocator);
        d.AddMember("cumulativeDetectionRate", mDetectionRate, allocator);

        d.AddMember("reactionsData", getRadarData(allocator), allocator);
        d.AddMember("recentlyReportedData", getRecentReported(allocator), allocator);
        d.AddMember("detectionRatesData", getDetectionRates(allocator), allocator);

        rapidjson::StringBuffer strBuf;
        rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
        d.Accept(writer);
        std::string jsonString = strBuf.GetString();
        curl_easy_setopt(curl, CURLOPT_URL, F2MDParameters::misbehaviorAuthorityParameters.webGuiDataUrl.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonString.c_str());
        CURLcode curlResponse = curl_easy_perform(curl);
        if (curlResponse != CURLE_OK) {
            std::cout << "request failed: " << curl_easy_strerror(curlResponse) << std::endl;
        }
        mNewReport = false;
    }

    bool sortByGenerationTime(RecentReported &a, RecentReported &b) {
        return a.lastGenerationTime < b.lastGenerationTime;
    }

    bool sortByStationId(RecentReported &a, RecentReported &b) {
        return a.stationId > b.stationId;
    }

    std::vector<RecentReported> MisbehaviorAuthority::getRecentReported() {
        std::vector<RecentReported> recentReported;
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

    void MisbehaviorAuthority::printReportsPerPseudonym() {
        int attackerCount = 0;
        int benignCount = 0;
        int reportTpCount = 0;
        int reportFpCount = 0;
        double meanReportsPerAttacker = 0;
        double meanReportsPerBenign = 0;
        double reportTpSdSum = 0;
        double reportFpSdSum = 0;
        double sdAttacker = 0;
        double sdBenign = 0;
        for (auto r : mReportedPseudonyms) {
            ReportedPseudonym reportedPseudonym = *r.second;
            if (getActualMisbehaviorType(reportedPseudonym.getStationId()) != misbehaviorTypes::Benign) {
                attackerCount++;
                reportTpCount += (int) reportedPseudonym.getReportCount();
            } else {
                benignCount++;
                reportFpCount += (int) reportedPseudonym.getReportCount();
            }
        }

        if (reportTpCount > 0) {
            meanReportsPerAttacker = (double) reportTpCount / attackerCount;
        }
        if (reportFpCount > 0) {
            meanReportsPerBenign = (double) reportFpCount / benignCount;
        }
        for (auto r : mReportedPseudonyms) {
            ReportedPseudonym reportedPseudonym = *r.second;
            if (getActualMisbehaviorType(reportedPseudonym.getStationId()) != misbehaviorTypes::Benign) {
                reportTpSdSum += pow((int) reportedPseudonym.getReportCount() - meanReportsPerAttacker, 2);
            } else {
                reportFpSdSum += pow((int) reportedPseudonym.getReportCount() - meanReportsPerBenign, 2);
            }
        }
        if (reportTpCount > 0) {
            sdAttacker = sqrt(reportTpSdSum / attackerCount);
        }
        if (reportFpCount > 0) {
            sdBenign = sqrt(reportFpSdSum / benignCount);
        }
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Reports per malicious pseudonym: " << meanReportsPerAttacker << " StdDev: " << sdAttacker
                  << std::endl;
        std::cout << "Reports per benign pseudonym: " << meanReportsPerBenign << " StdDev: " << sdBenign << std::endl;
    }


} // namespace artery