//
// Created by bastian on 15.06.21.
//

#include "DetectedSender.h"
#include "artery/application/misbehavior/util/HelperFunctions.h"
#include "artery/application/misbehavior/util/CheckTypes.h"
#include <artery/application/misbehavior/checks/LegacyChecks.h>
#include <artery/application/misbehavior/checks/CatchChecks.h>

namespace artery {
    DetectedSender::DetectedSender(const std::shared_ptr<const traci::API> &traciAPI,
                                   GlobalEnvironmentModel *globalEnvironmentModel,
                                   DetectionParameters *detectionParameters,
                                   const Timer *timer,
                                   const shared_ptr<vanetza::asn1::Cam> &message) {
        mStationId = (*message)->header.stationID;
        mHasBeenReported = false;
        mOmittedReportsPerErrorCode = std::vector<int>(16);
        mOmittedReportsCumulated = 0;
        mCamsArraySize = detectionParameters->detectedSenderCamArrayMaxSize;
        switch (detectionParameters->checkType) {
            case checkTypes::LegacyChecks:
                baseChecks = new LegacyChecks(traciAPI, globalEnvironmentModel, detectionParameters, timer, message);
                break;
            case checkTypes::CatchChecks:
                baseChecks = new CatchChecks(traciAPI, globalEnvironmentModel, detectionParameters, timer, message);
        }
    }


    DetectedSender::~DetectedSender() {
        mCams.clear();
        delete baseChecks;
    }


    std::shared_ptr<CheckResult>
    DetectedSender::addAndCheckCam(const shared_ptr<vanetza::asn1::Cam> &message,
                                   const VehicleDataProvider *receiverVDP,
                                   const std::vector<Position> &receiverVehicleOutline,
                                   const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &relevantCams) {
        std::shared_ptr<CheckResult> result;
        if (!mCams.empty()) {
            result = baseChecks->checkCAM(receiverVDP, receiverVehicleOutline, message,
                                          mCams.back(), relevantCams);
        } else {
            result = baseChecks->checkCAM(receiverVDP, receiverVehicleOutline, message, nullptr,
                                          relevantCams);
        }
        mCams.emplace_back(message);
        if (mCams.size() > mCamsArraySize) {
            mCams.pop_front();
        }
        return result;
    }


    bool DetectedSender::checkOmittedReportsLimit(const bitset<16> &reportedErrorCodes) {
        if (!mHasBeenReported) {
            return true;
        }
        if (F2MDParameters::reportParameters.omittedReportsCountPerErrorCode) {
            bool omittedLimitReached = false;
            for (int i = 0; i < mOmittedReportsPerErrorCode.size(); i++) {
                std::cout << mOmittedReportsPerErrorCode[15 - i];
                if (mOmittedReportsPerErrorCode[i] >= F2MDParameters::reportParameters.omittedReportsCount &&
                    reportedErrorCodes[i]) {
                    omittedLimitReached = true;
                }
            }
            return omittedLimitReached;
        } else {
            if (mOmittedReportsCumulated >= F2MDParameters::reportParameters.omittedReportsCount) {
                return true;
            }
        }
        return false;
    }

    void DetectedSender::resetOmittedReports(const bitset<16> &reportedErrorCodes) {
        if (F2MDParameters::reportParameters.omittedReportsCountPerErrorCode) {
            for (int i = 0; i < mOmittedReportsPerErrorCode.size(); i++) {
                if (mOmittedReportsPerErrorCode[i] >= F2MDParameters::reportParameters.omittedReportsCount &&
                    reportedErrorCodes[i]) {
                    mOmittedReportsPerErrorCode[i] = 0;
                }
                std::cout << mOmittedReportsPerErrorCode[15 - i];
            }
            std::cout << std::endl;
        } else {
            if (mOmittedReportsCumulated >= F2MDParameters::reportParameters.omittedReportsCount) {
                mOmittedReportsCumulated = 0;
            }
        }
    }

    void DetectedSender::incrementOmittedReports(const std::vector<std::bitset<16>> &detectionLevelErrorCodes,
                                                 const std::bitset<16> &reportedErrorCodes) {
        std::bitset<16> errorCodesCumulated;
        for (auto errorCode: detectionLevelErrorCodes) {
            errorCodesCumulated |= errorCode;
        }
        if (F2MDParameters::reportParameters.omittedReportsCountPerErrorCode) {
            for (int i = 0; i < mOmittedReportsPerErrorCode.size(); i++) {
                if (errorCodesCumulated[i] && !reportedErrorCodes[i]) {
                    mOmittedReportsPerErrorCode[i]++;
                }
                std::cout << mOmittedReportsPerErrorCode[15 - i];
            }
            std::cout << std::endl;
        } else {
            if (errorCodesCumulated.any() && reportedErrorCodes.none()) {
                mOmittedReportsCumulated++;
            }
        }
    }


} //namespace artery

