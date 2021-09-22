//
// Created by bastian on 25.08.21.
//

#include "ReportingPseudonym.h"

namespace artery {

    ReportingPseudonym::ReportingPseudonym(StationID_t stationId) : mStationId(stationId) {

        std::string prefix = {"reportingPseudonym_" + std::to_string(mStationId) + "_"};
        statsReportedPseudonym.setName((prefix + "reportedPseudonym").c_str());
        statsValidity.setName((prefix + "validity").c_str());
        statsScore.setName((prefix + "score_hist").c_str());
        vectorScore.setName((prefix + "score_vec").c_str());
        statsDetectionType.setName((prefix + "detectionType").c_str());
        statsDetectionLevel.setName((prefix + "detectionLevel").c_str());
        statsErrorCode.setName((prefix + "errorCode").c_str());
    }

    void ReportingPseudonym::addReport(const std::shared_ptr<Report> &report) {
        mTotalReportScore += report->score / getAverageReportScore();
        mTotalReportCount++;

        statsReportedPseudonym.collect(report->reportingPseudonym->getStationId());
        statsValidity.collect(report->isValid);
        statsScore.collect(report->score);
        vectorScore.record(report->score);
        statsDetectionType.collect(report->detectionType.present);
        statsDetectionLevel.collect(report->detectionType.semantic->detectionLevel);
        statsErrorCode.collect(report->detectionType.semantic->errorCode.to_ulong());
    }

    void ReportingPseudonym::recordStatistics() {
        statsReportedPseudonym.record();
        statsValidity.record();
        statsScore.record();
        statsDetectionType.record();
        statsDetectionLevel.record();
        statsErrorCode.record();
    }

    double ReportingPseudonym::getAverageReportScore() const {
        if (mTotalReportCount > 0) {
            return mTotalReportScore / mTotalReportCount;
        } else {
            return 1;
        }
    }

    StationID_t ReportingPseudonym::getStationId() const {
        return mStationId;
    }
} // namespace artery