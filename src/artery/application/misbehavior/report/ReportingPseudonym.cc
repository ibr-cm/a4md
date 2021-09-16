//
// Created by bastian on 25.08.21.
//

#include "ReportingPseudonym.h"

namespace artery {

    ReportingPseudonym::ReportingPseudonym(StationID_t stationId) : mStationId(stationId) {

    }

    void ReportingPseudonym::addReport(const std::shared_ptr<Report> &report) {
        mTotalReportScore += report->score / getAverageReportScore();
        mTotalReportCount++;
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