//
// Created by bastian on 25.08.21.
//

#include "ReportingPseudonym.h"

namespace artery {

    ReportingPseudonym::ReportingPseudonym(StationID_t stationId) : mStationId(stationId) {

    }

    void ReportingPseudonym::addReport(const Report &report) {
        mTotalReportCount++;
        mTotalReportScore += report.score;
    }

    double ReportingPseudonym::getAverageReportScore() const {
        if (mTotalReportCount > 0) {
            return mTotalReportScore / mTotalReportCount;
        } else {
            return 1;
        }
    }
} // namespace artery