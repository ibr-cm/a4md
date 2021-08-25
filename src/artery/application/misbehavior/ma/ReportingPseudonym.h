//
// Created by bastian on 25.08.21.
//

#ifndef ARTERY_REPORTINGPSEUDONYM_H
#define ARTERY_REPORTINGPSEUDONYM_H
#include "vanetza/asn1/md/StationID.h"

namespace artery {
class ReportingPseudonym {
private:
    StationID_t mStationId;
    int mTotalReportCount = 0;
    double mTotalReportScore = 0;
public:
    explicit ReportingPseudonym(StationID_t stationId);

    void addReport(const ma::Report& report);

    double getAverageReportScore() const;
};

} // namespace artery


#endif //ARTERY_REPORTINGPSEUDONYM_H
