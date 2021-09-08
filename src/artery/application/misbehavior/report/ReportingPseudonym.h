//
// Created by bastian on 25.08.21.
//

#ifndef ARTERY_REPORTINGPSEUDONYM_H
#define ARTERY_REPORTINGPSEUDONYM_H

#include "vanetza/asn1/md/StationID.h"
#include "artery/application/misbehavior/report/Report.h"

namespace artery {

    class Report;

    class ReportingPseudonym {
    public:
        explicit ReportingPseudonym(StationID_t stationId);

        void addReport(const Report &report);

        double getAverageReportScore() const;

        StationID_t getStationId() const;

    private:
        StationID_t mStationId;
        int mTotalReportCount = 0;
        double mTotalReportScore = 0;
    };

} // namespace artery


#endif //ARTERY_REPORTINGPSEUDONYM_H
