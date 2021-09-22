//
// Created by bastian on 25.08.21.
//

#ifndef ARTERY_REPORTINGPSEUDONYM_H
#define ARTERY_REPORTINGPSEUDONYM_H

#include "vanetza/asn1/md/StationID.h"
#include "artery/application/misbehavior/report/Report.h"
#include <omnetpp.h>

namespace artery {

    class Report;

    class ReportingPseudonym {
    public:
        explicit ReportingPseudonym(StationID_t stationId);

        void addReport(const std::shared_ptr<Report> &report);

        void recordStatistics();

        double getAverageReportScore() const;

        StationID_t getStationId() const;

    private:
        StationID_t mStationId;
        int mTotalReportCount = 0;
        double mTotalReportScore = 0;

        omnetpp::cHistogram statsReportedPseudonym;
        omnetpp::cStdDev statsValidity;
        omnetpp::cStdDev statsScore;
        omnetpp::cOutVector vectorScore;
        omnetpp::cHistogram statsDetectionType;
        omnetpp::cHistogram statsDetectionLevel;
        omnetpp::cHistogram statsErrorCode;
    };

} // namespace artery


#endif //ARTERY_REPORTINGPSEUDONYM_H
