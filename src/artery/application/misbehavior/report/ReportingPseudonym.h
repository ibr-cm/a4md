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

        double getAverageReportScore() const;

        StationID_t getStationId() const;

        omnetpp::simsignal_t signalReportReportedPseudonym;
        omnetpp::simsignal_t signalReportValidity;
        omnetpp::simsignal_t signalReportScore;
        omnetpp::simsignal_t signalReportDetectionType;
        omnetpp::simsignal_t signalReportDetectionLevel;
        omnetpp::simsignal_t signalReportErrorCode;
        omnetpp::simsignal_t signalAverageReportScore;

    private:
        StationID_t mStationId;
        int mTotalReportCount = 0;
        double mTotalReportScore = 0;
    };

} // namespace artery


#endif //ARTERY_REPORTINGPSEUDONYM_H
