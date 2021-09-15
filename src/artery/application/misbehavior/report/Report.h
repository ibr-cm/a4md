//
// Created by bastian on 29.07.21.
//

#ifndef ARTERY_REPORT_H
#define ARTERY_REPORT_H

#include <string>
#include "vanetza/asn1/cam.hpp"
#include <bitset>
#include "artery/application/misbehavior/util/DetectionLevels.h"
#include "artery/application/misbehavior/report/ReportedPseudonym.h"
#include "artery/application/misbehavior/report/ReportingPseudonym.h"
#include "artery/application/misbehavior/report/ReportSummary.h"
#include "artery/application/misbehavior/util/HelperFunctions.h"

namespace artery {

    class ReportedPseudonym;

    class ReportingPseudonym;

    class Report;

    struct SemanticDetection {
        detectionLevels::DetectionLevels detectionLevel;
        std::bitset<16> errorCode;
    };

    struct SecurityDetection {
        std::bitset<32> errorCode;
    };

    namespace detectionTypes {
        enum DetectionTypes {
            none,
            SemanticType,
            SecurityType
        };
    }

    struct DetectionType {
        detectionTypes::DetectionTypes present = detectionTypes::none;
        SemanticDetection *semantic = nullptr;
        SecurityDetection *security = nullptr;
    };

    struct RelatedReport {
        std::string referencedReportId;
        long omittedReportsNumber;
    };

    struct Evidence {
        std::vector<std::shared_ptr<vanetza::asn1::Cam>> reportedMessages;
        std::vector<std::shared_ptr<vanetza::asn1::Cam>> neighbourMessages;
        SenderInfoContainer_t *senderInfo = nullptr;
        SenderSensorContainer_t *senderSensors = nullptr;
    };


    class Report {
    public:
        Report(const vanetza::asn1::MisbehaviorReport &misbehaviorReport,
               std::map<std::string, std::shared_ptr<ma::ReportSummary>> &reportSummaryList);

        Report(std::string reportId, std::shared_ptr<vanetza::asn1::Cam> cam, const uint64_t &generationTime);

        ~Report();

        std::string reportId;
        uint64_t generationTime;
        std::shared_ptr<vanetza::asn1::Cam> reportedMessage;
        std::shared_ptr<ReportedPseudonym> reportedPseudonym;
        std::shared_ptr<ReportingPseudonym> reportingPseudonym;
        DetectionType detectionType;
        std::shared_ptr<RelatedReport> relatedReport;
        Evidence evidence;
        bool isValid;
        bool successfullyParsed;
        double score;

        void setSemanticDetection(const detectionLevels::DetectionLevels &detectionLevel,
                                  const std::bitset<16> &errorCode);

        void setReportedMessages(const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &cams, const int &maxCamCount);

        void setRelatedReport(const std::string &relatedReportId, const long &omittedReportsNumber);


        void fillSenderInfoContainer(const VehicleDataProvider *vehicleDataProvider,
                                     const traci::VehicleController *vehicleController);

        vanetza::asn1::MisbehaviorReport encode();

    private:
        static void decodeMessageEvidenceContainer(const MessageEvidenceContainer &messageEvidenceContainer,
                                                   std::vector<std::shared_ptr<vanetza::asn1::Cam>> &messages);

        static std::shared_ptr<vanetza::asn1::Cam> getCamFromOpaque(const Opaque_t &data);
    };

} // namespace artery

#endif //ARTERY_REPORT_H
