//
// Created by bastian on 29.07.21.
//

#ifndef ARTERY_REPORT_H
#define ARTERY_REPORT_H

#include <string>
#include "vanetza/asn1/cam.hpp"
#include <bitset>
#include "artery/application/misbehavior/util/DetectionLevels.h"

namespace artery {

    namespace ma {

        struct Report;
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
            SemanticDetection *semantic;
            SecurityDetection *security;
        };

        struct RelatedReport {
            std::shared_ptr<Report> referencedReport;
            long omittedReportsNumber;
        };

        struct Evidence {
            std::vector<std::shared_ptr<vanetza::asn1::Cam>> reportedMessages;
            std::vector<std::shared_ptr<vanetza::asn1::Cam>> neighbourMessages;
            std::shared_ptr<SenderInfoContainer_t> senderInfo;
            std::shared_ptr<SenderSensorContainer_t> senderSensors;
        };


        struct Report {
            std::string reportId;
            uint64_t generationTime;
            std::shared_ptr<vanetza::asn1::Cam> reportedMessage;
            DetectionType detectionType;
            RelatedReport *relatedReport;
            Evidence evidence;
            bool isValid;
        };
    }

} // namespace artery

#endif //ARTERY_REPORT_H
