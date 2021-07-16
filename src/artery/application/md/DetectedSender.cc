//
// Created by bastian on 15.06.21.
//

#include "DetectedSender.h"
#include "artery/application/md/util/HelperFunctions.h"

namespace artery {
    DetectedSender::DetectedSender(const std::shared_ptr<const traci::API> &traciAPI,
                                   GlobalEnvironmentModel *globalEnvironmentModel,
                                   DetectionParameters *detectionParameters,
                                   const vanetza::asn1::Cam &message)
            : legacyChecks(traciAPI, globalEnvironmentModel, detectionParameters, message) {
        mStationId = message->header.stationID;
    }


    CheckResult *
    DetectedSender::addAndCheckCam(const vanetza::asn1::Cam &message, const VehicleDataProvider *receiverVDP,
                                   const std::vector<Position> &receiverVehicleOutline,
                                   const std::vector<vanetza::asn1::Cam *> &relevantCams) {
        CheckResult *result;
        if (!checkResults.empty()) {
            result = legacyChecks.checkCAM(receiverVDP, receiverVehicleOutline, message,
                                           &checkResults.back()->cam, relevantCams);
        } else {
            result = legacyChecks.checkCAM(receiverVDP, receiverVehicleOutline, message, nullptr,
                                           relevantCams);
        }
        result->cam = message;
        checkResults.push_back(result);
        return result;
    }

} //namespace artery
