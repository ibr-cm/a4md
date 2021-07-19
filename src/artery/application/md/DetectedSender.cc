//
// Created by bastian on 15.06.21.
//

#include "DetectedSender.h"
#include "artery/application/md/util/HelperFunctions.h"
#include "artery/application/md/util/CheckTypes.h"
#include <artery/application/md/checks/LegacyChecks.h>
#include <artery/application/md/checks/CatchChecks.h>

namespace artery {
    DetectedSender::DetectedSender(const std::shared_ptr<const traci::API> &traciAPI,
                                   GlobalEnvironmentModel *globalEnvironmentModel,
                                   DetectionParameters *detectionParameters,
                                   const vanetza::asn1::Cam &message) {
        mStationId = message->header.stationID;
        switch (detectionParameters->checkType) {
            case checkTypes::LegacyChecks:
                baseChecks = new LegacyChecks(traciAPI, globalEnvironmentModel, detectionParameters, message);
                break;
            case checkTypes::CatchChecks:
                baseChecks =new CatchChecks(traciAPI, globalEnvironmentModel, detectionParameters, message);
        }
    }


    CheckResult *
    DetectedSender::addAndCheckCam(const vanetza::asn1::Cam &message, const VehicleDataProvider *receiverVDP,
                                   const std::vector<Position> &receiverVehicleOutline,
                                   const std::vector<vanetza::asn1::Cam *> &relevantCams) {
        CheckResult *result;
        if (!checkResults.empty()) {
            result = baseChecks->checkCAM(receiverVDP, receiverVehicleOutline, message,
                                          &checkResults.back()->cam, relevantCams);
        } else {
            result = baseChecks->checkCAM(receiverVDP, receiverVehicleOutline, message, nullptr,
                                          relevantCams);
        }
        result->cam = message;
        checkResults.push_back(result);
        return result;
    }

} //namespace artery

