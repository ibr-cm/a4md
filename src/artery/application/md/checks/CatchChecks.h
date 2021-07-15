//
// Created by bastian on 12.07.21.
//

#ifndef ARTERY_CATCHCHECKS_H
#define ARTERY_CATCHCHECKS_H

#include "CheckResult.h"
#include <vanetza/asn1/cam.hpp>
#include <omnetpp.h>
#include <artery/application/md/kalman/Kalman_SVI.h>
#include <artery/application/md/kalman/Kalman_SC.h>
#include <artery/application/md/kalman/Kalman_SI.h>
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/GlobalEnvironmentModel.h>
#include <artery/application/VehicleDataProvider.h>
#include <artery/application/md/util/F2MDParameters.h>
#include "artery/traci/VehicleController.h"
#include "artery/utility/Geometry.h"

namespace artery {


    class CatchChecks {
    public:
        CatchChecks() = delete;

        CatchChecks(std::shared_ptr<const traci::API> traciAPI, GlobalEnvironmentModel *globalEnvironmentModel,
                    DetectionParameters *detectionParameters,
                    Kalman_SVI *kalmanSVI, Kalman_SC *kalmanSVSI,
                    Kalman_SI *kalmanSI, Kalman_SI *kalmanVI);

        CheckResult *checkCAM(const VehicleDataProvider *receiverVDP, const std::vector<Position>& receiverVehicleOutline, TrackedObjectsFilterRange &envModObjects,
                              const vanetza::asn1::Cam &currentCam, const vanetza::asn1::Cam *lastCamPtr);

    private:

        static bool staticInitializationComplete;
        static GlobalEnvironmentModel *mGlobalEnvironmentModel;
        static traci::Boundary mSimulationBoundary;
        static std::shared_ptr<const traci::API> mTraciAPI;
        DetectionParameters *detectionParameters;

//        std::vector<std::string> lastPolyIdsForDebug;
        Kalman_SVI *kalmanSVI;
        Kalman_SC *kalmanSVSI;
        Kalman_SI *kalmanSI;
        Kalman_SI *kalmanVI;

        double ProximityPlausibilityCheck(Position &testPosition, const Position &myPosition,
                                          TrackedObjectsFilterRange &envModObjects);

        double RangePlausibilityCheck(const ReferencePosition_t &senderReferencePosition, const ReferencePosition_t &receiverReferencePosition);
    };
} // namespace artery

#endif //ARTERY_CATCHCHECKS_H
