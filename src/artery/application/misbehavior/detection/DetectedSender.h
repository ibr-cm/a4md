//
// Created by bastian on 15.06.21.
//

#ifndef ARTERY_DETECTEDSENDER_H
#define ARTERY_DETECTEDSENDER_H


#include <artery/application/misbehavior/checks/BaseChecks.h>
#include <list>
#include <vanetza/asn1/cam.hpp>
#include <utility>
#include <artery/application/misbehavior/checks/kalman/Kalman_SVI.h>
#include <artery/application/misbehavior/checks/kalman/Kalman_SC.h>
#include <artery/application/misbehavior/checks/kalman/Kalman_SI.h>
#include <artery/application/misbehavior/checks/CheckResult.h>
#include <artery/application/misbehavior/util/DetectionLevels.h>
#include <artery/traci/VehicleController.h>
#include <artery/application/misbehavior/util/F2MDParameters.h>
#include <bitset>

namespace artery {

    class DetectedSender {
    public:
        DetectedSender(const std::shared_ptr<const traci::API> &traciAPI,
                       GlobalEnvironmentModel *globalEnvironmentModel,
                       DetectionParameters *detectionParameters,
                       const Timer *timer,
                       const std::shared_ptr<vanetza::asn1::Cam> &message,
                       const std::map<detectionLevels::DetectionLevels, bool> &checkableDetectionLevels);


        ~DetectedSender();


        std::shared_ptr<CheckResult> addAndCheckCam(const std::shared_ptr<vanetza::asn1::Cam> &message,
                                                    const VehicleDataProvider *receiverVDP,
                                                    const std::vector<Position> &receiverVehicleOutline,
                                                    const std::vector<std::shared_ptr<vanetza::asn1::Cam>> &surroundingCamObjects);

        StationID_t getStationId() const { return mStationId; };

        std::vector<std::shared_ptr<vanetza::asn1::Cam>> getCamVector() const {
            std::vector<std::shared_ptr<vanetza::asn1::Cam>> cams{camList.begin(), camList.end()};
            cams.pop_back();
            return cams;
        };


        bool hasBeenReported() const { return mHasBeenReported; };

        std::string getPreviousReportId() const { return mPreviousReportId; };

        void setReportId(std::string reportId) {
            mPreviousReportId = std::move(reportId);
            mHasBeenReported = true;
        };

        bool checkOmittedReportsLimit(const bitset<16> &reportedErrorCodes);

        void incrementOmittedReports(const std::vector<std::bitset<16>> &detectionLevelErrorCodes,
                                     const std::bitset<16> &reportedErrorCodes);

        void resetOmittedReports(const bitset<16> &reportedErrorCodes);

        std::list<std::shared_ptr<vanetza::asn1::Cam>> camList;
        std::shared_ptr<vanetza::asn1::Cam> latestCam;

    private:
        BaseChecks *baseChecks;
        int mCamsArraySize;

        Position mPosition;
        StationID_t mStationId;
        bool mHasBeenReported;
        std::string mPreviousReportId;
        std::vector<int> mOmittedReportsPerErrorCode;
        int mOmittedReportsCumulated;


    };

} //namespace artery

#endif //ARTERY_DETECTEDSENDER_H
