
#ifndef ARTERY_MDSERVICE_H_
#define ARTERY_MDSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/application/misbehavior/util/DetectionLevels.h"
#include "artery/application/misbehavior/report/Report.h"
#include <vanetza/asn1/asn1c_wrapper.hpp>
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/misbehavior/util/MisbehaviorTypes.h"
#include "DetectedSender.h"
#include "artery/application/misbehavior/util/F2MDParameters.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include <vanetza/asn1/misbehavior_report.hpp>
#include "artery/application/misbehavior/fusion/BaseFusion.h"

namespace artery {


    class MisbehaviorDetectionService : public ItsG5Service {
    public:

        ~MisbehaviorDetectionService() override;

        void initialize() override;

        void indicate(const vanetza::btp::DataIndication &, omnetpp::cPacket *, const NetworkInterface &) override;

        void
        receiveSignal(omnetpp::cComponent *, omnetpp::simsignal_t, omnetpp::cObject *, omnetpp::cObject *) override;

    private:
        const VehicleDataProvider *mVehicleDataProvider = nullptr;
        const LocalEnvironmentModel *mLocalEnvironmentModel = nullptr;
        static GlobalEnvironmentModel *mGlobalEnvironmentModel;
        const Timer *mTimer = nullptr;

        static bool staticInitializationComplete;
        static traci::Boundary mSimulationBoundary;
        static std::shared_ptr<const traci::API> mTraciAPI;

        std::shared_ptr<vanetza::asn1::Cam> mLastSentCam;
        BaseFusion *mFusionApplication;
        std::map<uint32_t, std::shared_ptr<DetectedSender>> detectedSenders;
        std::string lastPolyId;
        std::list<std::string> activePoIs;
        const traci::VehicleController *mVehicleController = nullptr;

        void visualizeCamPosition(vanetza::asn1::Cam cam, const libsumo::TraCIColor &color,
                                  const std::string &idPrefix);

        void initializeParameters();


        std::vector<std::shared_ptr<vanetza::asn1::Cam>> getSurroundingCamObjects(StationID_t senderStationId);

        std::shared_ptr<CheckResult> checkCam(const shared_ptr<vanetza::asn1::Cam> &message);

        string generateReportId(const StationID_t &senderStationId);

        void detectMisbehavior(const shared_ptr<vanetza::asn1::Cam> &message);

        vector<shared_ptr<vanetza::asn1::Cam>> getOverlappingCams(const DetectedSender &detectedSender);

    };

} // namespace artery

#endif /* ARTERY_MDSERVICE_H_ */