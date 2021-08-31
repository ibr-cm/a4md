
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
        MisbehaviorDetectionService();

        ~MisbehaviorDetectionService() override;

        void initialize() override;

        void indicate(const vanetza::btp::DataIndication &, omnetpp::cPacket *, const NetworkInterface &) override;

        void trigger() override;

        void
        receiveSignal(omnetpp::cComponent *, omnetpp::simsignal_t, omnetpp::cObject *, omnetpp::cObject *) override;

    protected:
        void handleMessage(omnetpp::cMessage *) override;

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

        vanetza::asn1::MisbehaviorReport createReport(detectionLevels::DetectionLevels detectionLevel,
                                                      const std::string &reportId, std::string &relatedReportId,
                                                      const vanetza::asn1::Cam *reportedMessage,
                                                      std::bitset<16> errorCode, DetectedSender &detectedSender);

        vanetza::asn1::MisbehaviorReport
        createLevel1Report(const std::string &reportId, const vanetza::asn1::Cam *reportedMessage,
                           const bitset<16> &semanticDetectionErrorCodeCAM);

        vanetza::asn1::MisbehaviorReport
        createLevel2Report(const std::string &reportId, const vanetza::asn1::Cam *reportedMessage,
                           const bitset<16> &semanticDetectionErrorCodeCAM,
                           DetectedSender &detectedSender);

        vanetza::asn1::MisbehaviorReport
        createLevel3Report(const std::string &reportId, const vanetza::asn1::Cam *reportedMessage,
                           const bitset<16> &semanticDetectionErrorCodeCAM,
                           DetectedSender &detectedSender);

        vanetza::asn1::MisbehaviorReport
        createLevel4Report(const string &reportId, const vanetza::asn1::Cam *reportedMessage,
                           const bitset<16> &semanticDetectionErrorCodeCAM,
                           DetectedSender &detectedSender);

        vanetza::asn1::MisbehaviorReport
        createBasicMisbehaviorReport(const string &reportId, const vanetza::asn1::Cam *cam);

//        void fillSenderInfoContainer(SenderInfoContainer_t &senderInfoContainer);
//        void fillSenderInfoContainer(const shared_ptr<SenderInfoContainer_t> &senderInfoContainer);

        static void fillRelatedReportContainer(RelatedReportContainer_t *&relatedReportContainer,
                                               const std::string &relatedReportId, const int &omittedReportsNumber);

        string generateReportId(const StationID_t &senderStationId);


        static void fillMisbehaviorTypeContainer(MisbehaviorTypeContainer_t &misbehaviorTypeContainer,
                                                 const detectionLevels::DetectionLevels &detectionLevel,
                                                 const bitset<16> &errorCode);

        void detectMisbehavior(const shared_ptr<vanetza::asn1::Cam> &message);


        vector<shared_ptr<vanetza::asn1::Cam>> getOverlappingCams(const DetectedSender &detectedSender);

    };

} // namespace artery

#endif /* ARTERY_MDSERVICE_H_ */