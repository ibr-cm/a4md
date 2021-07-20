
#ifndef ARTERY_MDSERVICE_H_
#define ARTERY_MDSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include <vanetza/asn1/asn1c_wrapper.hpp>
#include <curl/curl.h>
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/md/util/MisbehaviorTypes.h"
#include "DetectedSender.h"
#include "artery/application/md/util/F2MDParameters.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include <vanetza/asn1/misbehavior_report.hpp>
#include "artery/application/md/fusion/BaseFusion.h"

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

        static void addStationIdToVehicleList(uint32_t stationId, misbehaviorTypes::MisbehaviorTypes misbehaviorType);

        static void removeStationIdFromVehicleList(uint32_t stationId);

    protected:
        void handleMessage(omnetpp::cMessage *) override;

    private:
        static misbehaviorTypes::MisbehaviorTypes getMisbehaviorTypeOfStationId(uint32_t);

        void
        visualizeCamPosition(vanetza::asn1::Cam cam, const libsumo::TraCIColor &color, const std::string &idPrefix);

        void initializeParameters();

        vanetza::asn1::MisbehaviorReport
        createMisbehaviorReport(const string &reportId, const vanetza::asn1::Cam *cam,
                                DetectionReferenceCAM_t *semanticDetectionReferenceCam);

        void fillSenderInfoContainer(SenderInfoContainer_t &senderInfoContainer);

        static DetectionReferenceCAM_t
        fillSemanticDetectionReferenceCam(detectionLevels::DetectionLevels detectionLevelCam,
                                          std::bitset<16> semanticDetectionErrorCodeCAM);


        std::vector<vanetza::asn1::Cam *> getSurroundingCamObjects(StationID_t senderStationId);

        std::list<std::string> activePoIs;
        const traci::VehicleController *mVehicleController = nullptr;

        CURL *curl;
        const VehicleDataProvider *mVehicleDataProvider = nullptr;
        const LocalEnvironmentModel *mLocalEnvironmentModel = nullptr;
        static GlobalEnvironmentModel *mGlobalEnvironmentModel;
        const Timer *mTimer = nullptr;

        BaseFusion *fusionApplication;
        static traci::Boundary mSimulationBoundary;
        std::map<uint32_t, DetectedSender *> detectedSenders;
        static std::shared_ptr<const traci::API> mTraciAPI;
        static bool staticInitializationComplete;
        static std::map<uint32_t, misbehaviorTypes::MisbehaviorTypes> mStationIdMisbehaviorTypeMap;
        std::string lastPolyId;

        std::vector<std::bitset<16>> checkCam(const vanetza::asn1::Cam &message);

        void sendLevel1Report(const std::string &reportId, const std::string *relatedReportId,
                              const vanetza::asn1::Cam *reportedMessage,
                              std::bitset<16> semanticDetectionErrorCodeCAM);

        void sendLevel2Report(const std::string &reportId, const std::string *relatedReportId,
                              const vanetza::asn1::Cam *reportedMessage,
                              std::bitset<16> semanticDetectionErrorCodeCAM,
                              StationID_t senderStationId);

        void sendLevel3Report(const std::string &reportId, const std::string *relatedReportId,
                              const vanetza::asn1::Cam *reportedMessage, bitset<16> semanticDetectionErrorCodeCAM,
                              StationID_t senderStationId);

        void
        fillRelatedReportContainer(RelatedReportContainer_t *relatedReportContainer, const std::string &relatedReportId,
                                   int omittedReportsNumber);
    };

} // namespace artery

#endif /* ARTERY_MDSERVICE_H_ */