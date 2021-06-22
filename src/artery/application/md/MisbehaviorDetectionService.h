
#ifndef ARTERY_MDSERVICE_H_
#define ARTERY_MDSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include <vanetza/asn1/asn1c_wrapper.hpp>
#include <curl/curl.h>
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/application/VehicleDataProvider.h"
#include "MisbehaviorTypes.h"
#include "DetectedSender.h"
#include "F2MDParameters.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include <artery/application/md/util/CustomRing.h>

namespace artery {

    struct PolygonStruct{
        std::string id;
        std::shared_ptr<geometry::Box> boundingBox;
        std::shared_ptr<geometry::Ring> outline;
    };

    struct LaneStruct{
        std::string id;
        std::shared_ptr<geometry::Box> boundingBox;
        std::shared_ptr<geometry::LineString> shape;
        double width;
    };


    struct GridCell{
        geometry::Box boundingBox;
        std::vector<std::shared_ptr<PolygonStruct>> obstacles;
        std::vector<std::shared_ptr<PolygonStruct>> junctions;
        std::vector<std::shared_ptr<LaneStruct>> lanes;
    };


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

        void visualizeCamPosition(vanetza::asn1::Cam cam, const libsumo::TraCIColor &color, const std::string& idPrefix);

        void initializeParameters();

        static void initializeGridCells();

        void initializeObstacles();

        static void initializeLanes();

        static void initializeJunctions();

        static std::vector<std::pair<int, int>> getApplicableGridCells(geometry::Box boundingBox);

        std::list<std::string> activePoIs;
        const traci::VehicleController *mVehicleController = nullptr;

        omnetpp::cMessage *m_self_msg;
        CURL *curl;
        const VehicleDataProvider *mVehicleDataProvider = nullptr;
        const LocalEnvironmentModel *mLocalEnvironmentModel = nullptr;
        GlobalEnvironmentModel *mGlobalEnvironmentModel = nullptr;
        const Timer *mTimer = nullptr;

        static traci::Boundary simulationBoundary;
        std::map<uint32_t, DetectedSender *> detectedSenders;
        static std::shared_ptr<const traci::API> traciAPI;
        static bool staticInitializationComplete;
        static std::map<uint32_t, misbehaviorTypes::MisbehaviorTypes> mStationIdMisbehaviorTypeMap;
        static std::vector<std::vector<GridCell>> gridCells;


    };

} // namespace artery

#endif /* ARTERY_MDSERVICE_H_ */