
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
#include <vanetza/asn1/misbehavior_report.hpp>

//#include <boost/shared_ptr.hpp>
//#include <boost/geometry/index/rtree.hpp>
//#include <boost/geometry.hpp>
//#include <boost/geometry/geometries/point.hpp>
//#include <boost/geometry/geometries/box.hpp>
//
//namespace bg = boost::geometry;
//namespace bgi = boost::geometry::index;
//namespace boost {
//    namespace geometry {
//        namespace index {
//            template<typename Box>
//            struct indexable<std::shared_ptr<Box> > {
//                typedef std::shared_ptr<Box> V;
//
//                typedef Box const &result_type;
//
//                result_type operator()(V const &v) const { return *v; }
//            };
//        }
//    }
//}// namespace boost::geometry::index
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

        std::vector<Position> getVehicleOutline();

        vanetza::asn1::MisbehaviorReport createMisbehaviorReport(const std::string& reportId,vanetza::asn1::Cam cam);


        std::list<std::string> activePoIs;
        const traci::VehicleController *mVehicleController = nullptr;

        CURL *curl;
        const VehicleDataProvider *mVehicleDataProvider = nullptr;
        const LocalEnvironmentModel *mLocalEnvironmentModel = nullptr;
        static GlobalEnvironmentModel *mGlobalEnvironmentModel;
        const Timer *mTimer = nullptr;

        static traci::Boundary mSimulationBoundary;
        std::map<uint32_t, DetectedSender *> detectedSenders;
        static std::shared_ptr<const traci::API> mTraciAPI;
        static bool staticInitializationComplete;
        static std::map<uint32_t, misbehaviorTypes::MisbehaviorTypes> mStationIdMisbehaviorTypeMap;
        std::string lastPolyId;
    };

} // namespace artery

#endif /* ARTERY_MDSERVICE_H_ */