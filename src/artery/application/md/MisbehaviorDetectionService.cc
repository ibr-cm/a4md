
#include "artery/application/md/MisbehaviorDetectionService.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
#include <vanetza/asn1/cam.hpp>
#include "artery/application/md/base64.h"
#include "artery/application/CaService.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/md/MisbehaviorTypes.h"
#include "artery/envmod/sensor/Sensor.h"
#include <boost/units/io.hpp>
#include <inet/common/ModuleAccess.h>
#include "artery/traci/Cast.h"
#include "artery/application/md/MisbehaviorCaService.h"

namespace artery {
    using namespace omnetpp;

    Define_Module(MisbehaviorDetectionService);

    static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");

    bool MisbehaviorDetectionService::staticInitializationComplete = false;
    std::map<uint32_t, misbehaviorTypes::MisbehaviorTypes> MisbehaviorDetectionService::mStationIdMisbehaviorTypeMap;
    std::shared_ptr<const traci::API> MisbehaviorDetectionService::traciAPI;
    std::vector<std::vector<geometry::Box>> MisbehaviorDetectionService::gridCellBoundaries;
    std::vector<std::vector<std::vector<ObstacleStruct>>> MisbehaviorDetectionService::gridCellObstacles;
    std::vector<std::vector<std::vector<LaneStruct>>> MisbehaviorDetectionService::gridCellLanes;

    traci::Boundary MisbehaviorDetectionService::simulationBoundary;

    MisbehaviorDetectionService::MisbehaviorDetectionService() {
        curl = curl_easy_init();
    }

    MisbehaviorDetectionService::~MisbehaviorDetectionService() {
        cancelAndDelete(m_self_msg);
        curl_easy_cleanup(curl);

//        while (!activePoIs.empty()) {
//            traciPoiScope->remove(activePoIs.front());
//            activePoIs.pop_front();
//        }
    }

    void MisbehaviorDetectionService::initialize() {
        ItsG5BaseService::initialize();
        m_self_msg = new cMessage("InitializeMessage");
        subscribe(scSignalCamReceived);
        scheduleAt(simTime() + 3.0, m_self_msg);
        mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
        mLocalEnvironmentModel = getFacilities().get_mutable_ptr<LocalEnvironmentModel>();
        mGlobalEnvironmentModel = mLocalEnvironmentModel->getGlobalEnvMod();
        mVehicleController = &getFacilities().get_const<traci::VehicleController>();

        if (!staticInitializationComplete) {
            staticInitializationComplete = true;
            traciAPI = getFacilities().get_const<traci::VehicleController>().getTraCI();
            traciPoiScope = &traciAPI->poi;
            TraCIAPI::PolygonScope polygon = traciAPI->polygon;
            simulationBoundary = traci::Boundary{traciAPI->simulation.getNetBoundary()};
            initializeParameters();
            initializeGridCells();
            initializeObstacles();
            initializeLanes();
        }
    }


    void MisbehaviorDetectionService::initializeParameters() {
        F2MDParameters::detectionParameters.maxPlausibleSpeed = par("maxPlausibleSpeed");
        F2MDParameters::detectionParameters.maxPlausibleAcceleration = par("maxPlausibleAcceleration");
        F2MDParameters::detectionParameters.maxPlausibleDeceleration = par("maxPlausibleDeceleration");
        F2MDParameters::detectionParameters.maxPlausibleRange = par("maxPlausibleRange");

        F2MDParameters::detectionParameters.maxProximityRangeL = par("maxProximityRangeL");
        F2MDParameters::detectionParameters.maxProximityRangeW = par("maxProximityRangeW");
        F2MDParameters::detectionParameters.maxProximityDistance = par("maxProximityDistance");
        F2MDParameters::detectionParameters.maxTimeDelta = par("maxTimeDelta");
        F2MDParameters::detectionParameters.maxMgtRng = par("maxMgtRng");
        F2MDParameters::detectionParameters.maxMgtRngDown = par("maxMgtRngDown");
        F2MDParameters::detectionParameters.maxMgtRngUp = par("maxMgtRngUp");
        F2MDParameters::detectionParameters.maxSuddenAppearanceRange = par("maxSuddenAppearanceRange");
        F2MDParameters::detectionParameters.maxSuddenAppearanceTime = par("maxSuddenAppearanceTime");
        F2MDParameters::detectionParameters.maxCamFrequency = par("maxCamFrequency");
        F2MDParameters::detectionParameters.maxOffroadSpeed = par("maxOffroadSpeed");
        F2MDParameters::detectionParameters.positionHeadingTime = par("positionHeadingTime");
        F2MDParameters::detectionParameters.maxHeadingChange = par("maxHeadingChange");

        F2MDParameters::detectionParameters.maxKalmanTime = par("maxKalmanTime");
        F2MDParameters::detectionParameters.kalmanMinPosRange = par("kalmanMinPosRange");
        F2MDParameters::detectionParameters.kalmanMinSpeedRange = par("kalmanMinSpeedRange");
        F2MDParameters::detectionParameters.kalmanMinHeadingRange = par("kalmanMinHeadingRange");
        F2MDParameters::detectionParameters.kalmanPosRange = par("kalmanPosRange");
        F2MDParameters::detectionParameters.kalmanSpeedRange = par("kalmanSpeedRange");

        F2MDParameters::miscParameters.objectAccessHelperGridSize = par("objectAccessHelperGridSize");
    }

    void MisbehaviorDetectionService::initializeGridCells() {
        geometry::Box bbox(geometry::Point(0, 0), geometry::Point(
                position_cast(simulationBoundary, simulationBoundary.upperRightPosition()).x.value(),
                position_cast(simulationBoundary, simulationBoundary.lowerLeftPosition()).y.value()));

        int splitGridSize = F2MDParameters::miscParameters.objectAccessHelperGridSize;
        double gridLengthX = bbox.max_corner().get<0>() / splitGridSize;
        double gridLengthY = bbox.max_corner().get<1>() / splitGridSize;
        std::cout << "size: x: " << gridLengthX << " y: " << gridLengthY << std::endl;
        for (int i = 0; i < splitGridSize; i++) {
            std::vector<geometry::Box> row;
            std::vector<std::vector<ObstacleStruct>> rowObstacle;
            std::vector<std::vector<LaneStruct>> rowLane;
            for (int j = 0; j < splitGridSize; j++) {
                libsumo::TraCIPositionVector v;
                geometry::Box currentBox(geometry::Point(i * gridLengthX, j * gridLengthY),
                                         geometry::Point((i + 1) * gridLengthX, (j + 1) * gridLengthY));
                boost::geometry::model::ring<geometry::Point, true, true> ring;
                boost::geometry::convert(currentBox, ring);
                row.emplace_back(currentBox);
                std::vector<ObstacleStruct> cellObstacle;
                std::vector<LaneStruct> cellLane;
                rowObstacle.emplace_back(cellObstacle);
                rowLane.emplace_back(cellLane);

                libsumo::TraCIPositionVector outline;
                for (const geometry::Point &p : ring) {
                    outline.value.emplace_back(position_cast(simulationBoundary, Position(p.get<0>(), p.get<1>())));
                }
                std::string id{"outlinePoly_" + std::to_string(i) + "_" + std::to_string(j)};
                traciAPI->polygon.add(id, outline, libsumo::TraCIColor(255, 0, 255, 255), false, "helper", 5);
            }
            gridCellBoundaries.emplace_back(row);
            gridCellObstacles.emplace_back(rowObstacle);
            gridCellLanes.emplace_back(rowLane);
        }
    }

    void MisbehaviorDetectionService::initializeObstacles() {
        int splitGridSize = F2MDParameters::miscParameters.objectAccessHelperGridSize;
        ObstacleRtree *obstacleRtree = mGlobalEnvironmentModel->getObstacleRTree();
        ObstacleDB *obstacles = mGlobalEnvironmentModel->getObstacleDB();
        for (const auto &treeObject : *obstacleRtree) {
            geometry::Box obstacle = treeObject.first;
            for (int i = 0; i < splitGridSize; i++) {
                if (obstacle.min_corner().get<0>() >= gridCellBoundaries[i][0].min_corner().get<0>() &&
                    obstacle.min_corner().get<0>() <= gridCellBoundaries[i][0].max_corner().get<0>() ||
                    obstacle.max_corner().get<0>() >= gridCellBoundaries[i][0].min_corner().get<0>() &&
                    obstacle.max_corner().get<0>() <= gridCellBoundaries[i][0].max_corner().get<0>()) {
                    for (int j = 0; j < splitGridSize; j++) {
                        if (obstacle.min_corner().get<1>() >= gridCellBoundaries[i][j].min_corner().get<1>() &&
                            obstacle.min_corner().get<1>() <= gridCellBoundaries[i][j].max_corner().get<1>() ||
                            obstacle.max_corner().get<1>() >= gridCellBoundaries[i][j].min_corner().get<1>() &&
                            obstacle.max_corner().get<1>() <= gridCellBoundaries[i][j].max_corner().get<1>()) {

                            geometry::Ring outline;
                            for (const auto &p : (*obstacles)[treeObject.second]->getOutline()) {
                                boost::geometry::append(outline, geometry::Point(p.x.value(), p.y.value()));
                            }
                            boost::geometry::correct(outline);
                            ObstacleStruct obstacleStruct = {treeObject.second,
                                                             std::make_shared<geometry::Box>(treeObject.first),
                                                             std::make_shared<geometry::Ring>(outline)};
                            gridCellObstacles[i][j].emplace_back(obstacleStruct);
                        }
                    }
                }
            }
        }
    }


    void MisbehaviorDetectionService::initializeLanes() {
        int splitGridSize = F2MDParameters::miscParameters.objectAccessHelperGridSize;
        TraCIAPI::LaneScope lane = traciAPI->lane;
        std::vector<std::string> laneIDs = lane.getIDList();
        std::vector<std::string> junctionIDs = traciAPI->junction.getIDList();
//        std::vector<std::string> sanitizedJunctionIDs;
//        sanitizedJunctionIDs.reserve(junctionIDs.size());
//        for (const auto& junctionID : junctionIDs) {
//            sanitizedJunctionIDs.emplace_back(junctionID.substr(0, junctionID.find('_')));
//            if(junctionID == ":-13430_0_0"){
//                std::cout << junctionID.substr(0, junctionID.find('_'));
//            }
//        }
//        std::cout << sanitizedJunctionIDs[0] << std::endl;
        for (const auto &laneID : laneIDs) {
            if (std::find(junctionIDs.begin(), junctionIDs.end(), laneID.substr(1, laneID.find('_') - 1)) !=
                junctionIDs.end()) {
                continue;
            }
            libsumo::TraCIPositionVector shapeTraci = lane.getShape(laneID);
            geometry::LineString laneShape;
            for (const auto &v : shapeTraci.value) {
//                Position p = position_cast(simulationBoundary,v);
//                boost::geometry::append(laneShape, geometry::Point(p.x.value(),p.y.value()));
                boost::geometry::append(laneShape, position_cast(simulationBoundary, v));
            }
            geometry::Box bbox = boost::geometry::return_envelope<geometry::Box>(laneShape);
            for (int i = 0; i < splitGridSize; i++) {
                if (bbox.min_corner().get<0>() >= gridCellBoundaries[i][0].min_corner().get<0>() &&
                    bbox.min_corner().get<0>() <= gridCellBoundaries[i][0].max_corner().get<0>() ||
                    bbox.max_corner().get<0>() >= gridCellBoundaries[i][0].min_corner().get<0>() &&
                    bbox.max_corner().get<0>() <= gridCellBoundaries[i][0].max_corner().get<0>()) {
                    for (int j = 0; j < splitGridSize; j++) {
                        if (bbox.min_corner().get<1>() >= gridCellBoundaries[i][j].min_corner().get<1>() &&
                            bbox.min_corner().get<1>() <= gridCellBoundaries[i][j].max_corner().get<1>() ||
                            bbox.max_corner().get<1>() >= gridCellBoundaries[i][j].min_corner().get<1>() &&
                            bbox.max_corner().get<1>() <= gridCellBoundaries[i][j].max_corner().get<1>()) {
                            LaneStruct laneStruct = {laneID,
                                                     std::make_shared<geometry::Box>(bbox),
                                                     std::make_shared<geometry::LineString>(laneShape),
                                                     lane.getWidth(laneID)};
                            gridCellLanes[i][j].emplace_back(laneStruct);
                            if (i == 3 && j == 3) {
                                std::cout << laneID << " " << boost::geometry::dsv(bbox) << std::endl;
                            }
                        }
                    }
                }
            }
        }
    }

    void MisbehaviorDetectionService::trigger() {
        Enter_Method("trigger");
    }

    void MisbehaviorDetectionService::indicate(const vanetza::btp::DataIndication &ind, cPacket *packet,
                                               const NetworkInterface &net) {
        Enter_Method("indicate");

        EV_INFO << "packet indication on channel " << net.channel << "with byte length" << packet->getByteLength()
                << "\n";

        delete (packet);
    }

    void MisbehaviorDetectionService::handleMessage(cMessage *msg) {
        Enter_Method("handleMessage");

        if (msg == m_self_msg) {
            EV_INFO << "self message\n";
        }
    }

    void MisbehaviorDetectionService::visualizeCamPosition(vanetza::asn1::Cam cam, const libsumo::TraCIColor &color,
                                                           const std::string &idPrefix) {
        traci::TraCIGeoPosition traciGeoPosition = {
                (double) cam->cam.camParameters.basicContainer.referencePosition.longitude / 10000000.0,
                (double) cam->cam.camParameters.basicContainer.referencePosition.latitude / 10000000.0};
        traci::TraCIPosition traciPosition = mVehicleController->getTraCI()->convert2D(traciGeoPosition);
        std::string poiId = mVehicleController->getVehicleId();
        poiId += idPrefix;
        poiId += "_CAM_";
        poiId += std::to_string(cam->cam.generationDeltaTime);
        traciPoiScope->add(poiId, traciPosition.x, traciPosition.y, color,
                           poiId, 5, "", 0,
                           0, 0);
        activePoIs.push_back(poiId);
        if (activePoIs.size() > F2MDParameters::miscParameters.CamLocationVisualizerMaxLength) {
            traciPoiScope->remove(activePoIs.front());
            activePoIs.pop_front();
        }
        int alphaStep = 185 / F2MDParameters::miscParameters.CamLocationVisualizerMaxLength;
        int currentAlpha = 80;
//        for(const auto& poi : activePoIs){
//            traciPoiScope->setColor(poi,color);
//            currentAlpha += alphaStep;
//        }
    }

    void MisbehaviorDetectionService::receiveSignal(cComponent *source, simsignal_t signal, cObject *c_obj, cObject *) {
        Enter_Method("receiveSignal");
        if (signal == scSignalCamReceived) {
            CaObject *ca = dynamic_cast<CaObject *>(c_obj);
            vanetza::asn1::Cam message = ca->asn1();
            uint32_t senderStationId = message->header.stationID;
            std::cout << mVehicleDataProvider->getStationId() << " <-- " << senderStationId << std::endl;

            traci::TraCIGeoPosition traciGeoPositionSelf = {
                    mVehicleDataProvider->longitude().value(),
                    mVehicleDataProvider->latitude().value()};
            traci::TraCIPosition traciPositionSelf = traciAPI->convert2D(traciGeoPositionSelf);
            Position ownPosition = Position(traciPositionSelf.x, traciPositionSelf.y);
            auto &allObjects = mLocalEnvironmentModel->allObjects();
            TrackedObjectsFilterRange envModObjects = filterBySensorCategory(allObjects, "CA");
            CheckResult *result;
            if (detectedSenders.find(senderStationId) != detectedSenders.end()) {
                result = detectedSenders[senderStationId]->addAndCheckCam(message, ownPosition,
                                                                          envModObjects);
            } else {
                detectedSenders[senderStationId] = new DetectedSender(traciAPI, &F2MDParameters::detectionParameters,
                                                                      message);
                result = detectedSenders[senderStationId]->addAndCheckCam(message, ownPosition,
                                                                          envModObjects);
            }
            std::cout << result->toString(0.5) << std::endl;

            misbehaviorTypes::MisbehaviorTypes senderMisbehaviorType = getMisbehaviorTypeOfStationId(senderStationId);
            if (senderMisbehaviorType == misbehaviorTypes::LocalAttacker) {
                EV_INFO << "Received manipulated CAM!";
                EV_INFO << "Received DoS " << simTime().inUnit(SimTimeUnit::SIMTIME_MS) << " delta:  "
                        << message->cam.generationDeltaTime << "\n";
                vanetza::ByteBuffer byteBuffer = ca->asn1().encode();
                std::string encoded(byteBuffer.begin(), byteBuffer.end());
                std::string b64Encoded = base64_encode(reinterpret_cast<const unsigned char *>(encoded.c_str()),
                                                       encoded.length(), false);
                if (curl) {
                    curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:9981/newCAM");
                    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, b64Encoded.c_str());
                    CURLcode curlResponse = curl_easy_perform(curl);
                    if (curlResponse != CURLE_OK) {
                        EV_ERROR << "curl_easy_perform() failed: %s\n"
                                 << curl_easy_strerror(curlResponse);
                    }
                }
            } else if (senderMisbehaviorType == misbehaviorTypes::Benign) {
                EV_INFO << "Received benign CAM!";
            } else {
                EV_INFO << "Received weird misbehaviorType";
            }


            ReferencePosition_t referencePosition = message->cam.camParameters.basicContainer.referencePosition;
            traci::TraCIGeoPosition traciGeoPositionSender = {
                    (double) referencePosition.longitude / 10000000.0,
                    (double) referencePosition.latitude / 10000000.0};
            Position senderPosition = position_cast(simulationBoundary, traciAPI->convert2D(traciGeoPositionSender));

            bool messageHasPoi = false;

            int splitGridSize = F2MDParameters::miscParameters.objectAccessHelperGridSize;
            for (int i = 0; i < splitGridSize; i++) {
                if (senderPosition.x.value() >= gridCellBoundaries[i][0].min_corner().get<0>() &&
                    senderPosition.x.value() <= gridCellBoundaries[i][0].max_corner().get<0>())
                    for (int j = 0; j < splitGridSize; j++) {
                        if (senderPosition.y.value() >= gridCellBoundaries[i][j].min_corner().get<1>() &&
                            senderPosition.y.value() <= gridCellBoundaries[i][j].max_corner().get<1>()) {
                            for (const auto &obstacleStruct : gridCellObstacles[i][j]) {
                                geometry::Box *obstacleBox = obstacleStruct.boundingBox.get();
                                geometry::Point senderPoint(senderPosition.x.value(), senderPosition.y.value());
                                if (senderMisbehaviorType == misbehaviorTypes::LocalAttacker) {
                                    if (boost::geometry::within(senderPoint, obstacleBox)) {
                                        if (boost::geometry::within(senderPoint, *obstacleStruct.outline)) {
                                            std::string prefix = "inside";
                                            prefix += obstacleStruct.id;
//                                            std::cout << "exactly inside object " << obstacleStruct.id
//                                                      << " position: "
//                                                      << boost::geometry::dsv(obstacleBox) << std::endl;
                                            visualizeCamPosition(message, libsumo::TraCIColor(255, 255, 255, 255),
                                                                 prefix);
                                        } else {
                                            std::string prefix = "bbox";
                                            prefix += obstacleStruct.id;
//                                            std::cout << "inside bbox " << obstacleStruct.id << " position: "
//                                                      << boost::geometry::dsv(obstacleBox) << std::endl;
                                            visualizeCamPosition(message, libsumo::TraCIColor(255, 255, 0, 255),
                                                                 prefix);
                                        }
                                        messageHasPoi = true;
                                    } else {

                                    }
                                }
                            }
                        } else {
                        }
                    }
            }
            if (!messageHasPoi && senderMisbehaviorType == misbehaviorTypes::LocalAttacker) {
                visualizeCamPosition(message, libsumo::TraCIColor(0, 255, 0, 255),
                                     "outside");
            }
        }
    }


    misbehaviorTypes::MisbehaviorTypes MisbehaviorDetectionService::getMisbehaviorTypeOfStationId(uint32_t stationId) {
        auto search = mStationIdMisbehaviorTypeMap.find(stationId);
        if (search != mStationIdMisbehaviorTypeMap.end()) {
            return search->second;
        } else {
            return misbehaviorTypes::SIZE_OF_ENUM;
        }
    }

    void MisbehaviorDetectionService::addStationIdToVehicleList(uint32_t stationId,
                                                                misbehaviorTypes::MisbehaviorTypes misbehaviorType) {
        mStationIdMisbehaviorTypeMap[stationId] = misbehaviorType;
    }

    void MisbehaviorDetectionService::removeStationIdFromVehicleList(uint32_t stationId) {
        auto it = mStationIdMisbehaviorTypeMap.find(stationId);
        if (it != mStationIdMisbehaviorTypeMap.end()) {
            mStationIdMisbehaviorTypeMap.erase(it);
        }
    }
}
