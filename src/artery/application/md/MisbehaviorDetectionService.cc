
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
    std::vector<std::vector<GridCell>> MisbehaviorDetectionService::gridCells;

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
            initializeJunctions();

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
        gridCells = std::vector<std::vector<GridCell>>(
                F2MDParameters::miscParameters.objectAccessHelperGridSize,
                std::vector<GridCell>(F2MDParameters::miscParameters.objectAccessHelperGridSize, GridCell()));


        geometry::Box bbox(geometry::Point(0, 0), geometry::Point(
                position_cast(simulationBoundary, simulationBoundary.upperRightPosition()).x.value(),
                position_cast(simulationBoundary, simulationBoundary.lowerLeftPosition()).y.value()));

        double gridLengthX = bbox.max_corner().get<0>() / F2MDParameters::miscParameters.objectAccessHelperGridSize;
        double gridLengthY = bbox.max_corner().get<1>() / F2MDParameters::miscParameters.objectAccessHelperGridSize;
        for (int i = 0; i < F2MDParameters::miscParameters.objectAccessHelperGridSize; i++) {
            for (int j = 0; j < F2MDParameters::miscParameters.objectAccessHelperGridSize; j++) {
                gridCells[i][j].boundingBox = geometry::Box(geometry::Point(i * gridLengthX, j * gridLengthY),
                                                            geometry::Point((i + 1) * gridLengthX,
                                                                            (j + 1) * gridLengthY));

                boost::geometry::model::ring<geometry::Point, true, true> ring;
                boost::geometry::convert(gridCells[i][j].boundingBox, ring);
                libsumo::TraCIPositionVector outline;
                for (const geometry::Point &p : ring) {
                    outline.value.emplace_back(position_cast(simulationBoundary, Position(p.get<0>(), p.get<1>())));
                }
                std::string id{"outlinePoly_" + std::to_string(i) + "_" + std::to_string(j)};
                traciAPI->polygon.add(id, outline, libsumo::TraCIColor(255, 0, 255, 255), false, "helper", 5);
            }
        }
    }

    std::vector<std::pair<int, int>> MisbehaviorDetectionService::getApplicableGridCells(geometry::Box boundingBox) {
        std::vector<std::pair<int, int>> applicableCells;
        for (int i = 0; i < F2MDParameters::miscParameters.objectAccessHelperGridSize; i++) {
            if (boundingBox.min_corner().get<0>() >= gridCells[i][0].boundingBox.min_corner().get<0>() &&
                boundingBox.min_corner().get<0>() <= gridCells[i][0].boundingBox.max_corner().get<0>() ||
                boundingBox.max_corner().get<0>() >= gridCells[i][0].boundingBox.min_corner().get<0>() &&
                boundingBox.max_corner().get<0>() <= gridCells[i][0].boundingBox.max_corner().get<0>()) {
                for (int j = 0; j < F2MDParameters::miscParameters.objectAccessHelperGridSize; j++) {
                    if (boundingBox.min_corner().get<1>() >= gridCells[i][j].boundingBox.min_corner().get<1>() &&
                        boundingBox.min_corner().get<1>() <= gridCells[i][j].boundingBox.max_corner().get<1>() ||
                        boundingBox.max_corner().get<1>() >= gridCells[i][j].boundingBox.min_corner().get<1>() &&
                        boundingBox.max_corner().get<1>() <= gridCells[i][j].boundingBox.max_corner().get<1>()) {
                        applicableCells.emplace_back(i, j);
                    }
                }
            }
        }
        return applicableCells;
    }

    void MisbehaviorDetectionService::initializeObstacles() {
        ObstacleRtree *obstacleRtree = mGlobalEnvironmentModel->getObstacleRTree();
        ObstacleDB *obstacles = mGlobalEnvironmentModel->getObstacleDB();
        for (const auto &treeObject : *obstacleRtree) {
            geometry::Box obstacle = treeObject.first;
            geometry::Ring outline;
            for (const auto &p : (*obstacles)[treeObject.second]->getOutline()) {
                boost::geometry::append(outline, geometry::Point(p.x.value(), p.y.value()));
            }
            boost::geometry::correct(outline);
            PolygonStruct obstacleStruct = {treeObject.second,
                                            std::make_shared<geometry::Box>(treeObject.first),
                                            std::make_shared<geometry::Ring>(outline)};
            for (auto coordinates : getApplicableGridCells(treeObject.first)) {
                gridCells[coordinates.first][coordinates.second].obstacles.emplace_back(
                        std::make_shared<PolygonStruct>(obstacleStruct));
            }
        }
    }


    void MisbehaviorDetectionService::initializeLanes() {
        TraCIAPI::LaneScope laneScope = traciAPI->lane;
        std::vector<std::string> laneIDs = laneScope.getIDList();
        std::vector<std::string> junctionIDs = traciAPI->junction.getIDList();
        for (const auto &laneID : laneIDs) {
            if (std::find(junctionIDs.begin(), junctionIDs.end(), laneID.substr(1, laneID.find('_') - 1)) !=
                junctionIDs.end()) {
                continue;
            } else {
                libsumo::TraCIPositionVector shapeTraci = laneScope.getShape(laneID);
                geometry::LineString laneShape;
                for (const auto &v : shapeTraci.value) {
                    boost::geometry::append(laneShape, position_cast(simulationBoundary, v));
                }
                geometry::Box boundingBox = boost::geometry::return_envelope<geometry::Box>(laneShape);
                LaneStruct laneStruct = {laneID,
                                         std::make_shared<geometry::Box>(boundingBox),
                                         std::make_shared<geometry::LineString>(laneShape),
                                         laneScope.getWidth(laneID)};
                for (auto coordinates : getApplicableGridCells(boundingBox)) {
                    gridCells[coordinates.first][coordinates.second].lanes.emplace_back(
                            std::make_shared<LaneStruct>(laneStruct));
                }
            }

        }
    }

    void MisbehaviorDetectionService::initializeJunctions() {
        TraCIAPI::JunctionScope junctionScope = traciAPI->junction;
        std::vector<std::string> junctionIDs = junctionScope.getIDList();
        for (const auto &junctionID : junctionIDs) {
            if (std::find(junctionIDs.begin(), junctionIDs.end(), junctionID.substr(1, junctionID.find('_') - 1)) !=
                junctionIDs.end()) {
                continue;
            }
            geometry::Ring outline;
            for (const auto &v: junctionScope.getShape(junctionID).value) {
                boost::geometry::append(outline, position_cast(simulationBoundary, v));
            }
            boost::geometry::correct(outline);

            geometry::Box boundingBox = boost::geometry::return_envelope<geometry::Box>(outline);
            PolygonStruct junctionStruct = {junctionID,
                                            std::make_shared<geometry::Box>(boundingBox),
                                            std::make_shared<geometry::Ring>(outline)};
            for (auto coordinates : getApplicableGridCells(boundingBox)) {
                gridCells[coordinates.first][coordinates.second].junctions.emplace_back(
                        std::make_shared<PolygonStruct>(junctionStruct));
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
//            std::cout << mVehicleDataProvider->getStationId() << " <-- " << senderStationId << std::endl;

//            traci::TraCIGeoPosition traciGeoPositionSelf = {
//                    mVehicleDataProvider->longitude().value(),
//                    mVehicleDataProvider->latitude().value()};
//            traci::TraCIPosition traciPositionSelf = traciAPI->convert2D(traciGeoPositionSelf);
//            Position ownPosition = Position(traciPositionSelf.x, traciPositionSelf.y);
//            auto &allObjects = mLocalEnvironmentModel->allObjects();
//            TrackedObjectsFilterRange envModObjects = filterBySensorCategory(allObjects, "CA");
//            CheckResult *result;
//            if (detectedSenders.find(senderStationId) != detectedSenders.end()) {
//                result = detectedSenders[senderStationId]->addAndCheckCam(message, ownPosition,
//                                                                          envModObjects);
//            } else {
//                detectedSenders[senderStationId] = new DetectedSender(traciAPI, &F2MDParameters::detectionParameters,
//                                                                      message);
//                result = detectedSenders[senderStationId]->addAndCheckCam(message, ownPosition,
//                                                                          envModObjects);
//            }
//            std::cout << result->toString(0.5) << std::endl;

            misbehaviorTypes::MisbehaviorTypes senderMisbehaviorType = getMisbehaviorTypeOfStationId(senderStationId);
//            if (senderMisbehaviorType == misbehaviorTypes::LocalAttacker) {
//                EV_INFO << "Received manipulated CAM!";
//                EV_INFO << "Received DoS " << simTime().inUnit(SimTimeUnit::SIMTIME_MS) << " delta:  "
//                        << message->cam.generationDeltaTime << "\n";
//                vanetza::ByteBuffer byteBuffer = ca->asn1().encode();
//                std::string encoded(byteBuffer.begin(), byteBuffer.end());
//                std::string b64Encoded = base64_encode(reinterpret_cast<const unsigned char *>(encoded.c_str()),
//                                                       encoded.length(), false);
//                if (curl) {
//                    curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:9981/newCAM");
//                    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, b64Encoded.c_str());
//                    CURLcode curlResponse = curl_easy_perform(curl);
//                    if (curlResponse != CURLE_OK) {
//                        EV_ERROR << "curl_easy_perform() failed: %s\n"
//                                 << curl_easy_strerror(curlResponse);
//                    }
//                }
//            } else if (senderMisbehaviorType == misbehaviorTypes::Benign) {
//                EV_INFO << "Received benign CAM!";
//            } else {
//                EV_INFO << "Received weird misbehaviorType";
//            }


            ReferencePosition_t referencePosition = message->cam.camParameters.basicContainer.referencePosition;
            traci::TraCIGeoPosition traciGeoPositionSender = {
                    (double) referencePosition.longitude / 10000000.0,
                    (double) referencePosition.latitude / 10000000.0};
            Position senderPosition = position_cast(simulationBoundary, traciAPI->convert2D(traciGeoPositionSender));
            bool messageHasPoi = false;
            if (senderMisbehaviorType == misbehaviorTypes::LocalAttacker) {
                for (int i = 0; i < F2MDParameters::miscParameters.objectAccessHelperGridSize; i++) {
                    if (senderPosition.x.value() >= gridCells[i][0].boundingBox.min_corner().get<0>() &&
                        senderPosition.x.value() <= gridCells[i][0].boundingBox.max_corner().get<0>()) {
                        for (int j = 0; j < F2MDParameters::miscParameters.objectAccessHelperGridSize; j++) {
                            if (senderPosition.y.value() >= gridCells[i][j].boundingBox.min_corner().get<1>() &&
                                senderPosition.y.value() <= gridCells[i][j].boundingBox.max_corner().get<1>()) {
                                for (const auto &obstacleStruct : gridCells[i][j].obstacles) {
                                    if (boost::geometry::within(senderPosition, *obstacleStruct->boundingBox)) {
                                        if (boost::geometry::within(senderPosition, *obstacleStruct->outline)) {
                                            if (!messageHasPoi) {
                                                std::string prefix = "inside obstacle";
                                                prefix += obstacleStruct->id;
                                                visualizeCamPosition(message, libsumo::TraCIColor(0, 255, 255, 255),
                                                                     prefix);
                                                messageHasPoi = true;
                                            }
                                        }
                                    }
                                }
                                for (const auto &laneStruct: gridCells[i][j].lanes) {
                                    if (boost::geometry::within(senderPosition, *laneStruct->boundingBox)) {
                                        if (boost::geometry::distance(senderPosition, *laneStruct->shape) <
                                            laneStruct->width / 2) {
                                            if (!messageHasPoi) {
                                                std::string prefix = "inside lane";
                                                prefix += laneStruct->id;
                                                visualizeCamPosition(message, libsumo::TraCIColor(255, 255, 0, 255),
                                                                     prefix);

                                                messageHasPoi = true;
                                            }
                                        }
                                    }
                                }
                                for (const auto &junctionStruct : gridCells[i][j].junctions) {
                                    if (boost::geometry::within(senderPosition, *junctionStruct->boundingBox)) {
                                        if (boost::geometry::within(senderPosition, *junctionStruct->outline)) {
                                            if (!messageHasPoi) {
                                                std::string prefix = "inside junction";
                                                prefix += junctionStruct->id;
                                                visualizeCamPosition(message, libsumo::TraCIColor(255, 0, 255, 255),
                                                                     prefix);

                                                messageHasPoi = true;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
//            if (!messageHasPoi) {
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
