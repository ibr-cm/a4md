//
// Created by bastian on 22.06.21.
//

#include "GridCellMatrix.h"
#include <artery/traci/Cast.h>

#include <utility>

namespace artery {


    void GridCellMatrix::initializeGridCells() {
        mGridCells = std::vector<std::vector<GridCell >>(
                mGridSize,
                std::vector<GridCell>(mGridSize, GridCell()));


        geometry::Box bbox(geometry::Point(0, 0), geometry::Point(
                position_cast(mSimulationBoundary, mSimulationBoundary.upperRightPosition()).x.value(),
                position_cast(mSimulationBoundary, mSimulationBoundary.lowerLeftPosition()).y.value()));

        double gridLengthX = bbox.max_corner().get<0>() / mGridSize;
        double gridLengthY = bbox.max_corner().get<1>() / mGridSize;
        for (int i = 0; i < mGridSize; i++) {
            for (int j = 0; j < mGridSize; j++) {
                mGridCells[i][j].boundingBox = geometry::Box(geometry::Point(i * gridLengthX, j * gridLengthY),
                                                             geometry::Point((i + 1) * gridLengthX,
                                                                             (j + 1) * gridLengthY));

                boost::geometry::model::ring<geometry::Point, true, true> ring;
                boost::geometry::convert(mGridCells[i][j].boundingBox, ring);
                libsumo::TraCIPositionVector outline;
                for (const geometry::Point &p : ring) {
                    outline.value.emplace_back(position_cast(mSimulationBoundary, Position(p.get<0>(), p.get<1>())));
                }
                std::string id{"outlinePoly_" + std::to_string(i) + "_" + std::to_string(j)};
                mTraciAPI->polygon.add(id, outline, libsumo::TraCIColor(255, 0, 255, 255), false, "helper", 5);
            }
        }
    }

    std::vector<std::pair<int, int>> GridCellMatrix::getApplicableGridCells(geometry::Box boundingBox) const {
        std::vector<std::pair<int, int>> applicableCells;
        for (int i = 0; i < mGridSize; i++) {
            if (boundingBox.min_corner().get<0>() >= mGridCells[i][0].boundingBox.min_corner().get<0>() &&
                boundingBox.min_corner().get<0>() <= mGridCells[i][0].boundingBox.max_corner().get<0>() ||
                boundingBox.max_corner().get<0>() >= mGridCells[i][0].boundingBox.min_corner().get<0>() &&
                boundingBox.max_corner().get<0>() <= mGridCells[i][0].boundingBox.max_corner().get<0>()) {
                for (int j = 0; j < mGridSize; j++) {
                    if (boundingBox.min_corner().get<1>() >= mGridCells[i][j].boundingBox.min_corner().get<1>() &&
                        boundingBox.min_corner().get<1>() <= mGridCells[i][j].boundingBox.max_corner().get<1>() ||
                        boundingBox.max_corner().get<1>() >= mGridCells[i][j].boundingBox.min_corner().get<1>() &&
                        boundingBox.max_corner().get<1>() <= mGridCells[i][j].boundingBox.max_corner().get<1>()) {
                        applicableCells.emplace_back(i, j);
                    }
                }
            }
        }
        return applicableCells;
    }

    void GridCellMatrix::initializeObstacles(ObstacleRtree *obstacleRtree, ObstacleDB *obstacles) {
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
                mGridCells[coordinates.first][coordinates.second].obstacles.emplace_back(
                        std::make_shared<PolygonStruct>(obstacleStruct));
            }
        }
    }


    void GridCellMatrix::initializeLanes() {
        TraCIAPI::LaneScope laneScope = mTraciAPI->lane;
        std::vector<std::string> laneIDs = laneScope.getIDList();
        std::vector<std::string> junctionIDs = mTraciAPI->junction.getIDList();
        for (const auto &laneID : laneIDs) {
            if (std::find(junctionIDs.begin(), junctionIDs.end(), laneID.substr(1, laneID.find('_') - 1)) !=
                junctionIDs.end()) {
                continue;
            } else {
                libsumo::TraCIPositionVector shapeTraci = laneScope.getShape(laneID);
                geometry::LineString laneShape;
                for (const auto &v : shapeTraci.value) {
                    boost::geometry::append(laneShape, position_cast(mSimulationBoundary, v));
                }
                geometry::Box boundingBox = boost::geometry::return_envelope<geometry::Box>(laneShape);
                LaneStruct laneStruct = {laneID,
                                         std::make_shared<geometry::Box>(boundingBox),
                                         std::make_shared<geometry::LineString>(laneShape),
                                         laneScope.getWidth(laneID)};
                for (auto coordinates : getApplicableGridCells(boundingBox)) {
                    mGridCells[coordinates.first][coordinates.second].lanes.emplace_back(
                            std::make_shared<LaneStruct>(laneStruct));
                }
            }

        }
    }

    void GridCellMatrix::initializeJunctions() {
        TraCIAPI::JunctionScope junctionScope = mTraciAPI->junction;
        std::vector<std::string> junctionIDs = junctionScope.getIDList();
        for (const auto &junctionID : junctionIDs) {
            if (std::find(junctionIDs.begin(), junctionIDs.end(), junctionID.substr(1, junctionID.find('_') - 1)) !=
                junctionIDs.end()) {
                continue;
            }
            geometry::Ring outline;
            for (const auto &v: junctionScope.getShape(junctionID).value) {
                boost::geometry::append(outline, position_cast(mSimulationBoundary, v));
            }
            boost::geometry::correct(outline);

            geometry::Box boundingBox = boost::geometry::return_envelope<geometry::Box>(outline);
            PolygonStruct junctionStruct = {junctionID,
                                            std::make_shared<geometry::Box>(boundingBox),
                                            std::make_shared<geometry::Ring>(outline)};
            for (auto coordinates : getApplicableGridCells(boundingBox)) {
                mGridCells[coordinates.first][coordinates.second].junctions.emplace_back(
                        std::make_shared<PolygonStruct>(junctionStruct));
            }
        }
    }

    void GridCellMatrix::initialize(int gridSize, traci::API *traciAPI, ObstacleRtree *obstacleRtree,
                                    ObstacleDB *obstacles) {
        mGridSize = gridSize;
        mTraciAPI = traciAPI;
        mSimulationBoundary = traci::Boundary{mTraciAPI->simulation.getNetBoundary()};
        initializeGridCells();
        initializeObstacles(obstacleRtree, obstacles);
        initializeLanes();
        initializeJunctions();
    }

    GridCell GridCellMatrix::getGridCellFromPosition(const Position &position) const {
        std::cout << boost::geometry::dsv(position) << std::endl;
        for (int i = 0; i < mGridSize; i++) {
            if (position.x.value() >= mGridCells[i][0].boundingBox.min_corner().get<0>() &&
                position.x.value() <= mGridCells[i][0].boundingBox.max_corner().get<0>()) {
                for (int j = 0; j < mGridSize; j++) {
                    if (position.y.value() >= mGridCells[i][j].boundingBox.min_corner().get<1>() &&
                        position.y.value() <= mGridCells[i][j].boundingBox.max_corner().get<1>()) {
                        return mGridCells[i][j];
                    }
                }
            }
        }
        return GridCell();
    }
}// namespace artery