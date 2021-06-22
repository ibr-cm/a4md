//
// Created by bastian on 22.06.21.
//

#ifndef ARTERY_GRIDCELLMATRIX_H
#define ARTERY_GRIDCELLMATRIX_H


#include <string>
#include <memory>
#include <artery/utility/Geometry.h>
#include <traci/Boundary.h>
#include <traci/API.h>
#include <artery/envmod/EnvironmentModelObstacle.h>

namespace artery {
    struct PolygonStruct {
        std::string id;
        std::shared_ptr<geometry::Box> boundingBox;
        std::shared_ptr<geometry::Ring> outline;
    };

    struct LaneStruct {
        std::string id;
        std::shared_ptr<geometry::Box> boundingBox;
        std::shared_ptr<geometry::LineString> shape;
        double width;
    };


    struct GridCell {
        geometry::Box boundingBox;
        std::vector<std::shared_ptr<PolygonStruct>> obstacles;
        std::vector<std::shared_ptr<PolygonStruct>> junctions;
        std::vector<std::shared_ptr<LaneStruct>> lanes;
    };

    class GridCellMatrix {
    public:
        void initialize(int gridSize, traci::API *traciAPI,ObstacleRtree *obstacleRtree, ObstacleDB *obstacles);

        GridCell getGridCellFromPosition(const Position& position) const;

    private:

        std::vector<std::pair<int, int>> getApplicableGridCells(geometry::Box boundingBox) const;

        void initializeGridCells();

        void initializeObstacles(ObstacleRtree *obstacleRtree, ObstacleDB *obstacles);

        void initializeLanes();

        void initializeJunctions();

        int mGridSize;
        std::vector<std::vector<GridCell>> mGridCells;
        traci::Boundary mSimulationBoundary;
        traci::API *mTraciAPI;
    };
}// namespace artery

#endif //ARTERY_GRIDCELLMATRIX_H
