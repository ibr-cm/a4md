/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef GLOBALENVIRONMENTMODEL_H_
#define GLOBALENVIRONMENTMODEL_H_


 #include "artery/envmod/sensor/Sensor.h"
#include "artery/envmod/sensor/SensorDetection.h"
#include "artery/envmod/Geometry.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/EnvironmentModelPolygon.h"
#include "artery/envmod/EnvironmentModelAbstract.h"
#include "artery/envmod/EnvironmentModelLane.h"
#include "artery/utility/Geometry.h"
#include <omnetpp/ccanvas.h>
#include <omnetpp/clistener.h>
#include <omnetpp/csimplemodule.h>
#include <boost/geometry/index/rtree.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <map>
#include <memory>
#include <string>


namespace traci {
    class API;

    class VehicleController;
}

namespace artery {

    class EnvironmentModelPolygon;

    class IdentityRegistry;

    class PreselectionMethod;

/**
 * Implementation of the environment model.
 */
    class GlobalEnvironmentModel : public omnetpp::cSimpleModule, public omnetpp::cListener {
    public:
        GlobalEnvironmentModel();

        virtual ~GlobalEnvironmentModel();

        // cSimpleModule life-cycle
        void initialize() override;

        void finish() override;

        // cListener handlers
        void receiveSignal(omnetpp::cComponent *, omnetpp::simsignal_t, const omnetpp::SimTime &,
                           omnetpp::cObject *) override;

        void receiveSignal(omnetpp::cComponent *, omnetpp::simsignal_t, const char *, omnetpp::cObject *) override;

        void receiveSignal(omnetpp::cComponent *, omnetpp::simsignal_t, unsigned long, omnetpp::cObject *) override;

        /**
         * Fetch an object by its external id.
         * @param externalID
         * @return model object matching external id
         */
        std::shared_ptr<EnvironmentModelObject> getObject(const std::string &objId);

        /**
         * Get an obstacle by its id
         * @param obsId obstacle id
         * @return obstacle model matching the id or nullptr
         */
        std::shared_ptr<EnvironmentModelPolygon> getObstacle(const std::string &obsId);

        std::shared_ptr<EnvironmentModelLane> getLane(const std::string &laneId);

        std::shared_ptr<EnvironmentModelPolygon> getJunction(const std::string &junctionId);

        /**
         * Returns GSDE of all objects in based on the detection logic of the sensor
         * @param detect
         * @return
         */
        SensorDetection detectObjects(std::function<SensorDetection(GeometryRtree &, PreselectionMethod &)> detect);

        GeometryRtree *getObstacleRTree() {
            return &mObstacleRtree;
        }

        GeometryRtree  *getLaneRTree(){
            return &mLaneRtree;
        }

        GeometryRtree *getJunctionRTree(){
            return &mJunctionRtree;
        }


    private:
        /**
         * Refresh all dynamic objects in the database.
         */
        void refresh();

        /**
         * Add vehicle to the environment database
         * @param vehicle TraCI mobility corresponding to vehicle
         * @return true if successful
         */
        bool addVehicle(traci::VehicleController *vehicle);

        /**
         * Remove vehicle from the database
         * @param nodeId TraCI id of vehicle to be removed
         * @return true if the vehicle is successfully removed
         */
        bool removeVehicle(std::string nodeId);

        /**
         * Remove all known vehicles from internal database
         */
        void removeVehicles();

        /**
         * Add (static) obstacles to the obstacle database
         * @param id Obstacle's id
         * @param outline Obstacle's outline
         * @return true if it could be added
         */
        bool addObstacle(std::string id, std::vector<Position> outline);

        bool addLane(const std::string& id, const std::vector<Position>&shape, double width);

        bool addJunction(const std::string& id, std::vector<Position> outline);

        /**
         * Create the obstacle rtree.
         * This method should be called after all static obstacles have been added.
         */
        void buildObstacleRtree();

        void buildLaneRTree();

        void buildJunctionRTree();

        /**
         * Clears the internal database completely
         */
        void clear();

        /**
         * Fetch static obstacles (polygons) from TraCI
         * @param api TraCI API object
         */
        void fetchObstacles(const traci::API &api);

        void fetchLanes(const traci::API &api);

        void fetchJunctions(const traci::API &api);

        /**
         * Try to get vehicle controller corresponding to given module
         * @param mod vehicle host module
         * @return nullptr if no vehicle controller is available
         */
        virtual traci::VehicleController *getVehicleController(omnetpp::cModule *mod);

        ObjectDB mObjects;
        PolygonDB mObstacles;
        GeometryRtree mObstacleRtree;
        LaneDB mLanes;
        PolygonDB mJunctions;
        GeometryRtree mLaneRtree;
        GeometryRtree mJunctionRtree;
        std::unique_ptr<PreselectionMethod> mPreselector;
        IdentityRegistry *mIdentityRegistry;
        bool mTainted;
        omnetpp::cGroupFigure *mDrawObstacles = nullptr;
        omnetpp::cGroupFigure *mDrawVehicles = nullptr;
        std::set<std::string> mObstacleTypes;

    };

} // namespace artery

#endif /* GLOBALENVIRONMENTMODEL_H_ */
