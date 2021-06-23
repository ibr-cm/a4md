/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ENVIRONMENTMODELOBSTACLE_H_
#define ENVIRONMENTMODELOBSTACLE_H_

#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/EnvironmentModelAbstract.h"
#include "artery/utility/Geometry.h"
#include <boost/geometry/index/rtree.hpp>
#include <string>
#include <utility>
#include <vector>

namespace artery {

/**
 * Representation of an obstacle inside the environment model
 */
    class EnvironmentModelPolygon : public EnvironmentModelAbstract {
    public:
        EnvironmentModelPolygon(std::string id, std::vector<Position> outline) :
                mPolygon(std::move(outline)) {
            mId = std::move(id);
        }

        /**
         * Returns the obstacle coords
         * @return obstacle coords
         */
        const std::vector<Position> &getOutline() const { return mPolygon; }

    private:
        std::vector<Position> mPolygon; //!< Obstacle outline
    };

    using PolygonDB = std::map<std::string, std::shared_ptr<EnvironmentModelPolygon>>;
//    using GeometryRtreeValue = std::pair<geometry::Box, std::string>;
//    using GeometryRtree = boost::geometry::index::rtree<GeometryRtreeValue, boost::geometry::index::rstar<16>>;

} // namespace artery

#endif /* ENVIRONMENTMODELOBSTACLE_H_ */
