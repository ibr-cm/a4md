//
// Created by bastian on 23.06.21.
//

#ifndef ARTERY_ENVIRONMENTMODELABSTRACT_H
#define ARTERY_ENVIRONMENTMODELABSTRACT_H

#include "artery/utility/Geometry.h"
#include <string>
#include <utility>

namespace artery {

    class EnvironmentModelAbstract {

    public:
        /**
             * Returns the id
             * @return id
             */
        const std::string &getId() const { return mId; }

    protected:
        std::string mId; //!< Unique id
    };

    using GeometryRtreeValue = std::pair<geometry::Box, std::string>;
    using GeometryRtree = boost::geometry::index::rtree<GeometryRtreeValue, boost::geometry::index::rstar<16>>;
} // namespace artery

#endif //ARTERY_ENVIRONMENTMODELABSTRACT_H
