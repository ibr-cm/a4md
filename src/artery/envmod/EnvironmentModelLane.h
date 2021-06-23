//
// Created by bastian on 23.06.21.
//

#ifndef ARTERY_ENVIRONMENTMODELLANE_H
#define ARTERY_ENVIRONMENTMODELLANE_H

#include <utility>

#include "artery/envmod/EnvironmentModelAbstract.h"
namespace artery {

    class EnvironmentModelLane : public EnvironmentModelAbstract {
    public:
        EnvironmentModelLane(std::string id, geometry::LineString shape, double width) :
                 mShape(std::move(shape)), mWidth(width) {
            mId = std::move(id);
        }

        /**
         * Returns the lane coords
         * @return lane coords
         */
        const geometry::LineString &getShape() const { return mShape; }

        const double &getWidth() const { return mWidth; };

    private:
        geometry::LineString mShape; //!< Lane shape
        double mWidth; //!< Lane Width
    };

    using LaneDB = std::map<std::string, std::shared_ptr<EnvironmentModelLane>>;

} // namespace artery



#endif //ARTERY_ENVIRONMENTMODELLANE_H
