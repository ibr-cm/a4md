//
// Created by bastian on 18.06.21.
//

#include "CustomRing.h"

namespace artery{

    CustomRing::CustomRing() = default;

//    CustomRing::CustomRing(libsumo::TraCIPositionVector traCiPositionVector) {
//
//    }
    CustomRing::CustomRing(const traci::Boundary& boundaryBox) {
        ring = ring_type{};
        boost::geometry::append(ring,point_type(boundaryBox.lowerLeftPosition().x,boundaryBox.lowerLeftPosition().y));
        boost::geometry::append(ring,point_type(boundaryBox.lowerLeftPosition().x,boundaryBox.upperRightPosition().y));
        boost::geometry::append(ring,point_type(boundaryBox.upperRightPosition().x,boundaryBox.upperRightPosition().y));
        boost::geometry::append(ring,point_type(boundaryBox.upperRightPosition().x,boundaryBox.lowerLeftPosition().y));
        boost::geometry::correct(ring);
    }

    libsumo::TraCIPositionVector CustomRing::toTraCiPositionVector() {
        libsumo::TraCIPositionVector outline;
        for(const point_type& p : ring){
            outline.value.emplace_back(Position(p.x(),p.y()).toTraCIPosition());
        }
        return outline;
    }


}
