//
// Created by bastian on 18.06.21.
//

#ifndef ARTERY_CUSTOMRING_H
#define ARTERY_CUSTOMRING_H

#include <artery/utility/Geometry.h>
#include <traci/Boundary.h>

namespace artery{

    class CustomRing {
        typedef boost::geometry::model::d2::point_xy<double> point_type;
        typedef boost::geometry::model::ring<point_type> ring_type;

    public:
        void append(point_type point);
        void append(Position position);
        bool close();
        libsumo::TraCIPositionVector toTraCiPositionVector();
        CustomRing();
//        CustomRing(libsumo::TraCIPositionVector traCiPositionVector);
        CustomRing(const traci::Boundary& boundaryBox);
    private:
        ring_type ring;

    };
}


#endif //ARTERY_CUSTOMRING_H
