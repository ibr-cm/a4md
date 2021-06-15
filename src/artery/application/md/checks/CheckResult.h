#include <vanetza/asn1/cam.hpp>

struct CheckResult {

    vanetza::asn1::Cam cam;

    double positionPlausibility{};
    double speedPlausibility{};
    double proximityPlausibility{};
    double rangePlausibility{};

    double positionConsistency{};
    double speedConsistency{};
    double positionSpeedConsistency{};
    double positionSpeedMaxConsistency{};
    double positionHeadingConsistency{};

    double kalmanPositionSpeedConsistencyPosition{};
    double kalmanPositionSpeedConsistencySpeed{};
    double kalmanPositionSpeedScalarConsistencyPosition{};
    double kalmanPositionSpeedScalarConsistencySpeed{};
    double kalmanPositionConsistencyConfidence{};
    double kalmanPositionAccelerationConsistencySpeed{};
    double kalmanSpeedConsistencyConfidence{};

    double intersection{};
    double suddenAppearance{};
    double frequency{};
};