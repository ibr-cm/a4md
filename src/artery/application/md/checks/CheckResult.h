#include <vanetza/asn1/cam.hpp>

#ifndef ARTERY_CHECKRESULT_H
#define ARTERY_CHECKRESULT_H

#include <string>
#include <iostream>
#include <sstream>

struct CheckResult {

    vanetza::asn1::Cam cam;

    double positionPlausibility = 1;
    double speedPlausibility = 1;
    double proximityPlausibility = 1;
    double rangePlausibility = 1;

    bool consistencyIsChecked = false;
    double positionConsistency = 1;
    double speedConsistency = 1;
    double positionSpeedConsistency = 1;
    double positionSpeedMaxConsistency = 1;
    double positionHeadingConsistency = 1;

    double kalmanPositionSpeedConsistencyPosition = 1;
    double kalmanPositionSpeedConsistencySpeed = 1;
    double kalmanPositionSpeedScalarConsistencyPosition = 1;
    double kalmanPositionSpeedScalarConsistencySpeed = 1;
    double kalmanPositionConsistencyConfidence = 1;
    double kalmanPositionAccelerationConsistencySpeed = 1;
    double kalmanSpeedConsistencyConfidence = 1;

    double intersection = 1;
    double suddenAppearance = 1;
    double frequency = 1;

    std::string toString(double threshold) const {
        std::stringstream ss;
        ss.precision(2);
        ss << "Plausibility: " << std::endl;
        ss << attributeToString("Position", 1, positionPlausibility, threshold);
        ss << attributeToString("Speed", 1, speedPlausibility, threshold);
        ss << attributeToString("Proximity", 1, proximityPlausibility, threshold);
        ss << attributeToString("Range", 1, rangePlausibility, threshold);
        if (consistencyIsChecked) {
            ss << "Consistency: " << std::endl;
            ss << attributeToString("Position", 1, positionConsistency, threshold);
            ss << attributeToString("Speed", 1, speedConsistency, threshold);
            ss << attributeToString("PositionSpeed", 1, positionSpeedConsistency, threshold);
            ss << attributeToString("PositionSpeedMax", 1, positionSpeedMaxConsistency, threshold);
            ss << attributeToString("PositionHeading", 1, positionHeadingConsistency, threshold);
            ss << "\tKalman: " << std::endl;
            ss << attributeToString("PositionSpeed-Position", 2, kalmanPositionSpeedConsistencyPosition, threshold);
            ss << attributeToString("PositionSpeed-Speed", 2, kalmanPositionSpeedConsistencySpeed, threshold);
            ss << attributeToString("PositionSpeedScalar-Position", 2, kalmanPositionSpeedScalarConsistencyPosition,
                                    threshold);
            ss << attributeToString("PositionSpeedScalar-Speed", 2, kalmanPositionSpeedScalarConsistencySpeed,
                                    threshold);
            ss << attributeToString("Position", 2, kalmanPositionConsistencyConfidence, threshold);
            ss << attributeToString("PositionAcceleration-Speed", 2, kalmanPositionAccelerationConsistencySpeed,
                                    threshold);
            ss << attributeToString("Speed", 2, kalmanSpeedConsistencyConfidence, threshold);
            ss << attributeToString("Frequency", 0, frequency, threshold);
            ss << attributeToString("Intersection", 0, intersection, threshold);
        } else {
            ss << attributeToString("Sudden Appearance", 0, suddenAppearance, threshold);
        }

        return ss.str();
    };

    static std::string attributeToString(const std::string &name, int tabCount, double value, double threshold) {
        std::stringstream ss;
        ss.precision(2);
        for (int i = 0; i < tabCount; i++) {
            ss << "  ";
        }
        if (value < threshold) {
            ss << "\x1B[31m";
        } else {
            ss << "\x1B[32m";
        }
        ss << name << ": " << value << "\033[0m" << std::endl;
        return ss.str();
    };
};

#endif //ARTERY_CHECKRESULT_H