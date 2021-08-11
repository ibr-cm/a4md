//
// Created by bastian on 07.07.21.
//

#include "ThresholdFusion.h"
#include "artery/application/misbehavior/util/DetectionLevels.h"

namespace artery {

    ThresholdFusion::ThresholdFusion(double threshold) :
            threshold(threshold) {
    }

    std::vector<std::bitset<16>> ThresholdFusion::checkForReport(const CheckResult &checkResult) {

        std::vector<std::bitset<16>> detectionLevelErrorCodes(4);

        if (checkResult.rangePlausibility < threshold) {
            detectionLevelErrorCodes[detectionLevels::Level4][0] = true;
        }
        if (checkResult.speedPlausibility < threshold) {
            detectionLevelErrorCodes[detectionLevels::Level1][2] = true;
        }
        if (checkResult.positionPlausibility < threshold) {
            detectionLevelErrorCodes[detectionLevels::Level3][0] = true;
        }
        if (checkResult.intersection < threshold) {
            detectionLevelErrorCodes[detectionLevels::Level3][0] = true;
        }
        if (checkResult.proximityPlausibility < threshold) {
            detectionLevelErrorCodes[detectionLevels::Level4][0] = true;
        }
        if (checkResult.consistencyIsChecked) {
            if (checkResult.positionConsistency < threshold) {
                detectionLevelErrorCodes[detectionLevels::Level2][2] = true;
            }
            if (checkResult.speedConsistency < threshold) {
                detectionLevelErrorCodes[detectionLevels::Level2][2] = true;
            }
            if (checkResult.positionSpeedConsistency < threshold) {
                detectionLevelErrorCodes[detectionLevels::Level2][0] = true;
            }
            if (checkResult.positionSpeedMaxConsistency < threshold) {
                detectionLevelErrorCodes[detectionLevels::Level2][0] = true;
            }
            if (checkResult.positionHeadingConsistency < threshold) {
                detectionLevelErrorCodes[detectionLevels::Level2][1] = true;
            }
            if (checkResult.kalmanPositionSpeedConsistencyPosition < threshold) {
                detectionLevelErrorCodes[detectionLevels::Level2][0] = true;
            }
            if (checkResult.kalmanPositionSpeedConsistencySpeed < threshold) {
                detectionLevelErrorCodes[detectionLevels::Level2][2] = true;
            }
            if (checkResult.kalmanPositionSpeedScalarConsistencyPosition < threshold) {
                detectionLevelErrorCodes[detectionLevels::Level2][0] = true;
            }
            if (checkResult.kalmanPositionSpeedScalarConsistencySpeed < threshold) {
                detectionLevelErrorCodes[detectionLevels::Level2][2] = true;
            }
            if (checkResult.kalmanPositionConsistencyConfidence < threshold) {
                detectionLevelErrorCodes[detectionLevels::Level2][0] = true;
            }
            if (checkResult.kalmanPositionAccelerationConsistencySpeed < threshold) {
                detectionLevelErrorCodes[detectionLevels::Level2][0] = true;
            }
            if (checkResult.kalmanSpeedConsistencyConfidence < threshold) {
                detectionLevelErrorCodes[detectionLevels::Level2][2] = true;
            }
        }

//        static int counter = 0;

//        detectionLevelErrorCodes[detectionLevels::Level1][0] = true;
//        if (counter % 2 == 0) {
//            detectionLevelErrorCodes[detectionLevels::Level2][1] = true;
//        }
//        if (counter % 3 == 0) {
//            detectionLevelErrorCodes[detectionLevels::Level1][2] = true;
//            detectionLevelErrorCodes[detectionLevels::Level2][2] = true;
//        }
//        counter++;
        return detectionLevelErrorCodes;
    }

} // namespace artery