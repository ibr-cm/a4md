//
// Created by bastian on 07.07.21.
//

#include "ThresholdFusion.h"
#include "artery/application/md/util/DetectionLevels.h"

namespace artery {

    ThresholdFusion::ThresholdFusion(double threshold) :
            threshold(threshold) {
    }


    /*DetectionReferenceCAM_t *ThresholdFusion::checkForReport(CheckResult &checkResult) {

        auto *semanticDetectionReferenceCam = new DetectionReferenceCAM_t();
        std::bitset<16> semanticDetectionErrorCodeCAM(0);
        int detectionLevelCam = 0;
        std::vector<std::bitset<16>> detectionLevelErrorCodes(4);

        if (checkResult.rangePlausibility < threshold) {
            detectionLevelErrorCodes[DetectionLevels::Level1][0] = true;
            semanticDetectionErrorCodeCAM[0] = true;
            detectionLevelCam = 1;
        } else if (checkResult.speedPlausibility < threshold) {
            semanticDetectionErrorCodeCAM[2] = true;
            detectionLevelCam = 1;
            detectionLevelErrorCodes[DetectionLevels::Level1][2] = true;
        } else if (checkResult.consistencyIsChecked) {
            if (checkResult.positionConsistency < threshold) {
                semanticDetectionErrorCodeCAM[0] = true;
                detectionLevelCam = 2;
                detectionLevelErrorCodes[DetectionLevels::Level2][2] = true;
            } else if (checkResult.speedConsistency < threshold) {
                semanticDetectionErrorCodeCAM[2] = true;
                detectionLevelCam = 2;
                detectionLevelErrorCodes[DetectionLevels::Level2][2] = true;
            } else if (checkResult.positionSpeedConsistency < threshold) {
                semanticDetectionErrorCodeCAM[0] = true;
                detectionLevelCam = 2;
                detectionLevelErrorCodes[DetectionLevels::Level2][0] = true;
            } else if (checkResult.positionSpeedMaxConsistency < threshold) {
                semanticDetectionErrorCodeCAM[0] = true;
                detectionLevelCam = 2;
                detectionLevelErrorCodes[DetectionLevels::Level2][0] = true;
            } else if (checkResult.positionHeadingConsistency < threshold) {
                semanticDetectionErrorCodeCAM[1] = true;
                detectionLevelCam = 2;
                detectionLevelErrorCodes[DetectionLevels::Level2][1] = true;
            } else if (checkResult.kalmanPositionSpeedConsistencyPosition < threshold) {
                semanticDetectionErrorCodeCAM[0] = true;
                detectionLevelCam = 2;
                detectionLevelErrorCodes[DetectionLevels::Level2][0] = true;
            } else if (checkResult.kalmanPositionSpeedConsistencySpeed < threshold) {
                semanticDetectionErrorCodeCAM[2] = true;
                detectionLevelCam = 2;
                detectionLevelErrorCodes[DetectionLevels::Level2][2] = true;
            } else if (checkResult.kalmanPositionSpeedScalarConsistencyPosition < threshold) {
                semanticDetectionErrorCodeCAM[0] = true;
                detectionLevelCam = 2;
                detectionLevelErrorCodes[DetectionLevels::Level2][0] = true;
            } else if (checkResult.kalmanPositionSpeedScalarConsistencySpeed < threshold) {
                semanticDetectionErrorCodeCAM[2] = true;
                detectionLevelCam = 2;
                detectionLevelErrorCodes[DetectionLevels::Level2][2] = true;
            } else if (checkResult.kalmanPositionConsistencyConfidence < threshold) {
                semanticDetectionErrorCodeCAM[0] = true;
                detectionLevelCam = 2;
                detectionLevelErrorCodes[DetectionLevels::Level2][0] = true;
            } else if (checkResult.kalmanPositionAccelerationConsistencySpeed < threshold) {
                semanticDetectionErrorCodeCAM[0] = true;
                detectionLevelCam = 2;
                detectionLevelErrorCodes[DetectionLevels::Level2][0] = true;
            } else if (checkResult.kalmanSpeedConsistencyConfidence < threshold) {
                semanticDetectionErrorCodeCAM[2] = true;
                detectionLevelCam = 2;
                detectionLevelErrorCodes[DetectionLevels::Level2][2] = true;
            }
        } else if (checkResult.positionPlausibility < threshold) {
            semanticDetectionErrorCodeCAM[0] = true;
            detectionLevelCam = 3;
            detectionLevelErrorCodes[DetectionLevels::Level3][0] = true;
        } else if (checkResult.intersection < threshold) {
            semanticDetectionErrorCodeCAM[0] = true;
            detectionLevelCam = 3;
            detectionLevelErrorCodes[DetectionLevels::Level3][0] = true;
        } else if (checkResult.proximityPlausibility < threshold) {
            semanticDetectionErrorCodeCAM[0] = true;
            detectionLevelCam = 4;
            detectionLevelErrorCodes[DetectionLevels::Level4][0] = true;
        }

//        if(detectionLevelCam > 0){
//        std::cout << "ThresholdFusion: isReporable" << std::endl;
//        }

        semanticDetectionReferenceCam->detectionLevelCAM = detectionLevelCam;
        std::string encoded = semanticDetectionErrorCodeCAM.to_string();
        OCTET_STRING_fromBuf(&semanticDetectionReferenceCam->semanticDetectionErrorCodeCAM,
                             encoded.c_str(), (int) strlen(encoded.c_str()));
        return semanticDetectionReferenceCam;
    }*/

    std::vector<std::bitset<16>> ThresholdFusion::checkForReport(const CheckResult &checkResult) {

        std::vector<std::bitset<16>> detectionLevelErrorCodes(4);

        if (checkResult.rangePlausibility < threshold) {
            detectionLevelErrorCodes[detectionLevels::Level1][0] = true;
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

        detectionLevelErrorCodes[detectionLevels::Level1][0] = true;
        detectionLevelErrorCodes[detectionLevels::Level1][2] = true;

        detectionLevelErrorCodes[detectionLevels::Level2][1] = true;
        detectionLevelErrorCodes[detectionLevels::Level2][2] = true;
        return detectionLevelErrorCodes;
    }

} // namespace artery