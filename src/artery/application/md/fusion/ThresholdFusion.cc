//
// Created by bastian on 07.07.21.
//

#include "ThresholdFusion.h"
#include <bitset>

namespace artery {

    ThresholdFusion::ThresholdFusion(double threshold) :
            threshold(threshold) {
    }

    DetectionReferenceCAM_t *ThresholdFusion::checkForReport(CheckResult &checkResult) {

        bool isReportable = false;
        auto *semanticDetectionReferenceCam = new DetectionReferenceCAM_t();
        std::bitset<16> semanticDetectionErrorCodeCAM(0);
        int detectionLevelCam = 0;
        if (checkResult.positionPlausibility < threshold ||
            checkResult.speedPlausibility < threshold ||
            checkResult.proximityPlausibility < threshold ||
            checkResult.rangePlausibility < threshold ||
            checkResult.intersection < threshold ||
            checkResult.suddenAppearance < threshold ||
            checkResult.frequency < threshold) {
            isReportable = true;
        }
        if (checkResult.consistencyIsChecked) {
            if (checkResult.positionConsistency < threshold ||
                checkResult.speedConsistency < threshold ||
                checkResult.positionSpeedConsistency < threshold ||
                checkResult.positionSpeedMaxConsistency < threshold ||
                checkResult.positionHeadingConsistency < threshold ||

                checkResult.kalmanPositionSpeedConsistencyPosition < threshold ||
                checkResult.kalmanPositionSpeedConsistencySpeed < threshold ||
                checkResult.kalmanPositionSpeedScalarConsistencyPosition < threshold ||
                checkResult.kalmanPositionSpeedScalarConsistencySpeed < threshold ||
                checkResult.kalmanPositionConsistencyConfidence < threshold ||
                checkResult.kalmanPositionAccelerationConsistencySpeed < threshold ||
                checkResult.kalmanPositionAccelerationConsistencySpeed < threshold ||
                checkResult.kalmanPositionAccelerationConsistencySpeed < threshold ||
                checkResult.kalmanSpeedConsistencyConfidence < threshold) {
                isReportable = true;
            }
        }

        if(checkResult.rangePlausibility < threshold){
            semanticDetectionErrorCodeCAM[0] = true;
            detectionLevelCam = 1;
        } else if(checkResult.speedPlausibility < threshold){
            semanticDetectionErrorCodeCAM[2] = true;
            detectionLevelCam = 1;
        } else if (checkResult.positionPlausibility < threshold){
            semanticDetectionErrorCodeCAM[0] = true;
            detectionLevelCam = 3;
        } else if(checkResult.proximityPlausibility < threshold){
            semanticDetectionErrorCodeCAM[0] = true;
            detectionLevelCam = 4;
        }
        
        if(checkResult.consistencyIsChecked){
            if (checkResult.positionConsistency < threshold) {
                semanticDetectionErrorCodeCAM[0] = true;
                detectionLevelCam = 2;
            } else if(checkResult.speedConsistency < threshold){
                semanticDetectionErrorCodeCAM[2] = true;
                detectionLevelCam = 2;
            } else if(checkResult.positionSpeedConsistency < threshold){
                semanticDetectionErrorCodeCAM[0] = true;
                detectionLevelCam = 2;
            } else if (checkResult.positionSpeedMaxConsistency < threshold){
                semanticDetectionErrorCodeCAM[0] = true;
                detectionLevelCam = 2;
            } else if (checkResult.positionHeadingConsistency < threshold){
                semanticDetectionErrorCodeCAM[1] = true;
                detectionLevelCam = 2;
            }
        }
        
        std::cout << "isReportable: " << isReportable << std::endl;

        semanticDetectionReferenceCam->detectionLevelCAM = detectionLevelCam;
        std::string encoded = semanticDetectionErrorCodeCAM.to_string();
        OCTET_STRING_fromBuf(&semanticDetectionReferenceCam->semanticDetectionErrorCodeCAM,
                             encoded.c_str(), (int) strlen(encoded.c_str()));
        return semanticDetectionReferenceCam;
    }

} // namespace artery