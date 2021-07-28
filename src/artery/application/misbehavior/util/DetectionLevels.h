//
// Created by bastian on 21.07.21.
//

#ifndef ARTERY_DETECTIONLEVELS_H
#define ARTERY_DETECTIONLEVELS_H

#endif //ARTERY_DETECTIONLEVELS_H
namespace artery {

    namespace detectionLevels {

        enum DetectionLevels {
            Level1 = 0,
            Level2 = 1,
            Level3 = 2,
            Level4 = 3,
            SIZE_OF_ENUM
        };

        const DetectionLevels DetectionLevelVector[] = {detectionLevels::Level1, detectionLevels::Level2,
                                                        detectionLevels::Level3, detectionLevels::Level4};

        static const char *DetectionLevelStrings[] = {"1", "2", "3", "4"};

    }

} // namespace artery