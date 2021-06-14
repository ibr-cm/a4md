typedef struct CheckResult {

    double proximityPlausibility;
    double rangePlausibility;
    double positionPlausibility;
    double speedPlausibility;

    double positionConsistency;
    double speedConsistency;
    double positionSpeedConsistency;
    double positionSpeedMaxConsistency;
    double positionHeadingConsistency;

    double intersection;
    double suddenAppearance;
    double frequency;
} CheckResult_t;