typedef struct CheckResult {
    double rangePlausibility;
    double positionPlausibility;
    double speedPlausibility;

    double positionConsinstency;
    double speedConsistency;
    double positionSpeedConsistency;
} CheckResult_t;