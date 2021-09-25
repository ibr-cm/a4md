package artery.application.misbehavior.detection;

simple MisbehaviorDetectionService like artery.application.ItsG5Service
{
    parameters:
        string appName = default("MisbehaviorDetectionService");

        // ### detection parameters
        int checkType = default(0);
        double misbehaviorThreshold = default(0.5);
        int detectedSenderCamArrayMaxSize = default(10);

        double generateLevel4AndBelowProbability = default(1);
        double generateLevel3AndBelowProbability = default(1);
        double generateLevel2AndBelowProbability = default(1);
        double generateLevel1Probability = default(1);

        // in meters per second
        double maxPlausibleSpeed = default(60);
        double maxPlausibleAcceleration = default(10);
        double maxPlausibleDeceleration = default(10);

        double maxPlausibleRange = default(1500);

        double maxProximityRangeL = default(30);
        double maxProximityRangeW = default(3);
        double maxProximityDistance = default(2);
        double maxTimeDelta = default(3.1);
        double maxMgtRng = default(4);
        double maxMgtRngDown = default(6.2);
        double maxMgtRngUp = default(2.1);
        double maxSuddenAppearanceRange = default(420);
        double maxSuddenAppearanceTime = default(2.1);
        double maxCamFrequency = default(1.05);
        double maxOffroadSpeed = default(-1);
        double maxDistanceFromRoad = default(5);
        double positionHeadingTime = default(1.1);
        double maxHeadingChange = default(90);
        double maxIntersectionDeltaTime = default(2.0);

        double maxKalmanTime = default(3.1);
        double kalmanMinPosRange = default(4.0);
        double kalmanMinSpeedRange = default(4.0);
        double kalmanMinHeadingRange = default(4.0);
        double kalmanPosRange = default(1.0);
        double kalmanSpeedRange = default(4.0);

        // ### report parameters
        // makes no sense to make this bigger than resultArrayMaxSize
        int evidenceContainerMaxCamCount = default(5);
        int omittedReportsCount = default(5);
        bool omittedReportsCountPerErrorCode = default(false);
        bool broadcastReport = default(false);

}