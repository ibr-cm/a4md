
package artery.application.md;

simple MisbehaviorCaService like artery.application.ItsG5Service
{
    parameters:
        @signal[CamReceived](type=CaObject);
        @signal[CamSent](type=CaObject);

        @statistic[reception](source=CamReceived;record=vector(camStationId)?,vector(camGenerationDeltaTime)?);
        @statistic[transmission](source=CamSent;record=vector(camStationId)?,vector(camGenerationDeltaTime)?);

        // evaluate DCC transmission interval restrictions
        bool withDccRestriction = default(true);

        // generation interval boundaries
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(1.0s);

        // generate at fixed rate (using minInterval, optionally restricted by DCC)
        bool fixedRate = default(false);

        // change in orientation triggering CAM generation (in degree)
        double headingDelta = default(4.0);

        // change in position triggering CAM generation
        double positionDelta @unit(m) = default(4.0m);

        // change in speed triggering CAM generation (in meter/second)
        double speedDelta @unit(mps) = default(0.5mps);

        // length of path history
        volatile int pathHistoryLength = default(23);

        
        string appName = default("MisbehaviorCaService");

        double START_ATTACK = default(10);

        double LOCAL_ATTACKER_PROB = default(0.05);
        int LOCAL_ATTACK_TYPE = default(7);

        double GLOBAL_ATTACKER_PROB = default(0.0);
        int GLOBAL_ATTACK_TYPE = default(20);

        double MaxRandomOffsetLatitude = default(0.001);
        double MaxRandomOffsetLongitude = default(0.001);

        // ------ Attacks Parameters -- Start

        // Constant Position Attack
        double AttackConstantPositionMinLatitude = default(49.58);
        double AttackConstantPositionMaxLatitude = default(49.63);
        double AttackConstantPositionMinLongitude = default(6.09);
        double AttackConstantPositionMaxLongitude = default(6.16);

        // Constant Position Offset Attack
        double AttackConstantPositionOffsetMaxLatitudeOffset = default(0.001);
        double AttackConstantPositionOffsetMaxLongitudeOffset = default(0.001);

        // Random Position Attack
        double AttackRandomPositionMinLatitude = default(49.58);
        double AttackRandomPositionMaxLatitude = default(49.63);
        double AttackRandomPositionMinLongitude = default(6.09);
        double AttackRandomPositionMaxLongitude = default(6.16);

        // Random Position Offset Attack
        double AttackRandomPositionOffsetMaxLatitudeOffset = default(0.001);
        double AttackRandomPositionOffsetMaxLongitudeOffset = default(0.001);

        // Constant Speed Attack
        // Meters per Second
        double AttackConstantSpeedMin = default(0.0);
        double AttackConstantSpeedMax = default(163.82);

        // Constant Speed Offset Attack
        // only adds speed so that there aren't negative values
        double AttackConstantSpeedOffsetMax = default(5.0);

        // Random Speed Attack
        // Meters per Second
        double AttackRandomSpeedMin = default(0.0);
        double AttackRandomSpeedMax = default(163.82);

        // Random Speed Offset Attack
        // only adds speed so that there aren't negative values
        double AttackRandomSpeedOffsetMax = default(5.0);

        // Eventual Stop Attack
        double AttackEventualStopProbabilityThreshold = default(0.05);

		// Disruptive Attack
		int AttackDisruptiveBufferSize = default(500);
		int AttackDisruptiveMinimumReceived = default(50);

        // Denial of Service Attack
        // Interval in ms
        // Keep in mind that appl.middleware.updateInterval might need to be customized
		int AttackDoSInterval = default(10);
		bool AttackDoSIgnoreDCC = default(true);


		// Stale Messages Attack
		int AttackStaleDelayCount = default(50);

}