
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

        double MaxRandomOffsetLatitude = default(0.001)
        double MaxRandomOffsetLongitude = default(0.001)

}