//
// Created by bastian on 10.06.21.
//

#ifndef ARTERY_MISBEHAVIORCAPARAMETERS_H
#define ARTERY_MISBEHAVIORCAPARAMETERS_H

typedef struct MisbehaviorCaParameters {
    double LOCAL_ATTACKER_PROBABILITY;
    double GLOBAL_ATTACKER_PROBABILITY;
    double ATTACK_START_TIME;

    //Constant Position Attack
    double AttackConstantPositionMinLatitude;
    double AttackConstantPositionMaxLatitude;
    double AttackConstantPositionMinLongitude;
    double AttackConstantPositionMaxLongitude;

    //Constant Position Offset Attack
    double AttackConstantPositionOffsetMaxLatitudeOffset;
    double AttackConstantPositionOffsetMaxLongitudeOffset;

    // Random Position Attack
    double AttackRandomPositionMinLatitude;
    double AttackRandomPositionMaxLatitude;
    double AttackRandomPositionMinLongitude;
    double AttackRandomPositionMaxLongitude;

    // Random Position Offset Attack
    double AttackRandomPositionOffsetMaxLatitudeOffset;
    double AttackRandomPositionOffsetMaxLongitudeOffset;

    // Constant Speed Attack
    double AttackConstantSpeedMin;
    double AttackConstantSpeedMax;

    //Constant Speed Offset Attack
    double AttackConstantSpeedOffsetMax;

    // Random Speed Attack
    double AttackRandomSpeedMin;
    double AttackRandomSpeedMax;

    // Random Speed Offset Attack
    double AttackRandomSpeedOffsetMax;

    // Eventual Stop Attack
    double AttackEventualStopProbabilityThreshold;

    // Disruptive Attack
    int AttackDisruptiveBufferSize;
    int AttackDisruptiveMinimumReceived;

    // Denial of Service Attack
    int AttackDoSInterval;
    bool AttackDoSIgnoreDCC;

    // Stale Messages Attack
    int AttackStaleDelayCount;

    // CAM Location Visualizer (PoI)
    bool CamLocationVisualizer;
    int CamLocationVisualizerMaxLength;

} MisbehaviorCaParameters_t;

#endif //ARTERY_MISBEHAVIORCAPARAMETERS_H
