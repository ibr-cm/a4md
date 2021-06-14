/*******************************************************************************
 * @author  Joseph Kamel
 * @email   josephekamel@gmail.com
 * @date    28/11/2018
 * @version 2.0
 *
 * SCA (Secure Cooperative Autonomous systems)
 * Copyright (c) 2013, 2018 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/

#ifndef __VEINS_AttackTypes_H_
#define __VEINS_AttackTypes_H_

#include <iostream>

namespace attackTypes {

enum AttackTypes {
    Benign = 0,
    ConstPos,
    ConstPosOffset,
    RandomPos,
    RandomPosOffset,
    ConstSpeed,
    ConstSpeedOffset,
    RandomSpeed,
    RandomSpeedOffset,
    EventualStop,
    Disruptive,
    DataReplay,
    StaleMessages,
    DoS,
    DoSRandom,
    DoSDisruptive,
    GridSybil,
    DataReplaySybil,
    DoSRandomSybil,
    DoSDisruptiveSybil,
    MAStress,
    SIZE_OF_ENUM
};

static const char* AttackNames[] = { "Benign", "ConstPos", "ConstPosOffset",
        "RandomPos", "RandomPosOffset", "ConstSpeed", "ConstSpeedOffset",
        "RandomSpeed", "RandomSpeedOffset", "EventualStop", "Disruptive",
        "DataReplay", "StaleMessages", "DoS", "DoSRandom", "DoSDisruptive",
        "GridSybil", "DataReplaySybil", "DoSRandomSybil", "DoSDisruptiveSybil",
        "MAStress" };

static_assert(sizeof(attackTypes::AttackNames)/sizeof(char*) == attackTypes::SIZE_OF_ENUM
        , "sizes dont match");

static const attackTypes::AttackTypes intAttacks[] = { Benign, ConstPos,
        ConstPosOffset, RandomPos, RandomPosOffset, ConstSpeed,
        ConstSpeedOffset, RandomSpeed, RandomSpeedOffset, EventualStop,
        Disruptive, DataReplay, StaleMessages, DoS, DoSRandom, DoSDisruptive,
        GridSybil, DataReplaySybil, DoSRandomSybil, DoSDisruptiveSybil, MAStress };

static_assert(sizeof(attackTypes::intAttacks)/sizeof(attackTypes::AttackTypes) == attackTypes::SIZE_OF_ENUM
        , "sizes dont match");

}

#endif
