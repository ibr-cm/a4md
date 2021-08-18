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

#ifndef ARTERY_ATTACKTYPES_H
#define ARTERY_ATTACKTYPES_H

#include <iostream>

namespace artery {

    namespace attackTypes {

        enum AttackTypes {
            Benign = 0,
            ConstPos = 1,
            ConstPosOffset = 2,
            RandomPos = 3,
            RandomPosOffset = 4,
            ConstSpeed = 5,
            ConstSpeedOffset = 6,
            RandomSpeed = 7,
            RandomSpeedOffset = 8,
            EventualStop = 9,
            Disruptive = 10,
            DataReplay = 11,
            StaleMessages = 12,
            DoS = 13,
            DoSRandom = 14,
            DoSDisruptive = 15,
            GridSybil = 16,
            DataReplaySybil = 17,
            DoSRandomSybil = 18,
            DoSDisruptiveSybil = 19,
            FakeReport = 20,
            SIZE_OF_ENUM
        };

        static const char *AttackNames[] = {"Benign", "ConstPos", "ConstPosOffset",
                                            "RandomPos", "RandomPosOffset", "ConstSpeed", "ConstSpeedOffset",
                                            "RandomSpeed", "RandomSpeedOffset", "EventualStop", "Disruptive",
                                            "DataReplay", "StaleMessages", "DoS", "DoSRandom", "DoSDisruptive",
                                            "GridSybil", "DataReplaySybil", "DoSRandomSybil", "DoSDisruptiveSybil",
                                            "MAStress"};

        static_assert(sizeof(attackTypes::AttackNames) / sizeof(char *) == attackTypes::SIZE_OF_ENUM,
                      "sizes dont match");

        static const attackTypes::AttackTypes intAttacks[] = {Benign, ConstPos,
                                                              ConstPosOffset, RandomPos, RandomPosOffset, ConstSpeed,
                                                              ConstSpeedOffset, RandomSpeed, RandomSpeedOffset,
                                                              EventualStop,
                                                              Disruptive, DataReplay, StaleMessages, DoS, DoSRandom,
                                                              DoSDisruptive,
                                                              GridSybil, DataReplaySybil, DoSRandomSybil,
                                                              DoSDisruptiveSybil, FakeReport};

        static_assert(sizeof(attackTypes::intAttacks) / sizeof(attackTypes::AttackTypes) == attackTypes::SIZE_OF_ENUM,
                      "sizes dont match");

    }
} //namespace artery

#endif
