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

#ifndef ARTERY_CHECKTYPES_H
#define ARTERY_CHECKTYPES_H

#include <iostream>

namespace artery {

    namespace misbehaviorTypes {

        enum MisbehaviorTypes {
            Benign = 0,
            LocalAttacker,
            GlobalAttacker,
            SIZE_OF_ENUM
        };

        static const char *mbNames[] = {"Benign", "LocalAttacker", "GlobalAttacker"};

        static const MisbehaviorTypes intMbs[] = {Benign, LocalAttacker, GlobalAttacker};

        static_assert(sizeof(misbehaviorTypes::mbNames) / sizeof(char *) == misbehaviorTypes::SIZE_OF_ENUM,
                      "sizes dont match");

        static_assert(sizeof(misbehaviorTypes::intMbs) / sizeof(MisbehaviorTypes) == misbehaviorTypes::SIZE_OF_ENUM,
                      "sizes dont match");


    }
} //namespace artery


#endif
