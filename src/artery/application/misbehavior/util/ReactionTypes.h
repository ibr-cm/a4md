//
// Created by bastian on 28.07.21.
//

#ifndef ARTERY_REACTIONTYPES_H
#define ARTERY_REACTIONTYPES_H

namespace artery {
    namespace reactionTypes {
        enum ReactionTypes {
            Nothing = 0,
            Warning = 1,
            Ticket = 2,
            PassiveRevocation = 3,
            ActiveRevocation = 4
        };
    }

} // namespace artery

#endif //ARTERY_REACTIONTYPES_H
