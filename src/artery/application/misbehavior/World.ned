package artery.application.misbehavior;

import artery.envmod.World;
import artery.application.misbehavior.authority.MisbehaviorAuthority;

network World extends artery.envmod.World
{

    submodules:
        misbehaviorAuthority: MisbehaviorAuthority {
        }

}