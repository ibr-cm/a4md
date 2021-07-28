package artery.application.misbehavior;

import artery.envmod.World;
import artery.application.misbehavior.ma.MisbehaviorAuthority;

network World extends artery.envmod.World
{

    submodules:
        misbehaviorAuthority: MisbehaviorAuthority {
        }

}