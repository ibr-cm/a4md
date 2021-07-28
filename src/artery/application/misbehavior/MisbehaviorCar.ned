package artery.application.misbehavior;

import artery.envmod.Car;
import artery.networking.Vanetza;

module MisbehaviorCar extends artery.envmod.Car
{
    parameters:
        vanetza[*].dcc.typename = "NoRateControlDccEntity";
        double misbehaviorUpdateInterval = default(0.01s) @unit(s);
        //middleware.updateInterval = 0.1s;
        middleware.updateInterval = misbehaviorUpdateInterval;
}
