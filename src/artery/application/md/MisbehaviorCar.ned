package artery.application.md;

import artery.envmod.Car;
import artery.networking.Vanetza;

module MisbehaviorCar extends artery.envmod.Car
{
    parameters:
        vanetza[*].dcc.typename = "NoRateControlDccEntity";
        middleware.updateInterval = 0.01s;
}
