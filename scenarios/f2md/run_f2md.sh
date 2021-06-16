#!/bin/bash
OPP_RUN=/opt/omnetpp-5.6.2/bin/opp_run_dbg
NED_FOLDERS="/home/bastian/Documents/f2md-artery/artery-f2md/src/artery:/home/bastian/Documents/f2md-artery/artery-f2md/src/traci:/home/bastian/Documents/f2md-artery/artery-f2md/extern/veins/examples/veins:/home/bastian/Documents/f2md-artery/artery-f2md/extern/veins/src/veins:/home/bastian/Documents/f2md-artery/artery-f2md/extern/inet/src:/home/bastian/Documents/f2md-artery/artery-f2md/extern/inet/examples:/home/bastian/Documents/f2md-artery/artery-f2md/extern/inet/tutorials:/home/bastian/Documents/f2md-artery/artery-f2md/extern/inet/showcases"
LIBRARIES="-l/home/bastian/Documents/f2md-artery/artery-f2md/build/src/artery/envmod/libartery_envmod.so -l/home/bastian/Documents/f2md-artery/artery-f2md/build/src/artery/libartery_core.so -l/home/bastian/Documents/f2md-artery/artery-f2md/build/src/traci/libtraci.so -l/home/bastian/Documents/f2md-artery/artery-f2md/build/extern/libveins.so -l/home/bastian/Documents/f2md-artery/artery-f2md/build/extern/libINET.so -l/home/bastian/Documents/f2md-artery/artery-f2md/build/src/artery/storyboard/libartery_storyboard.so"

$OPP_RUN -n $NED_FOLDERS $LIBRARIES "$@"
