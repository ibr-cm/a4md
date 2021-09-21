#!/bin/bash
clear
export SUMO_HOME=/home/joseph/Projects/F2MD/Programs/sumo-1.0.1/
/home/joseph/Projects/F2MD/Programs/sumo-1.0.1/tools/route/cutRoutes.py lust.net.xml due.local.actuated.2.rou.xml --routes-output due-filt.local.actuated.2.rou.xml --orig-net lust-orig.net.xml
