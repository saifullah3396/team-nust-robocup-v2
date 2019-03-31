#!/bin/bash

trap killgroup SIGINT

killgroup() {
  echo killing...
  kill 0
}

killall vrep
$PATH_TO_VREP_DIR/vrep.sh ../scenes/KickSimulationIncomingBall.ttt
