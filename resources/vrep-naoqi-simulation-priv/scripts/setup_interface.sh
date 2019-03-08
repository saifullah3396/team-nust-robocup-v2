#!/bin/bash

trap killgroup SIGINT

killgroup() {
  echo killing...
  kill 0
}

killall vrep-naoqi-simulation
killall hal
killall naoqi-bin
../build/vrep-naoqi-simulation $1
