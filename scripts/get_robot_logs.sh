#!/bin/bash

baseDir="$(cd "$(dirname "$(which "$0")")" && pwd)"
bhDir="$(dirname "${baseDir}")"
includeDir="${baseDir}/Include/"

source "${includeDir}/bhumanBase"

FILES=""
BUILD=""
ROBOTS="Nu-11/Nu-12/Nu-13/Nu-14/Nu-15/Nu-16/Nu-17"
ROBOT=""
IP_PREFIX="192.168.30"
WLAN_PREFIX="10.0.30"

usage()
{
    echo "Usage:"
    echo "./get_robot_logs.sh -r=<ROBOT>"
    echo ""
    echo " -h | --help : Displays the help"
    echo " -r | --robot : Name of the robot for which calibration is needed (" $ROBOTS ")"
    echo " -w | --wlan : If the robot is connected through the lan network"
    echo ""
}

while [ "$1" != "" ]; do
    PARAM=`echo $1 | awk -F= '{print $1}'`
    VALUE=`echo $1 | awk -F= '{print $2}'`
    case $PARAM in
        -h | --help)
            usage
            exit
            ;;
        -w  | --wlan )
            IP_PREFIX=$WLAN_PREFIX
            ;;
        -r | --robot)
            ROBOT=$VALUE
            ;;
        *)
            echo "ERROR: unknown parameter \"$PARAM\""
            usage
            exit 1
            ;;
    esac
    shift
done

if [ "$ROBOT" != "Nu-11" ] && 
   [ "$ROBOT" != "Nu-12" ] && 
   [ "$ROBOT" != "Nu-13" ] && 
   [ "$ROBOT" != "Nu-14" ] && 
   [ "$ROBOT" != "Nu-15" ] &&
   [ "$ROBOT" != "Nu-16" ] &&
   [ "$ROBOT" != "Nu-17" ]; then
    echo "Invalid robot name: $ROBOT . Options are: ( $ROBOTS )."
    exit 1;
fi

ROBOT_NUM=""
if [ "$ROBOT" = "Nu-11" ]; then
  ROBOT_NUM=1
elif [ "$ROBOT" = "Nu-12" ]; then
  ROBOT_NUM=2
elif [ "$ROBOT" = "Nu-13" ]; then
  ROBOT_NUM=3
elif [ "$ROBOT" = "Nu-14" ]; then
  ROBOT_NUM=4
elif [ "$ROBOT" = "Nu-15" ]; then
  ROBOT_NUM=5
elif [ "$ROBOT" = "Nu-16" ]; then
  ROBOT_NUM=6
elif [ "$ROBOT" = "Nu-17" ]; then
  ROBOT_NUM=7
fi

rsyncOptions="${rsyncOptions} --links -v -r"
echo ${rsyncOptions} 
echo ${sshCommand}

LOG_DIR_PATH=$PATH_TO_TEAM_NUST_DIR/logs/Robots/Nu-1$ROBOT_NUM
rsync ${rsyncOptions} -e "${sshCommand}" nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/team_nust/logs/* $LOG_DIR_PATH
