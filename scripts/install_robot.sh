#!/bin/bash

baseDir="$(cd "$(dirname "$(which "$0")")" && pwd)"
bhDir="$(dirname "${baseDir}")"
includeDir="${baseDir}/Include/"

source "${includeDir}/bhumanBase"

FILES=""
BUILD=""
ROBOTS="Nu-11/Nu-12/Nu-13/Nu-14/Nu-15/Nu-16"
ROBOT=""
IP_PREFIX="192.168.30"
WLAN_PREFIX="10.0.30"
ROBOT_VERSION="V5"

usage()
{
    echo "Usage:"
    echo "./install_robot.sh -b=<BUILD> -t=<TOOLCHAIN> -r=<ROBOT> -f=<FILES> -w"
    echo ""
    echo " -h | --help : Displays the help"
    echo " -b | --build : Build type (Release/Debug/Release-Motion)"
    echo " -t | --tool-chain : Toolchain name used in code compilation (" $TOOLCHAIN ")"
    echo " -r | --robot : Name of the robot for which calibration is needed (" $ROBOTS ")"
    echo " -f | --files: Files to be copied on to the robot (ALL/DEPENDS/CONFIG/LIBS)"
    echo " -w | --robot : If the robot is connected through the lan network"
    echo " -rv | --robot-version : V5 or V6 robot"
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
        -t  | --tool-chain )
            TOOLCHAIN=$VALUE
            ;;
        -r | --robot)
            ROBOT=$VALUE
            ;;
        -b  | --tool-chain )
            BUILD=$VALUE
            ;;
        -f  | --files )
            FILES=$VALUE
            ;;
        -rv | --robot-version )
            ROBOT_VERSION=$VALUE
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
   [ "$ROBOT" != "Nu-16" ]; then
    echo "Invalid robot name: $ROBOT . Options are: ( $ROBOTS )."
    exit 1;
fi

if [ "$BUILD" = "" ]; then
  echo "Please provide the build type to continue."
  exit 1
fi

if [ "$FILES" = "" ]; then
  FILES="ALL"
fi

if [ "$TOOLCHAIN" = "" ]; then
  TOOLCHAIN="cross"
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
fi

rsyncOptions="${rsyncOptions} --links -v -r"
echo ${rsyncOptions} 
echo ${sshCommand}

DEPENDS_TARGET=/home/nao/depends
BIN_TARGET=/home/nao/depends/bin
LIB_TARGET=/home/nao/depends/lib
TARGET_CONFIG_FOLDER=/home/nao/config
ROBOT_DIR=$PATH_TO_TEAM_NUST_DIR/config/Robots/Nu-1$ROBOT_NUM
CROSS_DEPENDS_DIR=$PATH_TO_TEAM_NUST_DIR/cross-depends
BUILD_DIR=$PATH_TO_TEAM_NUST_DIR/build-$ROBOT_VERSION/$BUILD/$TOOLCHAIN/lib
echo $BUILD_DIR
if [ "$FILES" = "ALL" ]; then
  # Copy naoqi config over to the robot
  rsync ${rsyncOptions} -e "${sshCommand}" ./Files/autoload.tnust nao@$IP_PREFIX.$ROBOT_NUM:${BIN_TARGET} || fatal "Can't copy to '${BIN_TARGET}' on NAO"
  # Copy configuration files to the robot 
  rsync ${rsyncOptions} -e "${sshCommand}" $ROBOT_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:${TARGET_CONFIG_FOLDER} || fatal "Can't copy to '${TARGET_CONFIG_FOLDER}' on NAO"
  rsync ${rsyncOptions} -e "${sshCommand}" $ROBOT_DIR/../../BehaviorConfigs nao@$IP_PREFIX.$ROBOT_NUM:${TARGET_CONFIG_FOLDER} || fatal "Can't copy to '${TARGET_CONFIG_FOLDER}' on NAO"
  rsync ${rsyncOptions} -e "${sshCommand}" $ROBOT_DIR/../../Common/* nao@$IP_PREFIX.$ROBOT_NUM:${TARGET_CONFIG_FOLDER} || fatal "Can't copy to '${TARGET_CONFIG_FOLDER}' on NAO"
  rsync ${rsyncOptions} -e "${sshCommand}" $ROBOT_DIR/../../Keys nao@$IP_PREFIX.$ROBOT_NUM:${TARGET_CONFIG_FOLDER} || fatal "Can't copy to '${TARGET_CONFIG_FOLDER}' on NAO"
  if [ "$ROBOT_VERSION" = "V5" ]; then
  # Copy camera driver if it is V5 robot
  rsync ${rsyncOptions} -e "${sshCommand}" $PATH_TO_TEAM_NUST_DIR/bkernel-modules/mt9m114.ko
  fi
  # Copy cross-compiled libraries to the robot
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
elif [ "$FILES" = "DEPENDS" ]; then
  # Copy cross-compiled dependencies to the robot
  rsync ${rsyncOptions} -e "${sshCommand}" $CROSS_DEPENDS_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:${DEPENDS_TARGET} || fatal "Can't copy to '${DEPENDS_TARGET}' on NAO"
  if [ "$ROBOT_VERSION" = "V5" ]; then
    # Copy camera driver if it is V5 robot
    rsync ${rsyncOptions} -e "${sshCommand}" $PATH_TO_TEAM_NUST_DIR/bkernel-modules/mt9m114.ko
  fi
elif [ "$FILES" = "CONFIG" ]; then
  # Copy configuration files to the robot 
  rsync ${rsyncOptions} -e "${sshCommand}" $ROBOT_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:${TARGET_CONFIG_FOLDER} || fatal "Can't copy to '${TARGET_CONFIG_FOLDER}' on NAO"
  rsync ${rsyncOptions} -e "${sshCommand}" $ROBOT_DIR/../../BehaviorConfigs nao@$IP_PREFIX.$ROBOT_NUM:${TARGET_CONFIG_FOLDER} || fatal "Can't copy to '${TARGET_CONFIG_FOLDER}' on NAO"
  rsync ${rsyncOptions} -e "${sshCommand}" $ROBOT_DIR/../../Common/* nao@$IP_PREFIX.$ROBOT_NUM:${TARGET_CONFIG_FOLDER} || fatal "Can't copy to '${TARGET_CONFIG_FOLDER}' on NAO"
elif [ "$FILES" = "BIN" ]; then
  if [ "$ROBOT_VERSION" = "V6" ]; then
    # Copy camera driver if it is V5 robot
    rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/../bin/* nao@$IP_PREFIX.$ROBOT_NUM:${BIN_TARGET} || fatal "Can't copy to '${BIN_TARGET}' on NAO"
  fi
elif [ "$FILES" = "LIBS" ]; then
  # Copy cross-compiled libraries to the robot
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
  if [ "$ROBOT_VERSION" = "V6" ]; then
    # Copy camera driver if it is V5 robot
    rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/../bin/* nao@$IP_PREFIX.$ROBOT_NUM:${BIN_TARGET} || fatal "Can't copy to '${BIN_TARGET}' on NAO"
  fi
elif [ "$FILES" = "LIB_RESOURCES" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libfftw* nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libnlopt* nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libjsoncpp* nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libqpOASES* nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
elif [ "$FILES" = "LIB_UTILS" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libtnrs-utils.so nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
elif [ "$FILES" = "LIB_BASE" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libtnrs-base.so nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
elif [ "$FILES" = "LIB_BM" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libtnrs-behavior-manager.so nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
elif [ "$FILES" = "LIB_GAME_COMM" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libtnrs-game-comm-module.so nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
elif [ "$FILES" = "LIB_USER_COMM" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libtnrs-user-comm-module.so nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
elif [ "$FILES" = "LIB_CONTROL" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libtnrs-control-module.so nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
elif [ "$FILES" = "LIB_GENERAL" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libtnrs-gb-module.so nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
elif [ "$FILES" = "LIB_MOTION" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libtnrs-motion-module.so nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
elif [ "$FILES" = "LIB_PLANNING" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libtnrs-planning-module.so nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
elif [ "$FILES" = "LIB_VISION" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libtnrs-vision-module.so nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
elif [ "$FILES" = "LIB_LOCALIZATION" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libtnrs-localization-module.so nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
elif [ "$FILES" = "LIB_TNRS" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/libtnrs-module.so nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
fi
  
