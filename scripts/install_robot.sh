#!/bin/bash

baseDir="$(cd "$(dirname "$(which "$0")")" && pwd)"
tnDir="$(dirname "${baseDir}")"
includeDir="${baseDir}/Include/"

source "${includeDir}/bhumanBase"

FILES=""
BUILD=""
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
    echo " -r | --robot : Name of the robot for which calibration is needed (" Nu-<ROBOT_NUMBER> ")"
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

if [ "$BUILD" = "" ]; then
  echo "Please provide the build type to continue."
  usage
  exit 1
fi

if [ "$FILES" = "" ]; then
  FILES="ALL"
fi

if [ "$TOOLCHAIN" = "" ]; then
  TOOLCHAIN="cross"
fi

PREFIX="${ROBOT:0:3}"
NUMBER_LEN=`expr 3 + ${#ROBOT} - 3`
ROBOT_NUM="${ROBOT:3:${NUMBER_LEN}}"
if [ "$PREFIX" != "Nu-" ]; then
  echo "Invalid robot name: $ROBOT . Options are: ( Nu-<RobotNumber> )."
  usage
  exit 1;
fi

if [ "$ROBOT_NUM" = "" ]; then
  echo "Invalid robot name: $ROBOT . Options are: ( Nu-<RobotNumber> )."
  usage
  exit 1;
fi

rsyncOptions="${rsyncOptions} --links -v -r"
echo ${rsyncOptions} 
echo ${sshCommand}

NAOQI_PREF_TARGET=/home/nao/team_nust/pref
VIDEO_DRIVER_TARGET=/lib/modules/2.6.33.9-rt31-aldebaran-rt/kernel/drivers/media/video
DEPENDS_TARGET=/home/nao/team_nust/depends
BIN_TARGET=/home/nao/team_nust/bin
FILES_TARGET=/home/nao/team_nust/files
LIB_TARGET=/home/nao/team_nust/lib
CONFIG_TARGET=/home/nao/team_nust/config
LOGS_TARGET=/home/nao/team_nust/logs
ssh -i "${privateKey}" ${sshOptions} nao@$IP_PREFIX.$ROBOT_NUM "mkdir -p ${DEPENDS_TARGET}" || fatal "Can't create '${DEPENDS_TARGET}' on NAO"
ssh -i "${privateKey}" ${sshOptions} nao@$IP_PREFIX.$ROBOT_NUM "mkdir -p ${BIN_TARGET}" || fatal "Can't create '${BIN_TARGET}' on NAO"
ssh -i "${privateKey}" ${sshOptions} nao@$IP_PREFIX.$ROBOT_NUM "mkdir -p ${LIB_TARGET}" || fatal "Can't create '${LIB_TARGET}' on NAO"
ssh -i "${privateKey}" ${sshOptions} nao@$IP_PREFIX.$ROBOT_NUM "mkdir -p ${CONFIG_TARGET}" || fatal "Can't create '${CONFIG_TARGET}' on NAO"
ssh -i "${privateKey}" ${sshOptions} nao@$IP_PREFIX.$ROBOT_NUM "mkdir -p ${LOGS_TARGET}" || fatal "Can't create '${LOGS_TARGET}' on NAO"

ROBOT_DIR=$PATH_TO_TEAM_NUST_DIR/config/Robots/Nu-1$ROBOT_NUM
CROSS_DEPENDS_DIR=$PATH_TO_TEAM_NUST_DIR/cross-depends
BUILD_DIR=$PATH_TO_TEAM_NUST_DIR/build-$ROBOT_VERSION/$BUILD/$TOOLCHAIN/lib
echo $BUILD_DIR
if [ "$FILES" = "ALL" ]; then
  # Copy naoqi config over to the robot
  rsync ${rsyncOptions} -e "${sshCommand}" ./Files/* nao@$IP_PREFIX.$ROBOT_NUM:${FILES_TARGET} || fatal "Can't copy to '${FILES_TARGET}' on NAO"
  # Copy configuration files to the robot 
  rsync ${rsyncOptions} -e "${sshCommand}" $ROBOT_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:${CONFIG_TARGET} || fatal "Can't copy to '${CONFIG_TARGET}' on NAO"
  rsync ${rsyncOptions} -e "${sshCommand}" $ROBOT_DIR/../../BehaviorConfigs nao@$IP_PREFIX.$ROBOT_NUM:${CONFIG_TARGET} || fatal "Can't copy to '${CONFIG_TARGET}' on NAO"
  rsync ${rsyncOptions} -e "${sshCommand}" $ROBOT_DIR/../../Common/* nao@$IP_PREFIX.$ROBOT_NUM:${CONFIG_TARGET} || fatal "Can't copy to '${CONFIG_TARGET}' on NAO"
  rsync ${rsyncOptions} -e "${sshCommand}" $ROBOT_DIR/../../Keys nao@$IP_PREFIX.$ROBOT_NUM:${CONFIG_TARGET} || fatal "Can't copy to '${CONFIG_TARGET}' on NAO"
  if [ "$ROBOT_VERSION" = "V5" ]; then
  # Copy camera driver if it is V5 robot
  rsync ${rsyncOptions} -e "${sshCommand}" $PATH_TO_TEAM_NUST_DIR/bkernel-modules/mt9m114.ko nao@$IP_PREFIX.$ROBOT_NUM:${DEPENDS_TARGET}
  # ssh -i "${privateKey}" ${sshOptions} nao@$IP_PREFIX.$ROBOT_NUM "cd ${DEPENDS_TARGET}; cp mt9m114.ko ${VIDEO_DRIVER_TARGET}" || fatal "Can't copy to '${VIDEO_DRIVER_TARGET}' on NAO"
  fi
  # Copy cross-compiled libraries to the robot
  rsync ${rsyncOptions} -e "${sshCommand}" $BUILD_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:${LIB_TARGET} || fatal "Can't copy to '${LIB_TARGET}' on NAO"
elif [ "$FILES" = "DEPENDS" ]; then
  # Copy cross-compiled dependencies to the robot
  rsync ${rsyncOptions} -e "${sshCommand}" $CROSS_DEPENDS_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:${DEPENDS_TARGET} || fatal "Can't copy to '${DEPENDS_TARGET}' on NAO"
  rsync ${rsyncOptions} -e "${sshCommand}" $PATH_TO_TEAM_NUST_DIR/src/GameController/lib/libgamectrl.so nao@$IP_PREFIX.$ROBOT_NUM:${DEPENDS_TARGET} || fatal "Can't copy to '${DEPENDS_TARGET}' on NAO"
  if [ "$ROBOT_VERSION" = "V5" ]; then
    # Copy camera driver if it is V5 robot
    rsync ${rsyncOptions} -e "${sshCommand}" $PATH_TO_TEAM_NUST_DIR/bkernel-modules/mt9m114.ko
  fi
elif [ "$FILES" = "CONFIG" ]; then
  # Copy configuration files to the robot 
  rsync ${rsyncOptions} -e "${sshCommand}" $ROBOT_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:${CONFIG_TARGET} || fatal "Can't copy to '${CONFIG_TARGET}' on NAO"
  rsync ${rsyncOptions} -e "${sshCommand}" $ROBOT_DIR/../../BehaviorConfigs nao@$IP_PREFIX.$ROBOT_NUM:${CONFIG_TARGET} || fatal "Can't copy to '${CONFIG_TARGET}' on NAO"
  rsync ${rsyncOptions} -e "${sshCommand}" $ROBOT_DIR/../../Common/* nao@$IP_PREFIX.$ROBOT_NUM:${CONFIG_TARGET} || fatal "Can't copy to '${CONFIG_TARGET}' on NAO"
elif [ "$FILES" = "NAOQI_PREF" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" ./naoqi_preferences/* nao@$IP_PREFIX.$ROBOT_NUM:${NAOQI_PREF_TARGET} || fatal "Can't copy to '${NAOQI_PREF_TARGET}' on NAO"
elif [ "$FILES" = "BIN" ]; then
  rsync ${rsyncOptions} -e "${sshCommand}" ./bin/* nao@$IP_PREFIX.$ROBOT_NUM:${BIN_TARGET} || fatal "Can't copy to '${BIN_TARGET}' on NAO"
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
  
