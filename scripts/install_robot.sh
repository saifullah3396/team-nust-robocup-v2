#!/bin/sh

FILES=""
BUILD=""
ROBOTS="Nu-11/Nu-12/Nu-13/Nu-14/Nu-15"
ROBOT=""
IP_PREFIX="192.168.30"
WLAN_PREFIX="10.0.30"

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
   [ "$ROBOT" != "Nu-15" ]; then
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
fi

ROBOT_DIR=$PATH_TO_TEAM_NUST_DIR/config/Robots/Nu-1$ROBOT_NUM
CROSS_DEPENDS_DIR=$PATH_TO_TEAM_NUST_DIR/cross-depends
BUILD_DIR=$PATH_TO_TEAM_NUST_DIR/build/$BUILD/$TOOLCHAIN/lib
if [ "$FILES" = "ALL" ]; then
  rsync -r ./autoload.tnust nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/bin -v
  rsync -r $ROBOT_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/config -v
  rsync -r $CROSS_DEPENDS_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/depends -v
  rsync -r $BUILD_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/depends/lib -v
elif [ "$FILES" = "DEPENDS" ]; then
  rsync -r $CROSS_DEPENDS_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/depends -v
  rsync -r $PATH_TO_TEAM_NUST_DIR/bkernel-modules/mt9m114.ko 
elif [ "$FILES" = "CONFIG" ]; then
  rsync -r $ROBOT_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/config -v
  rsync -r $ROBOT_DIR/../../BehaviorConfigs nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/config -v
  rsync -r $ROBOT_DIR/../../Common nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/config -v
elif [ "$FILES" = "LIBS" ]; then
  rsync -r $BUILD_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/depends/lib -v
elif [ "$FILES" = "LIB_UTILS" ]; then
  rsync -r $BUILD_DIR/libtnrs-utils.so nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/depends/lib -v
elif [ "$FILES" = "LIB_MOTION" ]; then
rsync -r $BUILD_DIR/libtnrs-motion-module.so nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/depends/lib -v
elif [ "$FILES" = "LIB_BASE" ]; then
rsync -r $BUILD_DIR/libtnrs-base.so nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/depends/lib -v
elif [ "$FILES" = "LIB_VISION" ]; then
  rsync -r $BUILD_DIR/libtnrs-vision-module.so nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/depends/lib -v
elif [ "$FILES" = "LIB_USER_COMM" ]; then
  rsync -r $BUILD_DIR/libtnrs-user-comm-module.so nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/depends/lib -v
elif [ "$FILES" = "LIB_TNRS" ]; then
  rsync -r $BUILD_DIR/libtnrs-module.so nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/depends/lib -v
fi
