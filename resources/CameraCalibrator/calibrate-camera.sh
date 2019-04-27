#!/bin/sh

TOOLCHAIN=""
ROBOTS="Nu-11/Nu-12/Nu-13/Nu-14/Nu-15/Nu-16/Nu-17/Nu-18"
ROBOT=""
CAM="Top"
REFRESH="n"
ROBOT_VERSION=V5

usage()
{
    echo "Usage:"
    echo "./calibrate-camera.sh -t=<TOOLCHAIN> -r=<ROBOT> -c=<CAMERA> -rr=<y/n>"
    echo ""
    echo " -h | --help : Displays the help"
    echo " -t | --tool-chain : Toolchain name used in code compilation (" $TOOLCHAIN ")"
    echo " -r | --robot : Name of the robot for which calibration is needed (" $ROBOTS ")"
    echo " -c | --camera : Top or Bottom camera. Defaults to 'Top'"
    echo " -rv | --robot-version : Robot version."
    echo " -rr | --refresh : Moves image data available in robot directory to another directory and asks the user to get new images."
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
        -r | --robot)
            ROBOT=$VALUE
            ;;
        -rv | --robot-version)
            ROBOT_VERSION=$VALUE
            ;;
        -t  | --tool-chain )
            TOOLCHAIN=$VALUE
            ;;
        -c  | --camera )
            CAM=$VALUE
            ;;
        -rr  | --refresh )
            REFRESH=$VALUE
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
   [ "$ROBOT" != "Nu-17" ] &&
   [ "$ROBOT" != "Nu-18" ]; then
    echo "Invalid robot name: $ROBOT . Options are: ( $ROBOTS )."
    exit 1;
fi

if [ "$TOOLCHAIN" = "" ]; then
  echo "Please privide the toolchain name to continue."
  exit 1
fi

cd $PATH_TO_TEAM_NUST_DIR/logs/Robots/$ROBOT/Images/$CAM

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
elif [ "$ROBOT" = "Nu-18" ]; then
  ROBOT_NUM=8
fi

if [ "$REFRESH" = "y" ]; then
  if [ -d "." ]; then  
    echo
  else
    mkdir backup/
  fi
  if [ "$(ls -A . | grep .jpg)" ]; then
    mv *.jpg backup/
  fi
  $PATH_TO_TEAM_NUST_DIR/build-V5/Release-Motion/remote/bin/team-nust-spl --save-images $CAM --robot $ROBOT --pip "192.168.30.""$ROBOT_NUM";
fi

if [ "$(ls -A . | grep .jpg)" ]; then
  ls -d "$PWD"/*.jpg > ../../"$CAM"Images.txt 
else

  echo "No image data found. Please save '.jpg' image files in  the directory $PATH_TO_TEAM_NUST_DIR/logs/Robots/$ROBOT/ImageLogs/$CAM to continue."
  exit 1
fi

$PATH_TO_TEAM_NUST_DIR/build-$ROBOT_VERSION/Release-Motion/$TOOLCHAIN/bin/tnrs-camera-calibrator --robot $ROBOT --camera $CAM 

