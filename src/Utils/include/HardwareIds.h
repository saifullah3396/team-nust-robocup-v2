/**
 * @file Utils/include/HardwareIds.h
 *
 * This file declares the enumeration ids for all the sensors and
 * actuators of the robot
 *
 * @author <A href="mailto:saifullah3396@gmailcom">Saifullah</A>
 * @date 1 Jan 2017
 */

#pragma once

#include "Utils/include/EnumUtils.h"

/**
 * Enumeration for available joint sensor types
 *
 * @enum JointSensorTypes
 */
enum class JointSensorTypes : unsigned int
{
  position = 0,
  temp,
  //TEMP_STATUS,
  current,
  hardness,
  count
};

/**
 * Enumeration for available sensor types
 *
 * @enum SensorTypes
 */
enum class SensorTypes : unsigned int
{
  joints = 0,
  touchSensors = JointSensorTypes::count,
  switchSensors,
  batterySensors,
  inertialSensors,
  sonarSensors,
  fsrSensors,
  ledSensors,
  count
};

/**
 * Enumeration for available joint actuator types
 *
 * @enum JointActuatorTypes
 */
enum class JointActuatorTypes : unsigned int
{
  angles = 0,
  hardness,
  count
};

/**
 * Enumeration for available actuator types
 *
 * @enum ActuatorTypes
 */

enum class ActuatorTypes : unsigned int
{
  jointActuators = 0,
  ledActuators = JointActuatorTypes::count,
  count
};

/**
 * Enumeration for available sensors
 *
 * @enum HardwareIds
 */

enum class Joints : unsigned int
{
  //!Joints
  headYaw = 0,
  headPitch,
  lShoulderPitch,
  lShoulderRoll,
  lElbowYaw,
  lElbowRoll,
  lWristYaw,
  //L_HandPOSITION,
  rShoulderPitch,
  rShoulderRoll,
  rElbowYaw,
  rElbowRoll,
  rWristYaw,
  //rHandPOSITION,
  lHipYawPitch,
  lHipRoll,
  lHipPitch,
  lKneePitch,
  lAnklePitch,
  lAnkleRoll,
  rHipYawPitch,
  rHipRoll,
  rHipPitch,
  rKneePitch,
  rAnklePitch,
  rAnkleRoll,
  count,
  first = headYaw,
  last = rAnkleRoll
};
DECLARE_SPECIALIZED_ENUM(Joints)

enum class HardwareIds : unsigned int
{
  headStart = Joints::headYaw,
  nHead = toUType(Joints::headPitch) - headStart + 1,
  lArmStart = Joints::lShoulderPitch,
  nLArm = toUType(Joints::lWristYaw) - lArmStart + 1,
  rArmStart = Joints::rShoulderPitch,
  nRArm = toUType(Joints::rWristYaw) - rArmStart + 1,
  lLegStart = Joints::lHipYawPitch,
  nLLeg = toUType(Joints::lAnkleRoll) - lLegStart + 1,
  rLegStart = Joints::rHipYawPitch,
  nRLeg = toUType(Joints::rAnkleRoll) - rLegStart + 1,
};

enum class Links : unsigned int {
  //! Links
  headYaw = 0,
  headPitch,
  lShoulderPitch,
  lShoulderRoll,
  lElbowYaw,
  lElbowRoll,
  lWristYaw,
  //L_HandPOSITION,
  rShoulderPitch,
  rShoulderRoll,
  rElbowYaw,
  rElbowRoll,
  rWristYaw,
  //rHandPOSITION,
  lHipYawPitch,
  lHipRoll,
  lHipPitch,
  lKneePitch,
  lAnklePitch,
  lAnkleRoll,
  rHipYawPitch,
  rHipRoll,
  rHipPitch,
  rKneePitch,
  rAnklePitch,
  rAnkleRoll,
  torso,
  count,
  first = headYaw,
  last = torso
};
DECLARE_SPECIALIZED_ENUM(Links)

enum class TouchSensors : unsigned int {
  //!Touch Sensors
  headTouchFront = 0,
  headTouchRear,
  headTouchMiddle,
  lHandTouchBack,
  lHandTouchLeft,
  lHandTouchRight,
  rHandTouchBack,
  rHandTouchLeft,
  rHandTouchRight,
  count,
  first = headTouchFront,
  last = rHandTouchRight
};
DECLARE_SPECIALIZED_ENUM(TouchSensors)

enum class SwitchSensors : unsigned int {
  //!Switch Sensors
  chestBoardButton = 0,
  lFootBumperRight,
  lFootBumperLeft,
  rFootBumperRight,
  rFootBumperLeft,
  count,
  first = chestBoardButton,
  last = rFootBumperLeft
};
DECLARE_SPECIALIZED_ENUM(SwitchSensors)

enum class BatterySensors : unsigned int {
  ///!Battery Sensors
  headCpuTemperature = 0,
  batteryCurrent,
  batteryCharge,
  batteryTemperature,
  count,
  first = headCpuTemperature,
  last = batteryTemperature
};
DECLARE_SPECIALIZED_ENUM(BatterySensors)

enum class InertialSensors : unsigned int {
  //!Inertial Sensors
  gyroscopeX = 0,
  gyroscopeY,
  gyroscopeZ,
  torsoAngleX,
  torsoAngleY,
  torsoAngleZ,
  accelerometerX,
  accelerometerY,
  accelerometerZ,
  count,
  first = gyroscopeX,
  last = accelerometerZ
};
DECLARE_SPECIALIZED_ENUM(InertialSensors)

enum class SonarSensors : unsigned int {
  //!Sonar Sensors
  lUsSonar = 0,
  lUsSonar1,
  lUsSonar2,
  lUsSonar3,
  lUsSonar4,
  lUsSonar5,
  lUsSonar6,
  lUsSonar7,
  lUsSonar8,
  lUsSonar9,
  rUsSonar,
  rUsSonar1,
  rUsSonar2,
  rUsSonar3,
  rUsSonar4,
  rUsSonar5,
  rUsSonar6,
  rUsSonar7,
  rUsSonar8,
  rUsSonar9,
  count,
  first = lUsSonar,
  last = rUsSonar9
};
DECLARE_SPECIALIZED_ENUM(SonarSensors)

enum class FsrSensors : unsigned int {
  //!Fsr Sensors
  lFootFsrFL = 0,
  lFootFsrFR,
  lFootFsrRL,
  lFootFsrRR,
  lFootTotalWeight,
  lFootCopX,
  lFootCopY,
  rFootFsrFL,
  rFootFsrFR,
  rFootFsrRL,
  rFootFsrRR,
  rFootTotalWeight,
  rFootCopX,
  rFootCopY,
  count,
  first = lFootFsrFL,
  last = rFootCopY
};
DECLARE_SPECIALIZED_ENUM(FsrSensors)

enum class LedActuators : unsigned int {
  //!Led Actuators
  faceLedRedLeft_0_degActuator = 0,
  faceLedRedLeft_45_degActuator,
  faceLedRedLeft_90_degActuator,
  faceLedRedLeft_135_degActuator,
  faceLedRedLeft_180_degActuator,
  faceLedRedLeft_225_degActuator,
  faceLedRedLeft_270_degActuator,
  faceLedRedLeft_315_degActuator,
  faceLedGreenLeft_0_degActuator,
  faceLedGreenLeft_45_degActuator,
  faceLedGreenLeft_90_degActuator,
  faceLedGreenLeft_135_degActuator,
  faceLedGreenLeft_180_degActuator,
  faceLedGreenLeft_225_degActuator,
  faceLedGreenLeft_270_degActuator,
  faceLedGreenLeft_315_degActuator,
  faceLedBlueLeft_0_degActuator,
  faceLedBlueLeft_45_degActuator,
  faceLedBlueLeft_90_degActuator,
  faceLedBlueLeft_135_degActuator,
  faceLedBlueLeft_180_degActuator,
  faceLedBlueLeft_225_degActuator,
  faceLedBlueLeft_270_degActuator,
  faceLedBlueLeft_315_degActuator,
  endFaceLLed,
  nFaceLLed = endFaceLLed,
  faceLedRedRight_0_degActuator = endFaceLLed,
  faceLedRedRight_45_degActuator,
  faceLedRedRight_90_degActuator,
  faceLedRedRight_135_degActuator,
  faceLedRedRight_180_degActuator,
  faceLedRedRight_225_degActuator,
  faceLedRedRight_270_degActuator,
  faceLedRedRight_315_degActuator,
  faceLedGreenRight_0_degActuator,
  faceLedGreenRight_45_degActuator,
  faceLedGreenRight_90_degActuator,
  faceLedGreenRight_135_degActuator,
  faceLedGreenRight_180_degActuator,
  faceLedGreenRight_225_degActuator,
  faceLedGreenRight_270_degActuator,
  faceLedGreenRight_315_degActuator,
  faceLedBlueRight_0_degActuator,
  faceLedBlueRight_45_degActuator,
  faceLedBlueRight_90_degActuator,
  faceLedBlueRight_135_degActuator,
  faceLedBlueRight_180_degActuator,
  faceLedBlueRight_225_degActuator,
  faceLedBlueRight_270_degActuator,
  faceLedBlueRight_315_degActuator,
  endFaceRLed,
  nFaceRLed = endFaceRLed - endFaceLLed,
  earsLedLeft_0_degActuator = endFaceRLed,
  earsLedLeft_36_degActuator,
  earsLedLeft_72_degActuator,
  earsLedLeft_108_degActuator,
  earsLedLeft_144_degActuator,
  earsLedLeft_180_degActuator,
  earsLedLeft_216_degActuator,
  earsLedLeft_252_degActuator,
  earsLedLeft_288_degActuator,
  earsLedLeft_324_degActuator,
  endEarLLed,
  nEarLLed = endEarLLed - endFaceRLed,
  earsLedRight_0_degActuator = endEarLLed,
  earsLedRight_36_degActuator,
  earsLedRight_72_degActuator,
  earsLedRight_108_degActuator,
  earsLedRight_144_degActuator,
  earsLedRight_180_degActuator,
  earsLedRight_216_degActuator,
  earsLedRight_252_degActuator,
  earsLedRight_288_degActuator,
  earsLedRight_324_degActuator,
  endEarRLed,
  nEarRLed = endEarRLed - endEarLLed,
  chestBoardLedRedActuator = endEarRLed,
  chestBoardLedGreenActuator,
  chestBoardLedBlueActuator,
  endChestLed,
  nChestLed = endChestLed - endEarRLed,
  headLedRearLeft_0_Actuator = endChestLed,
  headLedRearLeft_1_Actuator,
  headLedRearLeft_2_Actuator,
  endHeadRearLeftLed,
  nHeadRearLeftLed = endHeadRearLeftLed - endChestLed,
  headLedRearRight_0_Actuator = endHeadRearLeftLed,
  headLedRearRight_1_Actuator,
  headLedRearRight_2_Actuator,
  endHeadRearRightLed,
  nHeadRearRightLed = endHeadRearRightLed - endHeadRearLeftLed,
  headLedFrontRight_0_Actuator = endHeadRearRightLed,
  headLedFrontRight_1_Actuator,
  endHeadFrontRightLed,
  nHeadFrontRightLed = endHeadFrontRightLed - endHeadRearRightLed,
  headLedFrontLeft_0_Actuator = endHeadFrontRightLed,
  headLedFrontLeft_1_Actuator,
  endHeadFrontLeftLed,
  nHeadFrontLeftLed = endHeadFrontLeftLed - endHeadFrontRightLed,
  headLedMiddleRight_0_Actuator = endHeadFrontLeftLed,
  headLedMiddleLeft_0_Actuator,
  endHeadMiddleLed,
  nHeadMiddleLed = endHeadMiddleLed - endHeadFrontLeftLed,
  nHeadLed = endHeadMiddleLed - endChestLed,
  lFootLedRedActuator = endHeadMiddleLed,
  lFootLedGreenActuator,
  lFootLedBlueActuator,
  endLFeetLed,
  nLFeetLed = endLFeetLed - endHeadMiddleLed,
  rFootLedRedActuator = endLFeetLed,
  rFootLedGreenActuator,
  rFootLedBlueActuator,
  endRFeetLed,
  nRFeetLed = endRFeetLed - endLFeetLed,
  count = endRFeetLed,
  first = faceLedRedLeft_0_degActuator,
  last = rFootLedBlueActuator
};
//DECLARE_SPECIALIZED_ENUM(LedActuators)

/**
 * Enumeration for the robot link chains
 *
 * @enum LinkChains
 */
enum class LinkChains : unsigned int
{
  head = 0,
  lArm,
  rArm,
  lLeg,
  rLeg,
  count,
  first = head,
  last = rLeg
};
DECLARE_SPECIALIZED_ENUM(LinkChains)

enum class LedGroups : unsigned int
{
  head = 0,
  lFace,
  rFace,
  lEar,
  rEar,
  chest,
  lFoot,
  rFoot,
  count,
  first = head,
  last = rFoot
};
DECLARE_SPECIALIZED_ENUM(LedGroups)

/**
 * Enumeration for the robot cameras
 *
 * @enum CameraId
 */
enum class CameraId : unsigned int
{
  headTop = 0,
  headBottom,
  count,
  first = headTop,
  last = headBottom
};
DECLARE_SPECIALIZED_ENUM(CameraId)

/**
 * Enumeration for the hands of the robot
 *
 * @enum RobotHands
 */
enum class RobotHands : unsigned int
{
  lHand = 0,
  rHand,
  count,
  first = lHand,
  last = rHand
};
DECLARE_SPECIALIZED_ENUM(RobotHands)

/**
 * Enumeration for the feet of the robot
 *
 * @enum RobotFeet
 */
enum class RobotFeet : unsigned int
{
  lFoot = LinkChains::lLeg,
  rFoot = LinkChains::rLeg,
  count = 2,
  unknown = count,
  first = lFoot,
  last = rFoot
};
DECLARE_SPECIALIZED_ENUM(RobotFeet)

/**
 * Enumeration for the used robot leg end effectors
 *
 * @enum LegEEs
 */
enum class LegEEs : unsigned int
{
  ankle = 0,
  footBase,
  footCenter,
  kickEE,
  footLeftBound,
  footRightBound,
  count,
  first = ankle,
  last = footRightBound
};
DECLARE_SPECIALIZED_ENUM(LegEEs)
