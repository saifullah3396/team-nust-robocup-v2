/**
 * @file Utils/include/Constants.h
 *
 * This file defines the constants used in the code.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <string>
#include <vector>
#include <math.h>

namespace Constants
{
  const double infinity = 1e12; //! Definition of infinity
  const double gravity = 9.80665; //! Gravitational acceleration

  const double imuX = -0.008; //! Same for V6 and V5
  const double imuY = 0.00606; //! Same for V6 and V5
  const double imuZ = 0.027; //! Same for V6 and V5
  const double shoulderOffsetY = 0.098; //! Same for V6 and V5
  const double elbowOffsetY = 0.015; //! Same for V6 and V5
  const double upperArmLength = 0.105; //! Same for V6 and V5
  const double shoulderOffsetZ = 0.100; //! Same for V6 and V5
  const double lowerArmLength = 0.05595; //! Same for V6 and V5
  const double handOffsetX = 0.05775; //! Same for V6 and V5
  const double handOffsetZ = 0.01231; //! Same for V6 and V5
  const double hipOffsetZ = 0.085; //! Same for V6 and V5
  const double hipOffsetY = 0.050; //! Same for V6 and V5
  const double thighLength = 0.100; //! Same for V6 and V5
  const double tibiaLength = 0.1029; //! Same for V6 and V5
  const double footHeight = 0.04519; //! Same for V6 and V5
  const double footSizeX = 0.16; //! Same for V6 and V5
  const double footSizeY = 0.088; //! Same for V6 and V5
  const double footSizeZ = 0.015; //! Same for V6 and V5
  const double footSeparation = 0.1108; //! Same for V6 and V5
  const double footOriginShiftX = 0.02; //! Same for V6 and V5
  const double footOriginShiftY = 0.0054; //! Same for V6 and V5

  const double neckOffsetZ = 0.1265; //! Same for V6 and V5
  const double cameraBottomX = 0.05071; //! Same for V6 and V5
  const double cameraBottomZ = 0.01774; //! Same for V6 and V5
  const double cameraTopX = 0.05871; //! Same for V6 and V5
  const double cameraTopZ = 0.06364; //! Same for V6 and V5
  const double cameraTopAngleY = 0.020944; //! Same for V6 and V5
  const double cameraBotAngleY = 0.6929; //! Same for V6 and V5

  const double headYawHigh = 2.0857; //! Same for V6 and V5
  const double headYawLow = -2.0857; //! Same for V6 and V5
  const double headPitchHigh = 0.5149; //! Same for V6 and V5
  const double headPitchLow = -0.6720; //! Same for V6 and V5

  const double lShoulderPitchHigh = 2.0857; //! Same for V6 and V5
  const double lShoulderPitchLow = -2.0857; //! Same for V6 and V5
  const double lShoulderRollHigh = 1.3265; //! Same for V6 and V5
  const double lShoulderRollLow = -0.3142; //! Same for V6 and V5
  const double lElbowYawHigh = 2.0875; //! Same for V6 and V5
  const double lElbowYawLow = -2.0875; //! Same for V6 and V5
  const double lElbowRollHigh = -0.0349; //! Same for V6 and V5
  const double lElbowRollLow = -1.5446; //! Same for V6 and V5
  const double lWristYawHigh = 1.8238; //! Same for V6 and V5
  const double lWristYawLow = -1.8238; //! Same for V6 and V5

  const double rShoulderPitchHigh = 2.0857; //! Same for V6 and V5
  const double rShoulderPitchLow = -2.0857; //! Same for V6 and V5
  const double rShoulderRollHigh = 0.3142; //! Same for V6 and V5
  const double rShoulderRollLow = -1.3265; //! Same for V6 and V5
  const double rElbowYawHigh = 2.0875; //! Same for V6 and V5
  const double rElbowYawLow = -2.0875; //! Same for V6 and V5
  const double rElbowRollHigh = 1.5446; //! Same for V6 and V5
  const double rElbowRollLow = 0.0349; //! Same for V6 and V5
  const double rWristYawHigh = 1.8238; //! Same for V6 and V5
  const double rWristYawLow = -1.8238; //! Same for V6 and V5

  const double lHipYawPitchHigh = 0.7408; //! Same for V6 and V5
  const double lHipYawPitchLow = -1.1453; //! Same for V6 and V5
  const double lHipRollHigh = 0.7904; //! Same for V6 and V5
  const double lHipRollLow = -0.3794; //! Same for V6 and V5
  const double lHipPitchHigh = 0.4840; //! Same for V6 and V5
  const double lHipPitchLow = -1.5358; //! Same for V6 and V5
  const double lKneePitchHigh = 2.1125; //! Same for V6 and V5
  const double lKneePitchLow = -0.0923; //! Same for V6 and V5
  const double lAnklePitchHigh = 0.9227; //! Same for V6 and V5
  const double lAnklePitchLow = -1.1895; //! Same for V6 and V5
  const double lAnkleRollHigh = 0.7690; //! Same for V6 and V5
  const double lAnkleRollLow = -0.3978; //! Same for V6 and V5

  const double rHipYawPitchHigh = 0.7408; //! Same for V6 and V5
  const double rHipYawPitchLow = -1.1453; //! Same for V6 and V5
  const double rHipRollHigh = 0.3794; //! Same for V6 and V5
  const double rHipRollLow = -0.7904; //! Same for V6 and V5
  const double rHipPitchHigh = 0.4840; //! Same for V6 and V5
  const double rHipPitchLow = -1.5358; //! Same for V6 and V5
  const double rKneePitchHigh = 2.1201; //! Same for V6 and V5
  const double rKneePitchLow = -0.1030; //! Same for V6 and V5
  const double rAnklePitchHigh = 0.9320; //! Same for V6 and V5
  const double rAnklePitchLow = -1.1864; //! Same for V6 and V5
  const double rAnkleRollHigh = 0.3978; //! Same for V6 and V5
  const double rAnkleRollLow = -0.7690; //! Same for V6 and V5

  const double totalMassH25 = 5.3053;//+0.345

  const double torsoMass = 1.0496; //! Same for V6 and V5
  const double torsoX = -0.00413; //! Same for V6 and V5
  const double torsoY = 0.0; //! Same for V6 and V5
  const double torsoZ = 0.04342; //! Same for V6 and V5

  const double batteryMass = 0.345;
  const double batteryX = -0.030;
  const double batteryY = 0.00;
  const double batteryZ = 0.039;

  const double headYawMass = 0.07842; //! Same for V6 and V5
  const double headYawX = -0.00001; //! Same for V6 and V5
  const double headYawY = 0.0;//0.00014 //! Same for V6 and V5
  const double headYawZ = -0.02742; //! Same for V6 and V5

  #ifndef V6_CROSS_BUILD
  const double headPitchMass = 0.60533;
  const double headPitchX = -0.00112;
  const double headPitchY = 0.0;
  const double headPitchZ = 0.05258;
  #else
  const double headPitchMass = 0.65973;
  const double headPitchX = 0.0010889;
  const double headPitchY = 0.0014628;
  const double headPitchZ = 0.0571855;
  #endif

  #ifndef V6_CROSS_BUILD
  const double rShoulderPitchMass = 0.09304;
  const double rShoulderPitchX = -0.00165;
  const double rShoulderPitchY = 0.02663;
  const double rShoulderPitchZ = 0.00014;
  #else
  const double rShoulderPitchMass = 0.09304;
  const double rShoulderPitchX = -0.00165;
  const double rShoulderPitchY = 0.02663;
  const double rShoulderPitchZ = 0.00014;
  #endif

  const double rShoulderRollMass = 0.15777; //! Same for V6 and V5
  const double rShoulderRollX = 0.02455; //! Same for V6 and V5
  const double rShoulderRollY = -0.00563; //! Same for V6 and V5
  #ifndef V6_CROSS_BUILD
  const double rShoulderRollZ = 0.0032;
  #else
  const double rShoulderRollZ = 0.0033;
  #endif

  const double rElbowYawMass = 0.06483; //! Same for V6 and V5
  const double rElbowYawX = -0.02744; //! Same for V6 and V5
  const double rElbowYawY = 0.0; //! Same for V6 and V5
  const double rElbowYawZ = -0.00014; //! Same for V6 and V5

  const double rElbowRollMass = 0.07761; //! Same for V6 and V5
  const double rElbowRollX = 0.02556; //! Same for V6 and V5
  const double rElbowRollY = -0.00281; //! Same for V6 and V5
  const double rElbowRollZ = 0.00076; //! Same for V6 and V5

  const double rWristYawMass = 0.18533; //! Same for V6 and V5
  const double rWristYawX = 0.03434; //! Same for V6 and V5
  const double rWristYawY = 0.00088; //! Same for V6 and V5
  const double rWristYawZ = 0.00308; //! Same for V6 and V5

  const double rHipYawPitchMass = 0.06981; //! Same for V6 and V5
  const double rHipYawPitchX = -0.00781; //! Same for V6 and V5
  const double rHipYawPitchY = 0.01114; //! Same for V6 and V5
  const double rHipYawPitchZ = 0.02661; //! Same for V6 and V5

  const double rHipRollMass = 0.14053; //! Same for V6 and V5
  const double rHipRollX = -0.01549; //! Same for V6 and V5
  const double rHipRollY = -0.00029; //! Same for V6 and V5
  const double rHipRollZ = -0.00515; //! Same for V6 and V5

  const double rHipPitchMass = 0.38968; //! Same for V6 and V5
  const double rHipPitchX = 0.00138; //! Same for V6 and V5
  const double rHipPitchY = -0.00221; //! Same for V6 and V5
  const double rHipPitchZ = -0.05373; //! Same for V6 and V5

  const double rKneePitchMass = 0.30142; //! Same for V6 and V5
  const double rKneePitchX = 0.00453; //! Same for V6 and V5
  const double rKneePitchY = -0.00225; //! Same for V6 and V5
  const double rKneePitchZ = -0.04936; //! Same for V6 and V5

  const double rAnklePitchMass = 0.13416; //! Same for V6 and V5
  const double rAnklePitchX = 0.00045; //! Same for V6 and V5
  const double rAnklePitchY = -0.00029; //! Same for V6 and V5
  const double rAnklePitchZ = 0.00685; //! Same for V6 and V5

  const double rAnkleRollMass = 0.17184; //! Same for V6 and V5
  const double rAnkleRollX = 0.02542; //! Same for V6 and V5
  const double rAnkleRollY = -0.0033; //! Same for V6 and V5
  const double rAnkleRollZ = -0.03239; //! Same for V6 and V5

  const double headYawVelLimit = 8.26797; //! Same for V6 and V5
  const double headPitchVelLimit = 7.19407; //! Same for V6 and V5
  const double lShoulderPitchVelLimit = 8.26797; //! Same for V6 and V5
  const double lShoulderRollVelLimit = 7.19407; //! Same for V6 and V5
  const double lElbowYawVelLimit = 8.26797; //! Same for V6 and V5
  const double lElbowRollVelLimit = 7.19407; //! Same for V6 and V5
  const double lWristYawVelLimit = 24.6229; //! Same for V6 and V5
  const double rShoulderPitchVelLimit = 8.26797; //! Same for V6 and V5
  const double rShoulderRollVelLimit = 7.19407; //! Same for V6 and V5
  const double rElbowYawVelLimit = 8.26797; //! Same for V6 and V5
  const double rElbowRollVelLimit = 7.19407; //! Same for V6 and V5
  const double rWristYawVelLimit = 24.6229; //! Same for V6 and V5
  const double lHipYawPitchVelLimit = 4.16174; //! Same for V6 and V5
  const double lHipRollVelLimit = 4.16174; //! Same for V6 and V5
  const double lHipPitchVelLimit = 6.40239; //! Same for V6 and V5
  const double lKneePitchVelLimit = 6.40239; //! Same for V6 and V5
  const double lAnklePitchVelLimit = 6.40239; //! Same for V6 and V5
  const double lAnkleRollVelLimit = 4.16174; //! Same for V6 and V5
  const double rHipYawPitchVelLimit = 4.16174; //! Same for V6 and V5
  const double rHipRollVelLimit = 4.16174; //! Same for V6 and V5
  const double rHipPitchVelLimit = 6.40239; //! Same for V6 and V5
  const double rKneePitchVelLimit = 6.40239; //! Same for V6 and V5
  const double rAnklePitchVelLimit = 6.40239; //! Same for V6 and V5
  const double rAnkleRollVelLimit = 4.16174; //! Same for V6 and V5

  const double lFsrFLX = 0.07025; //! Same for V6 and V5
  const double LFsrFLY = 0.0299; //! Same for V6 and V5
  const double LFsrFRX = 0.07025; //! Same for V6 and V5
  const double LFsrFRY = -0.0231; //! Same for V6 and V5
  const double LFsrRLX = -0.03025; //! Same for V6 and V5
  const double LFsrRLY = 0.0299; //! Same for V6 and V5
  const double LFsrRRX = -0.02965; //! Same for V6 and V5
  const double LFsrRRY = -0.0191; //! Same for V6 and V5

  const double RFsrFLX = 0.07025; //! Same for V6 and V5
  const double RFsrFLY = 0.0231; //! Same for V6 and V5
  const double RFsrFRX = 0.07025; //! Same for V6 and V5
  const double RFsrFRY = -0.0299; //! Same for V6 and V5
  const double RFsrRLX = -0.03025; //! Same for V6 and V5
  const double RFsrRLY = 0.0191; //! Same for V6 and V5
  const double RFsrRRX = -0.02965; //! Same for V6 and V5
  const double RFsrRRY = -0.0299; //! Same for V6 and V5

  const std::vector<std::string> jointNames =
  {
    "HeadYaw",
    "HeadPitch",
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw",
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw",
    "LHipYawPitch",
    "LHipRoll",
    "LHipPitch",
    "LKneePitch",
    "LAnklePitch",
    "LAnkleRoll",
    "RHipYawPitch",
    "RHipRoll",
    "RHipPitch",
    "RKneePitch",
    "RAnklePitch",
    "RAnkleRoll"
  };

  const double jointMaxPositions[24] =
  {
    headYawHigh,
    headPitchHigh,
    lShoulderPitchHigh,
    lShoulderRollHigh,
    lElbowYawHigh,
    lElbowRollHigh,
    lWristYawHigh,
    rShoulderPitchHigh,
    rShoulderRollHigh,
    rElbowYawHigh,
    rElbowRollHigh,
    rWristYawHigh,
    lHipYawPitchHigh,
    lHipRollHigh,
    lHipPitchHigh,
    lKneePitchHigh,
    lAnklePitchHigh,
    lAnkleRollHigh,
    rHipYawPitchHigh,
    rHipRollHigh,
    rHipPitchHigh,
    rKneePitchHigh,
    rAnklePitchHigh,
    rAnkleRollHigh
  };

  const double jointMinPositions[24] =
  {
    headYawLow,
    headPitchLow,
    lShoulderPitchLow,
    lShoulderRollLow,
    lElbowYawLow,
    lElbowRollLow,
    lWristYawLow,
    rShoulderPitchLow,
    rShoulderRollLow,
    rElbowYawLow,
    rElbowRollLow,
    rWristYawLow,
    lHipYawPitchLow,
    lHipRollLow,
    lHipPitchLow,
    lKneePitchLow,
    lAnklePitchLow,
    lAnkleRollLow,
    rHipYawPitchLow,
    rHipRollLow,
    rHipPitchLow,
    rKneePitchLow,
    rAnklePitchLow,
    rAnkleRollLow
  };

  const double jointMaxVelocities[24] =
  {
    headYawVelLimit,
    headPitchVelLimit,
    lShoulderPitchVelLimit,
    lShoulderRollVelLimit,
    lElbowYawVelLimit,
    lElbowRollVelLimit,
    lWristYawVelLimit,
    rShoulderPitchVelLimit,
    rShoulderRollVelLimit,
    rElbowYawVelLimit,
    rElbowRollVelLimit,
    rWristYawVelLimit,
    lHipYawPitchVelLimit,
    lHipRollVelLimit,
    lHipPitchVelLimit,
    lKneePitchVelLimit,
    lAnklePitchVelLimit,
    lAnkleRollVelLimit,
    rHipYawPitchVelLimit,
    rHipRollVelLimit,
    rHipPitchVelLimit,
    rKneePitchVelLimit,
    rAnklePitchVelLimit,
    rAnkleRollVelLimit
  };

  const double jointDHParams[24][4] =
  {
    {0.0,            0.0,        0.0,            0.0},
    {0.0,           -M_PI_2,     0.0,           -M_PI_2},
    {0.0,           -M_PI_2,     0.0,            0.0},
    {0.0,            M_PI_2,     0.0,            M_PI_2},
    {elbowOffsetY,   M_PI_2,     upperArmLength, 0.0},
    {0.0,           -M_PI_2,     0.0,            0.0},
    {0.0,            M_PI_2,     lowerArmLength, 0.0},
    {0.0,           -M_PI_2,     0.0,            0.0},
    {0.0,            M_PI_2,     0.0,            M_PI_2},
    {-elbowOffsetY,  M_PI_2,     upperArmLength, 0.0},
    {0.0,           -M_PI_2,     0.0,            0.0},
    {0.0,            M_PI_2,     lowerArmLength, 0.0},
    {0.0,           -3 * M_PI_4, 0.0,           -M_PI_2},
    {0.0,           -M_PI_2,     0.0,            M_PI_4},
    {0.0,            M_PI_2,     0.0,            0.0},
    {-thighLength,   0.0,        0.0,            0.0},
    {-tibiaLength,   0.0,        0.0,            0.0},
    {0.0,           -M_PI_2,     0.0,            0.0},
    {0.0,           -M_PI_4,     0.0,           -M_PI_2},
    {0.0,           -M_PI_2,     0.0,           -M_PI_4},
    {0.0,            M_PI_2,     0.0,            0.0},
    {-thighLength,   0.0,        0.0,            0.0},
    {-tibiaLength,   0.0,        0.0,            0.0},
    {0.0,           -M_PI_2,     0.0,            0.0}
  };

  const double jointMaxTorques[24] =  // N-m
  {
    0.93167,
    1.0740,
    0.93167,
    1.0740,
    0.93167,
    1.0740,
    0.24799,
    0.93167,
    1.0740,
    0.93167,
    1.0740,
    0.24799,
    3.2409,
    3.2409,
    2.1067,
    2.1067,
    2.1067,
    3.2409,
    3.2409,
    3.2409,
    2.1067,
    2.1067,
    2.1067,
    3.2409,
  };

}
