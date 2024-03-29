/**
 * @file MotionModule/src/MotionConfigs/MBMovementConfig.cpp
 *
 * This file implements the structs MBMovementConfig
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */
#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/PathPlanner/PathPlanner.h"

using namespace PathPlannerSpace;

DEFINE_BEHAVIOR_CONFIG(
  MBMovementConfig, MBConfig, MBMovementConfigPtr,
  (MBMovementTypes, naoqiFootsteps, NaoqiFootstepsConfig),
  (MBMovementTypes, naoqiMoveToward, NaoqiMoveTowardConfig),
  (MBMovementTypes, speedWalk, SpeedWalkConfig),
)

void NaoqiFootstepsConfig::init() {}
void NaoqiFootstepsConfig::validate()
{
  //! Throw a BConfigException is behavior configuration is invalid
}

void NaoqiMoveTowardConfig::init() {}
void NaoqiMoveTowardConfig::validate()
{
  //! Throw a BConfigException is behavior configuration is invalid
}

void SpeedWalkConfig::init() {}
void SpeedWalkConfig::validate()
{
  //! Throw a BConfigException is behavior configuration is invalid
}

