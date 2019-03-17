/**
 * @file MotionModule/src/MotionConfigs/MBPostureConfig.cpp
 *
 * This file implements the structs MBPostureConfig 
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */
 
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "MotionModule/include/PostureModule/PostureDefinitions.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "Utils/include/HardwareIds.h"

DEFINE_BEHAVIOR_CONFIG(
  MBPostureConfig, MBConfig, MBPostureConfigPtr,
  (MBPostureTypes, interpToPosture, InterpToPostureConfig),
)

void InterpToPostureConfig::init()
{
  if (toUType(targetPosture) < toUType(PostureState::staticPostures)) {
    jointsToReach =
      VectorXf::Map(
        &postureDefinitions[toUType(targetPosture)][0],
        sizeof(postureDefinitions[toUType(targetPosture)]) /
        sizeof(postureDefinitions[toUType(targetPosture)][0])
      );
  }
}

void InterpToPostureConfig::validate()
{
  if (timeToReachP <= 0.f || // Undefined time given
      jointsToReach.size() != static_cast<unsigned>(Joints::count))
  {
    throw
      BConfigException(
        this,
        "Invalid behavior configuration parameters passed.",
        false
      );
  }
}
