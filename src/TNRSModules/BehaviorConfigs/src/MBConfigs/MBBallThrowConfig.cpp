/**
 * @file MotionModule/src/MotionConfigs/MBBallThrowConfig.cpp
 *
 * This file implements the struct MBBallThrowConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "BehaviorConfigs/include/MBConfigs/MBBallThrowConfig.h"
#include "Utils/include/DataUtils.h"

DEFINE_BEHAVIOR_CONFIG(
  MBBallThrowConfig, MBConfig, MBBallThrowConfigPtr,
  (MBBallThrowTypes, wbBallThrow, WBBallThrowConfig),
)

void WBBallThrowConfig::init() {}
void WBBallThrowConfig::validate()
{
  if (timeToThrow <= 0.f) {
    throw
      BConfigException(
        this,
        "Invalid behavior configuration parameters passed.",
        false
      );
  }
}

