/**
 * @file MotionModule/src/MotionConfigs/MBKickConfig.cpp
 *
 * This file implements the structs MBKickConfig, JSKickConfig,
 * JSE2DImpKickConfig and JSOImpKickConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "BehaviorConfigs/include/MBConfigs/MBKickConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  MBKickConfig, MBConfig, MBKickConfigPtr,
  (MBKickTypes, jsoImpKick, JSOImpKickConfig),
  (MBKickTypes, jse2DImpKick, JSE2DImpKickConfig),
  (MBKickTypes, cSpaceBSplineKick, CSpaceBSplineKickConfig),
)

void JSOImpKickConfig::init()
{
  if (!balanceConfig) //! Default config for balancing
    balanceConfig = boost::make_shared<MPComControlConfig>();
}

void JSOImpKickConfig::validate()
{
  if (ball.x < 0.f ||
      //(reqVel.x == -1.f && target.x == -1.f) ||
      minTimeToKick <= 0.f)
  {
    throw
      BConfigException(
        this,
        "Invalid behavior configuration parameters passed.",
        false
      );
  }
}

void JSE2DImpKickConfig::init()
{
  if (!balanceConfig) //! Default config for balancing
    balanceConfig = boost::make_shared<MPComControlConfig>();
}

void JSE2DImpKickConfig::validate()
{
  if (ball.x < 0.f ||
      //(target.x == -1.f && targetDistAngle[0] == -1.f) ||
      minTimeToKick <= 0.f ||
      timeUntilImpact < 0.5f) // Minimum time given to kick should be 0.5 secs
  {
    throw
      BConfigException(
        this,
        "Invalid behavior configuration parameters passed.",
        false
      );
  }
}

void CSpaceBSplineKickConfig::init()
{
  if (!balanceConfig) //! Default config for balancing
    balanceConfig = boost::make_shared<MPComControlConfig>();
}

void CSpaceBSplineKickConfig::validate()
{
  //! Throw a BConfigException is behavior configuration is invalid
}
