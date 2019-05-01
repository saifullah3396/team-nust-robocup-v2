/**
 * @file TNRSModules/BehaviorConfigs/src/MBConfigs/MBBalanceConfig.cpp
 *
 * This file defines the structs MBBalanceConfig, MPComControlConfig,
 * PIDComControlConfig, and ZmpControlConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"
#include "Utils/include/HardwareIds.h"

DEFINE_BEHAVIOR_CONFIG(
  MBBalanceConfig, MBConfig, MBBalanceConfigPtr,
  (MBBalanceTypes, mpComControl, MPComControlConfig),
  (MBBalanceTypes, pidComControl, PIDComControlConfig),
  (MBBalanceTypes, zmpControl, ZmpControlConfig),
);

void MPComControlConfig::init() {}
void MPComControlConfig::validate()
{
  if (
    supportLeg != LinkChains::lLeg &&
    supportLeg != LinkChains::rLeg ||
    timeToReachB < 0.5)
  {
    LOG_INFO("Config parameters: ");
    LOG_INFO("supportLeg: " << toUType(supportLeg));
    LOG_INFO("timeToReachB: " << timeToReachB);
    throw
      BConfigException(
        this,
        "Invalid behavior configuration parameters passed.",
        false
      );
  }
}

void PIDComControlConfig::init() {}
void PIDComControlConfig::validate()
{
  if (
    supportLeg != LinkChains::lLeg &&
    supportLeg != LinkChains::rLeg)
  {
    throw
      BConfigException(
        this,
        "Invalid behavior configuration parameters passed.",
        false
      );
  }
}

void ZmpControlConfig::init()
{
  // Use all 5 joints each of lower body (except hip yaw pitch) as default
  activeJoints = vector<unsigned> (toUType(Joints::count), 0);
  for (int i = toUType(Joints::lHipRoll); i <= toUType(Joints::lAnkleRoll); ++i)
    activeJoints[i] = 1;
  for (int i = toUType(Joints::rHipRoll); i <= toUType(Joints::rAnkleRoll); ++i)
    activeJoints[i] = 1;
}

void ZmpControlConfig::validate()
{
  if (activeJoints.size() != toUType(Joints::count))// || !refGenerator)
  {
    throw
      BConfigException(
        this,
        "Invalid behavior configuration parameters passed.",
        false
      );
  }
}
