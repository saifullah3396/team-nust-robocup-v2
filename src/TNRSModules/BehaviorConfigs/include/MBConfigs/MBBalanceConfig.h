/**
 * @file BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h
 *
 * This file declares the structs MBBalanceConfig, MPComControlConfig,
 * PIDComControlConfig, and ZmpControlConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "BehaviorConfigs/include/MBConfigs/MBConfig.h"
#include "Utils/include/HardwareIds.h"

template <typename Scalar>
class ZmpRefGenerator;

/**
 * @struct MBBalanceConfig
 * @brief Balance behavior configuration
 * @param supportLeg The support leg to shift the balance to
 * @param timeToReachB Time to shift balance
 */
DECLARE_BEHAVIOR_CONFIG_WITH_VARS(
  MBBalanceConfig,
  MBConfig,
  MBBalanceConfigPtr,
  MBIds::balance,
  20.0,
  MBBalanceTypes,
  (LinkChains, supportLeg, LinkChains::lLeg),
  (float, timeToReachB, 2.0),
)

/**
 * @struct MPComControlConfig
 * @brief Motion primitives based com shift behavior
 */
DECLARE_BEHAVIOR_CONFIG_TYPE(
  MPComControlConfig,
  MBBalanceConfig,
  MBBalanceTypes::mpComControl,
  MPComControlConfigPtr
)

/**
 * @struct PIDComControlConfig
 * @brief PID controller based com balance behavior
 */
DECLARE_BEHAVIOR_CONFIG_TYPE(
  PIDComControlConfig,
  MBBalanceConfig,
  MBBalanceTypes::pidComControl,
  PIDComControlConfigPtr
)

/**
 * @struct ZmpControlConfig
 * @brief Preview controller based zmp dynamic balance behavior
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  ZmpControlConfig,
  MBBalanceConfig,
  MBBalanceTypes::zmpControl,
  ZmpControlConfigPtr,
  (bool, keepOtherLegContact, true),
  (bool, useTargetPosture, true),
  (bool, keepTorsoUpright, false),
  (bool, regularizeIk, true),
  (vector<unsigned>, activeJoints, vector<unsigned>()),
  (Eigen::Vector2f, target, Eigen::Vector2f()),
)
