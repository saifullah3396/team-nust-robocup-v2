/**
 * @file TNRSModules/BehaviorConfigs/include/MBConfigs/MBPostureConfig.h
 *
 * This file defines the structs MBPostureConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include <Eigen/Dense>
#include "MBConfig.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "Utils/include/HardwareIds.h"

/**
 * @struct MBPostureConfig
 * @brief Posture behavior base configuration
 */
DECLARE_BEHAVIOR_CONFIG_WITH_VARS(
  MBPostureConfig,
  MBConfig,
  MBPostureConfigPtr,
  MBIds::posture,
  10.0,
  MBPostureTypes,
  (float, timeToReachP, 2.0),
  (PostureState, targetPosture, PostureState::unknown),
);

/**
 * @struct InterpToPostureConfig
 * @brief Simply interpolates to a desired posture using quintic splines
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  InterpToPostureConfig,
  MBPostureConfig,
  MBPostureTypes::interpToPosture,
  InterpToPostureConfigPtr,
  (Eigen::VectorXf, jointsToReach, Eigen::VectorXf()),
);
