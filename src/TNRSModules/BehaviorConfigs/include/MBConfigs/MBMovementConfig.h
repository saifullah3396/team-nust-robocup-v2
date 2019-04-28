/**
 * @file TNRSModules/BehaviorConfigs/include/MBConfigs/MBMovementConfig.h
 *
 * This file defines the structs MBMovementConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */
#pragma once

#include "BehaviorConfigs/include/MBConfigs/MBConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/VelocityInput.h"
#include "Utils/include/PathPlanner/State.h"
#include "Utils/include/DataHolders/TNRSFootstep.h"

namespace PathPlannerSpace
{
  typedef vector<State>::const_iterator StateIterT;
}

template <typename Scalar>
struct TNRSFootstep;

/**
 * @brief WalkInputType enumeration for type of input
 *   given to the movement module
 */
enum class WalkInputType {
  footsteps,
  velocity
};

/**
 * @struct MBMovementConfig
 * @brief Movement behavior base configuration
 */
DECLARE_BEHAVIOR_CONFIG_WITH_VARS(
  MBMovementConfig,
  MBConfig,
  MBMovementConfigPtr,
  MBIds::movement,
  3600.0,
  MBMovementTypes,
  (MBPostureConfigPtr, startPosture, InterpToPostureConfigPtr()),
  (MBPostureConfigPtr, endPosture, InterpToPostureConfigPtr()),
);

/**
 * @struct NaoqiMoveTowardConfig
 * @brief Naoqi based movement based on preplanned footsteps
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  NaoqiFootstepsConfig,
  MBMovementConfig,
  MBMovementTypes::naoqiFootsteps,
  NaoqiFootstepsConfigPtr,
  (vector<TNRSFootstep<float>>, plannedPath, vector<TNRSFootstep<float>>()),
);

/**
 * @struct NaoqiMoveTowardConfig
 * @brief Velocity input based movement using Naoqi
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  NaoqiMoveTowardConfig,
  MBMovementConfig,
  MBMovementTypes::naoqiMoveToward,
  NaoqiMoveTowardConfigPtr,
  (VelocityInput<float>, velocityInput, VelocityInput<float>()),
);

/**
 * @struct NaoqiMoveToConfig
 * @brief Goal based movement using naoqi
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  NaoqiMoveToConfig,
  MBMovementConfig,
  MBMovementTypes::naoqiMoveTo,
  NaoqiMoveToConfigPtr,
  (RobotPose2D<float>, goal, RobotPose2D<float>()),
);

/**
 * @struct SpeedWalkConfig
 * @brief Movement based on whole-body ik and preview controllers for
 *   speed-based walking behavior
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  SpeedWalkConfig,
  MBMovementConfig,
  MBMovementTypes::speedWalk,
  SpeedWalkConfigPtr,
  (VelocityInput<float>, velocityInput, VelocityInput<float>()),
  (bool, minimizeJointVels, false),
  (bool, keepTorsoUpright, false),
  (bool, addHipCompensation, false),
  (int, maxNSteps, 1),
  (Vector2f, refOffset, Vector2f::Zero()),
);

/**
 * @struct KinResolutionWalkConfig
 * @brief Movement based on whole-body ik and preview controllers for
 *   speed-based walking behavior
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  KinResolutionWalkConfig,
  MBMovementConfig,
  MBMovementTypes::kinResolutionWalk,
  KinResolutionWalkConfigPtr,
  (VelocityInput<float>, velocityInput, VelocityInput<float>()),
  (bool, addHipCompensation, false),
  (int, maxNSteps, 1),
  (Vector2f, refOffset, Vector2f::Zero()),
);
