/**
 * @file BehaviorConfigs/include/MBConfigs/MBMovementConfig.h
 *
 * This file defines the structs MBMovementConfig
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */
#pragma once

#include "MBConfig.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/VelocityInput.h"
#include "Utils/include/PathPlanner/State.h"

namespace PathPlannerSpace
{
  typedef vector<State>::const_iterator StateIterT;
}

struct MBPostureConfig;
typedef boost::shared_ptr<MBPostureConfig> MBPostureConfigPtr;

namespace PathPlannerSpace {
  class PathPlanner;
  typedef boost::shared_ptr<PathPlanner> PathPlannerPtr;
}

using namespace PathPlannerSpace;

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
 * @brief Movement behavior configuration
 */
struct MBMovementConfig : MBConfig
{
  /**
   * Constructor
   *
   * @param type: Type of MovementModule
   */
  MBMovementConfig(const MBMovementTypes& type);
  
  /**
   * @derived
   */
  virtual bool assignFromJson(const Json::Value& obj);

  /**
   * @derived
   */
  virtual Json::Value getJson();

  /**
   * Makes an object of type this and returns it if valid
   */
  static boost::shared_ptr<MBMovementConfig>
    makeFromJson(const Json::Value& obj);

  MBPostureConfigPtr startPosture;
  MBPostureConfigPtr endPosture;
};

typedef boost::shared_ptr<MBMovementConfig> MBMovementConfigPtr;

/**
 * @struct NaoqiFootstepsConfig
 * @brief Naoqi footsteps based movement behavior configuration
 */
struct NaoqiFootstepsConfig : MBMovementConfig
{
  /**
   * Constructor
   *
   * @param plannedPath: Path given in terms of footsteps
   * @param type: Type of MovementModule
   */
  NaoqiFootstepsConfig(
    const vector<PathPlannerSpace::State>& plannedPath = vector<PathPlannerSpace::State>());

  /**
   * @derived
   */
  void validate();

  /**
   * @derived
   */
  virtual bool assignFromJson(const Json::Value& obj);

  /**
   * @derived
   */
  virtual Json::Value getJson();

  /**
   * Makes an object of type this and returns it if valid
   */
  static boost::shared_ptr<NaoqiFootstepsConfig>
    makeFromJson(const Json::Value& obj);

  vector<PathPlannerSpace::State> plannedPath;
};
typedef boost::shared_ptr<NaoqiFootstepsConfig> NaoqiFootstepsConfigPtr;

/**
 * @struct NaoqiMoveTowardConfig
 * @brief Naoqi move toward based movement behavior configuration
 */
struct NaoqiMoveTowardConfig : MBMovementConfig
{
  /**
   * Constructor
   *
   * @param plannedPath: Path given in terms of footsteps
   * @param type: Type of MovementModule
   */
  NaoqiMoveTowardConfig(
    const VelocityInput<float>& velocityInput = VelocityInput<float>());

  /**
   * @derived
   */
  void validate();

  /**
   * @derived
   */
  virtual bool assignFromJson(const Json::Value& obj);

  /**
   * @derived
   */
  virtual Json::Value getJson();

  /**
   * Makes an object of type this and returns it if valid
   */
  static boost::shared_ptr<NaoqiMoveTowardConfig>
    makeFromJson(const Json::Value& obj);

  VelocityInput<float> velocityInput;
};
typedef boost::shared_ptr<NaoqiFootstepsConfig> NaoqiFootstepsConfigPtr;

/**
 * @struct SpeedWalkConfig
 * @brief Config for speed-based walking behavior
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  SpeedWalkConfig,
  MBMovementConfig,
  MBMovementTypes::speedWalk,
  SpeedWalkConfigPtr,
  (VelocityInput<float>, velocityInput, VelocityInput<float>()),
  (bool, minimizeJointVels, false),
  (bool, keepTorsoUpright, false),
)
