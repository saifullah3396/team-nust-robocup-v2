/**
 * @file PlanningModule/include/NavigationBehavior/NavigationBehavior.h
 *
 * This file declares the class NavigationBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 06 Oct 2017
 */

#pragma once

#include <Eigen/Dense>
#include "TNRSBase/include/DebugBase.h"
#include "PlanningModule/include/PlanningBehavior.h"
#include "Utils/include/PathPlanner/State.h"
#include "Utils/include/PathPlanner/helper.h"
#include "Utils/include/DataHolders/RobotPose2D.h"

using namespace Eigen;
using namespace Utils;
namespace PathPlannerSpace {
  class State;
  class PathPlanner;
}
struct PBNavigationConfig;
template <typename Scalar>
struct RobotPose2D;

/**
 * @class NavigationBehavior
 * @brief The base class for defining robot navigation
 */
class NavigationBehavior : public PlanningBehavior, public DebugBase
{
  INIT_DEBUG_BASE_(
    //! Option to send planned footsteps
    (int, sendFootsteps, 0),
    //! Option to draw planned footsteps
    (int, drawFootsteps, 0),
  )
public:
  /**
   * @brief NavigationBehavior Constructor
   *
   * @param planningModule: Pointer to base planning module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  NavigationBehavior(
    PlanningModule* planningModule,
    const boost::shared_ptr<PBNavigationConfig>& config,
    const string& name = "NavigationBehavior");

  /**
   * Default destructor for this class
   */
  virtual ~NavigationBehavior() {}

  /**
   * Returns its own child based on the given type
   *
   * @param planningModule: Pointer to base planning module
   * @param cfg: Config of the requested behavior
   *
   * @return BehaviorConfigPtr
   */
  static boost::shared_ptr<NavigationBehavior> getType(
    PlanningModule* planningModule, const BehaviorConfigPtr& cfg);

  /**
   * Derived from Behavior
   */
  virtual bool initiate() override;
  virtual void reinitiate(const BehaviorConfigPtr& cfg) override;
  virtual void update() override;
  virtual void finish() override;

  /**
   * Derived from Behavior. Child type may or may not use the same
   * behavior config as parent.
   */
  virtual void loadExternalConfig() override;

protected:
  virtual void planPathAction();
  virtual void executeMotionAction() = 0;
  virtual void validatePathAction();
  void execUnchangingStepsAction();

  /**
   * Finds the starting feet placement and sets the starting pose
   * in path planner
   *
   * @return true if starting pose is set up
   */
  bool setStart();

  /**
   * Sets the starting pose in path planner
   *
   * @param left: Left foot state in the environment
   * @param right: Right foot state in the environment
   *
   * @return true if starting pose is set up
   */
  virtual bool setStart(const PathPlannerSpace::State& left, const PathPlannerSpace::State& right) override;

  /**
   * Sets the goal position to reach in the environment
   *
   * @param goal: Desired goal robot pose
   *
   * @return true if goal is successfully set up
   */
  bool setGoal(const RobotPose2D<float>& goal);

  /**
   * Finds the closest possible goal in the direction from initial pose
   * to goal pose
   *
   * @param goal: Desired goal robot pose
   *
   * @return true if a possible goal is found
   */
  bool findPossibleGoal(RobotPose2D<float>& goal);

  /**
   * Gets the feet transformation frames in environment from robot pose
   * and kinematics info.
   *
   * @param foot: Transformation frame to be updated
   * @param id: Foot id defined in path planner
   *
   * @return true if a valid transformation matrix is found
   */
  bool getFootTransform(Matrix<float, 4, 4>& foot, const PathPlannerSpace::Leg& id);

  /**
   * Checks whether the current path is no more valid
   */
  bool checkPathValidity();

  /**
   * Wrapper for pathPlanner->plan()
   */
  bool plan(const bool& removeFirstStep = true);

  /**
   * Wrapper for pathPlanner->replan()
   */
  bool replan();

  /**
   * Initiates the walking behavior
   */
  void executeWalk();

  /**
   * Draws planned footsteps on an image
   */
  void drawFootstepsData();

  /**
   * Send planned footsteps to clients through the comm module
   */
  void sendFootstepsData();

  /// Search direction
  bool forwardSearch;

  //! Vector of path steps
  vector<PathPlannerSpace::State> plannedPath;

  //! Transformation frame of the right foot in previous cycle
  Matrix<float, 4, 4> prevRFoot;

  //! Transformation frame of the left foot in previous cycle
  Matrix<float, 4, 4> prevLFoot;

  //! The start footstep id for tracking footsteps
  int startStep;

  //! Current goal pose
  RobotPose2D<float> goal;

  //! The path planner object
  boost::shared_ptr<PathPlannerSpace::PathPlanner> pathPlanner;

  //! Current state of this behavior
  unsigned behaviorState;

  //! Whether the path is currently planned
  bool pathPlanned;

  //! Whether the path is sent for execution
  bool pathExecuted;

  //! Fail count while planning
  int planFailCount;

  //! Fail count while re-planning
  int replanFailCount;

  //! Max tries for planning
  static int planMaxTries;

  //! Max tries for re-planning
  static int replanMaxTries;

  /**
   * States of this behavior
   *
   * @enum BehaviorState
   */
  enum BehaviorState
  {
    planPath,
    executeMotion,
    validatePath
  };

  enum MBManagerIds {
    MOTION_1
  };

private:
  /**
   * Returns the cast of config to MBBallThrowConfigPtr
   */
  boost::shared_ptr<PBNavigationConfig> getBehaviorCast();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<NavigationBehavior> NavigationBehaviorPtr;
