/**
 * @file MotionModule/include/KickModule/Types/JointSpaceKick.h
 *
 * This file declares the class JointSpaceKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#pragma once

#include "BehaviorManager/include/StateMachineMacros.h"
#include "MotionModule/include/KickModule/KickModule.h"

struct JSKickConfig;
struct ZmpControlConfig;

/**
 * @class JointSpaceKick
 * @brief The base class for defining kicking motions using
 *   joint space planning
 */
template <typename Scalar>
class JointSpaceKick : public KickModule<Scalar>
{
public:
  /**
   * @brief JointSpaceKick Constructor
   * @param motionModule Pointer to base motion module
   * @param config Configurtion of this behavior
   * @param name Name of this behavior
   */
  JointSpaceKick(
    MotionModule* motionModule,
    const boost::shared_ptr<JSKickConfig>& config,
    const string& name = "JointSpaceKick");

  /**
   * @brief ~JointSpaceKick Destructor
   */
  virtual ~JointSpaceKick() {}

  /**
   * @brief initiate See Behavior::initiate()
   */
  bool initiate() override;

  /**
   * @brief update See Behavior::update()
   */
  void update() override;

  /**
   * @brief finish See Behavior::finish()
   */
  void finish() override;

  /**
   * @brief loadExternalConfig See Behavior::loadExternalConfig()
   */
  virtual void loadExternalConfig() override;

protected:
  /**
   * @brief setupKickBase Sets up the kick parameters according to the
   *   behavior configuration
   */
  virtual void setupKickBase() = 0;

  /**
   * @brief solveForImpact Solves for the impact conditions based on the
   *   underlying model
   */
  virtual void solveForImpact() = 0;

  /**
   * @brief plotKick Plots the planned kick trajectory
   */
  void plotKick();

  /**
   * @brief logEndEffectorActual Logs end-effector motion using actual
   *   joint states
   */
  void logEndEffectorActual();

  /**
   * @brief defineTrajectory Defines the overall kicking motion
   */
  virtual void defineTrajectory();

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * @brief requestExecution Requests kick execution based on naoqi
   *   joint interpolation
   * @param addArmsMovement Whether to use arms for increased stability
   */
  virtual void requestExecution(const bool& addArmsMovement = false);
  #endif

  ///< Finite state machine for this behavior
  DECLARE_FSM(fsm, JointSpaceKick<Scalar>)

  ///< GoToPosture: Sends the robot to start posture for kicking
  DECLARE_FSM_STATE(JointSpaceKick<Scalar>, SetStartPosture, setStartPosture, onStart, onRun,)

  ///< GoToBalance: Shifts the balance to support foot
  DECLARE_FSM_STATE(JointSpaceKick<Scalar>, GoToBalance, goToBalance, onStart, onRun,)

  ///< PlanKick: Plans the kicking motion
  DECLARE_FSM_STATE(JointSpaceKick<Scalar>, PlanKick, planKick, onRun,)

  ///< ExecuteKick: Executes the kick
  DECLARE_FSM_STATE(JointSpaceKick<Scalar>, ExecuteKick, executeKick, onStart, onRun,)

  ///< SetPostKickPosture: Sets the posture to required one after kick
  DECLARE_FSM_STATE(JointSpaceKick<Scalar>, SetPostKickPosture, setPostKickPosture, onStart, onRun,)

  ///< Zmp control config for hands after balance shift
  boost::shared_ptr<ZmpControlConfig> zmpControlCfgInKick;

  ///< Ratio of distance from impact pose in terms of ball radius for the pose before impact pose
  static Scalar preImpactBeta;

  ///< Tolerance distance for lifting the foot off the ground before moving in x-y directions. Used
  ///< in second pose
  static Scalar preImpactAlpha;

  ///< Distance tolerance of foot from ground on end placement. This is used so that the robot
  ///< does not hit the ground
  static Scalar postImpactGroundDist;

  ///< Maximum number of iterations for solving inverse kinematics for each of the poses
  static unsigned numIkIterations;

  ///< Initial knots values
  static Scalar knotsInitialValue;

  ///< If true, angular velocity jacobian is used to find impact pose joint velocities
  static bool solveForImpactOmega;

  ///< If true, hipyawpitch joint is used in kick generation
  static bool useHipYawPitchJoint;

  ///< If true, torque constraints are added on kick motion
  static bool addTorqueConstraint;

  ///< If true, zmp constraints are added on kick motion
  static bool addZmpConstraint;

  ///< Maximum number of iterations allowed for constant velocity increment
  static unsigned maxConstantVelIterations;

  ///< Cut-off distance for constant velocity zone in terms of ball radius
  static Scalar constantVelCutoffDist;

  ///< Wait time after kick to change posture
  static Scalar afterKickWait;

  ///< Wait time after balance to kick
  static Scalar afterBalanceWait;

  ///< Wait time after posture to balance
  static Scalar afterPostureWait;

  ///< Minumum limit for the total optimized trajectory time
  Scalar minTimeToKick = {1.0};

  ///< Time taken by kick trajectory until it reaches the hitting pose
  Scalar kickTimeToImpact = {0.0};

  ///< Time of kick execution
  Scalar execTime = {0.0};

  ///< Vector of required cartesian poses for interpolation before impact
  vector<Matrix<Scalar, 4, 4>> cPosesPre;

  ///< Vector of required cartesian poses for interpolation after impact
  vector<Matrix<Scalar, 4, 4>> cPosesPost;

  ///< Discretized kick trajectories with step size equal to cycleTime.
  vector<vector<Scalar> > jointTrajectories;

private:
  /**
   * @brief getBehaviorCast Casts the behavior config to JSKickConfig
   * @return boost::shared_ptr<JSKickConfig>
   */
  boost::shared_ptr<JSKickConfig> getBehaviorCast();
};

typedef boost::shared_ptr<JointSpaceKick<MType> > JointSpaceKickPtr;
