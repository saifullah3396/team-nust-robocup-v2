/**
 * @file MotionModule/include/KickModule/Types/CSpaceBSplineKick.h
 *
 * This file declares the class CSpaceBSplineKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#pragma once

#include "MotionModule/include/KickModule/KickModule.h"

template <typename Scalar>
class BSpline;
class JointCmdsRecorder;
struct CSpaceBSplineKickConfig;

template <typename Scalar>
class CSpaceBSplineKick : public KickModule<Scalar>
{
public:
  /**
   * @brief CSpaceBSplineKick Constructor
   * @param motionModule Pointer to base motion module
   * @param config Configuration of the behavior
   */
  CSpaceBSplineKick(
    MotionModule* motionModule,
    const boost::shared_ptr<CSpaceBSplineKickConfig>& config);

  /**
   * @brief ~CSpaceBSplineKick Destructor
   */
  ~CSpaceBSplineKick() final {}

  /**
   * @brief initiate See Behavior::initiate()
   */
  bool initiate() final;

  /**
   * @brief update See Behavior::update()
   */
  void update() final;

  /**
   * @brief finish See Behavior::finish()
   */
  void finish() final;
private:
  /**
   * @brief getBehaviorCast Casts the config to CSpaceBSplineKickConfig
   * @return boost::shared_ptr<CSpaceBSplineKickConfig>
   */
  boost::shared_ptr<CSpaceBSplineKickConfig> getBehaviorCast();

  /**
   * @brief setupKickBase Sets up kick parameters according
   *   to the behavior configuration
   */
  void setupKickBase();

  /**
   * @brief solveForImpact Finds best the end-effector point and
   *   orientation based on maximum mass*velocity product and
   *   x-coordinate on the foot surface.
   */
  virtual void solveForImpact();

  /**
   * @brief defineTrajectory Defines the overall kicking motion
   */
  void defineTrajectory();

  //! Bspline trajectory object
  boost::shared_ptr<BSpline<Scalar> > bSpline;

  //! Bspline trajectory object
  Matrix<Scalar, Dynamic, Dynamic> cartTraj;

  //! Current state of this behavior
  unsigned behaviorState;

  //! Joints to be used for kicking while balancing
  vector<bool> activeJoints;

  //! Whether the requested kick is not acheivable
  bool kickFailed = {false};

  /**
   * States of this behavior
   *
   * @enum BehaviorState
   */
  enum BehaviorState
  {
    posture,
    balance,
    kick,
    postKickPosture
  };
};

typedef boost::shared_ptr<CSpaceBSplineKick<MType>> CSpaceBSplineKickPtr;
