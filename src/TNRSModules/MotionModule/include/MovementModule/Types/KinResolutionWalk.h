/**
 * @file MotionModule/include/MovementModule/Types/KinResolutionWalk.h
 *
 * This file declares the class KinResolutionWalk
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include <fstream>
#include <boost/shared_ptr.hpp>
#include <deque>
#include "MotionModule/include/MovementModule/MovementModule.h"

template <typename Scalar>
class ZmpPreviewController;
template <typename Scalar>
struct WalkParameters;
template <typename Scalar>
class WalkZmpRefGen;
template <typename Scalar>
struct TNRSFootstep;
struct KinResolutionWalkConfig;

/**
 * @class KinResolutionWalk
 * @brief A class for that defines a robot walk based on input desired speed
 */
template <typename Scalar>
class KinResolutionWalk : public MovementModule<Scalar>
{
public:
  /**
   * @brief KinResolutionWalk Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  KinResolutionWalk(
    MotionModule* motionModule,
    const boost::shared_ptr<KinResolutionWalkConfig>& config);

  /**
   * @brief ~KinResolutionWalk Destructor
   */
  ~KinResolutionWalk() final;

  /**
   * @brief initiate See Behavior::initiate()
   */
  bool initiate() final;

  /**
   * @brief reinitiate See Behavior::reinitiate()
   * @param cfg New configuration
   */
  virtual void reinitiate(const BehaviorConfigPtr& cfg);

  /**
   * @brief update See Behavior::update()
   */
  void update() final;

  /**
   * @brief finish See Behavior::finish()
   */
  void finish() final;

  /**
   * @brief loadExternalConfig See Behavior::loadExternalConfig()
   */
  void loadExternalConfig() final;
private:
  /**
   * @brief getBehaviorCast Returns the cast of config to KinResolutionWalkConfigPtr
   */
  boost::shared_ptr<KinResolutionWalkConfig> getBehaviorCast();
  boost::shared_ptr<WalkParameters<Scalar> > params;

  void addStep(
    const boost::shared_ptr<TNRSFootstep<Scalar> >& fs);
  boost::shared_ptr<TNRSFootstep<Scalar> >& getFrontStep();
  boost::shared_ptr<TNRSFootstep<Scalar> >& getBackStep();
  void popStep();

  /**
   * @brief computeStepFromTo Computes the transformation of a robot step
   *   from other leg step and returns the step
   * @param from The step from which transformation is generated
   * @param to The new step by other leg
   * @param transFromTo Transformation matrix
   * @return TNRSFootstep
   */
  TNRSFootstep<Scalar> computeStepFromTo(
    const TNRSFootstep<Scalar>& from,
    const TNRSFootstep<Scalar>& to,
    Eigen::Matrix<Scalar, 4, 4>& transFromTo);

  /**
   * @brief stepVelToPose Generates a step update based on
   *   velocity input
   * @param vi Desired normalized velocity of the robot
   * @return Step within constraints for given velocity
   */
  RobotPose2D<Scalar> stepVelToPose(const VelocityInput<Scalar>& vi);

  /**
   * @brief addFirstStep Add first step of robot using the initial position
   *   of the step from global base chain
   */
  void addFirstStep();

  /**
   * @brief genNextStep Generates the next step in sequence based on
   *   desired velocity input and updates the steps queue
   */
  void genNextStep();

  /**
   * @brief genStepTrajectory Generates the trajectory of next step
   *   in sequence
   */
  void genStepTrajectory();

  /**
   * @brief drawSteps Draws all the steps in queue on an image
   */
  void drawSteps();

  ///< Foot steps queue
  std::deque<boost::shared_ptr<TNRSFootstep<Scalar>>> stepsQueue;

  ///< Walk zmp reference generator
  boost::shared_ptr<WalkZmpRefGen<Scalar> > walkZmpRefGen;

  ///< Zmp preview controllers for x-y directions
  vector<ZmpPreviewController<Scalar>*> controllers;

  ///< Latest step trajectory
  Matrix<Scalar, Dynamic, Dynamic> stepTraj;

  ///< The index of step trajectory in the cycle
  unsigned stepTrajIndex = {0};

  ///< Current base support foot
  RobotFeet currentBase;

  ///< Accumulative orientations of feet
  Matrix<Scalar, 4, 4> globalFeetOrientation;

  static Scalar maxHipLOffset;
  static Scalar maxHipROffset;

  ///< Log for storing center of mass data
  fstream comLog;

  ///< Log for storing zmp ref data
  fstream zmpRegLog;
};

typedef boost::shared_ptr<KinResolutionWalk<MType> > KinResolutionWalkPtr;
