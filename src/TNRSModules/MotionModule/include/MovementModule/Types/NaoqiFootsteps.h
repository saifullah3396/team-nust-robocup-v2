/**
 * @file MotionModule/MovementModule/Types/NaoqiFootsteps.h
 *
 * This file declares the class NaoqiFootsteps
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

template <typename Scalar>
struct TNRSFootstep;

#include <queue>
#include "MotionModule/include/MovementModule/MovementModule.h"

struct NaoqiFootstepsConfig;

/**
 * @class NaoqiFootsteps
 * @brief A class for defining movement of the robot using naoqi based
 *   footsteps execution
 */
template <typename Scalar>
class NaoqiFootsteps : public MovementModule<Scalar>
{
public:
  /**
   * @brief NaoqiFootsteps Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  NaoqiFootsteps(
    MotionModule* motionModule,
    const boost::shared_ptr<NaoqiFootstepsConfig>& config) :
    MovementModule<Scalar>(motionModule, config, "NaoqiFootsteps")
  {
  }

  /**
   * @brief ~NaoqiFootsteps Destructor
   */
  ~NaoqiFootsteps() final {}

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
   * Returns the cast of config to MBBallThrowConfigPtr
   */
  boost::shared_ptr<NaoqiFootstepsConfig> getBehaviorCast();

  /**
   * @brief Starts walk execution
   * @return true if walk execution is successful
   */
  bool initWalk();

  /**
   * @brief Checks if the walking is completely stopped
   * @return true if walk is completely finished with no step in progress
   */
  bool isWalkFinished();

  /**
   * @brief Gets the index of the current step from planned walking steps
   * @return unsigned
   */
  unsigned getCurrentStep();

  /**
   * @brief Gets the step transformation from 'from' to 'to' foot
   * @param from state of the support foot
   * @param to final state of the stepping foot
   * @param stepTrans transformation from support foot to step foot
   * @return State
   */
  TNRSFootstep<float> getFootstep(
    const TNRSFootstep<float>& from, const TNRSFootstep<float>& to, Matrix<Scalar, 4, 4>& stepTrans);

  void setPostureAction();
  void setupWalkAction();
  void updateWalkAction();
  void stopWalkAction();
  void postStopWalkAction();

  //! Planned footsteps
  vector<TNRSFootstep<float>> plannedPath;

  //! Time taken by one step
  static Scalar footstepTime;

  //! Projected position updates if a step is executed
  queue<boost::shared_ptr<TNRSFootstep<Scalar>>> steps;

  //! The current footstep start time
  Scalar footStepStartTime;

  //! Torso to current step
  Matrix<Scalar, 4, 4> torsoToSupportFoot = Matrix<Scalar, 4, 4>::Identity();

  //! The start footstep id for tracking footsteps from Naoqi
  int startStep = {0};

  //! Current step index in planned footsteps
  unsigned currentStep = {0};

  //! The previous footstep id for tracking footsteps
  int prevStep = {-1};

  //! Current state of this behavior
  unsigned behaviorState = {setPosture};

  /**
   * States of this behavior
   *
   * @enum BehaviorState
   */
  enum BehaviorState
  {
    setPosture,
    setupWalk,
    updateWalk,
    stopWalk,
    postStopWalk
  };
};

typedef boost::shared_ptr<NaoqiFootsteps<MType> > NaoqiFootstepsPtr;
