/**
 * @file MotionModule/DiveModule/Types/HandSaveDive.h
 *
 * This file declares the class HandSaveDive
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/DiveModule/DiveModule.h"

template <typename Scalar>
class MotionTask;
typedef boost::shared_ptr<MotionTask<MType> > TaskPtr;

template <typename Scalar>
class HandSaveDive : public DiveModule<Scalar>
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  HandSaveDive(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config) :
    DiveModule<Scalar>(motionModule, config, "HandSaveDive"),
    diveTime(0.f)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~HandSaveDive()
  {
  }

  /**
   * Derived from Behavior
   */
  bool initiate() final;
  void update() final;
  void finish() final;
  void loadExternalConfig() final;

private:
  /**
   * Returns the cast of config to KFMDiveConfigPtr
   */
  HandSaveDiveConfigPtr getBehaviorCast();
  Scalar diveTime;

  //! Weights and gains
  static vector<Scalar> taskWeights;
  static vector<Scalar> taskGains;

  //! Tasks for solving inverse kinematics
  vector<TaskPtr> tasks;

  enum Tasks {
    POSTURE_TASK,
    TORSO_TASK,
    OTHER_LEG_TASK,
    HAND_TASK,
    NUM_TASKS
  };
};

typedef boost::shared_ptr<HandSaveDive<MType> > HandSaveDivePtr;
