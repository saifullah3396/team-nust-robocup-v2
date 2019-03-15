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

struct HandSaveDiveConfig;
template <typename Scalar>
class MotionTask;
typedef boost::shared_ptr<MotionTask<MType> > TaskPtr;

template <typename Scalar>
class HandSaveDive : public DiveModule<Scalar>
{
public:
  /**
   * @brief HandSaveDive Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  HandSaveDive(
    MotionModule* motionModule,
    const boost::shared_ptr<HandSaveDiveConfig>& config);

  /**
   * @brief ~HandSaveDive Destructor
   */
  ~HandSaveDive() final {}

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

  /**
   * @brief finish See Behavior::loadExternalConfig()
   */
  void loadExternalConfig() final;

private:
  /**
   * @brief getBehaviorCast Returns the casted config
   * @return boost::shared_ptr<HandSaveDiveConfig>
   */
  boost::shared_ptr<HandSaveDiveConfig> getBehaviorCast();

  //! Weights and gains
  static vector<Scalar> taskWeights;
  static vector<Scalar> taskGains;

  //! Tasks for solving inverse kinematics
  vector<TaskPtr> tasks;

  //! Total dive time
  Scalar diveTime;

  enum Tasks {
    POSTURE_TASK,
    TORSO_TASK,
    OTHER_LEG_TASK,
    HAND_TASK,
    NUM_TASKS
  };
};

typedef boost::shared_ptr<HandSaveDive<MType> > HandSaveDivePtr;
