/**
 * @file MotionModule/include/Teleop/Types/TeleopJoints.h
 *
 * This file declares the class TeleopJoints
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/Teleop/Teleop.h"
#include "Utils/include/HardwareIds.h"

struct TeleopJointsConfig;
template <typename Scalar>
class PostureTask;
typedef boost::shared_ptr<PostureTask<MType> > PostureTaskPtr;

template <typename Scalar>
class TeleopJoints : public Teleop<Scalar>
{
public:
  /**
   * @brief TeleopJoints Constructor
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  TeleopJoints(
    MotionModule* motionModule,
    const boost::shared_ptr<TeleopJointsConfig>& config) :
    Teleop<Scalar>(motionModule, config, "TeleopJoints")
  {
  }

  /**
   * @brief ~TeleopJoints Destructor
   */
  ~TeleopJoints() final {}

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
   * @brief loadExternalConfig See Behavior::loadExternalConfig()
   */
  void loadExternalConfig() final;

private:
  /**
   * Returns the cast of config to KFMDiveConfigPtr
   */
  boost::shared_ptr<TeleopJointsConfig> getBehaviorCast();

  ///< Task gain
  static Scalar jointTaskGain;

  ///< Tasks for solving inverse kinematics
  PostureTaskPtr jointTask;
};

typedef boost::shared_ptr<TeleopJoints<MType> > TeleopJointsPtr;
