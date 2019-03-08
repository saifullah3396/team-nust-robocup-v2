/**
 * @file MotionModule/MovementModule/Types/NaoqiMoveToward.h
 *
 * This file declares the class NaoqiMoveToward
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/MovementModule/MovementModule.h"

struct NaoqiMoveTowardConfig;

/**
 * @class NaoqiMoveToward
 * @brief A class for defining movement of the robot using naoqi based
 *   moveToward() that moves robot based on desired velocity
 */
template <typename Scalar>
class NaoqiMoveToward : public MovementModule<Scalar>
{
public:
  /**
   * @brief NaoqiMoveToward Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  NaoqiMoveToward(
    MotionModule* motionModule,
    const boost::shared_ptr<NaoqiMoveTowardConfig>& config);

  /**
   * @brief ~NaoqiMoveToward Destructor
   */
  ~NaoqiMoveToward() final {}
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
   * @brief getBehaviorCast Casts the config to NaoqiMoveTowardConfig
   * @return boost::shared_ptr<NaoqiMoveTowardConfig>
   */
  boost::shared_ptr<NaoqiMoveTowardConfig> getBehaviorCast();

  void setPostureAction();
  void walkAction();

  unsigned behaviorState;

  /**
   * States of this behavior
   *
   * @enum BehaviorState
   */
  enum BehaviorState
  {
    setPosture,
    walk
  };
};

typedef boost::shared_ptr<NaoqiMoveToward<MType> > NaoqiMoveTowardPtr;
