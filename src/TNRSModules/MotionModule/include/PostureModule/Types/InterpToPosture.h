/**
 * @file MotionModule/include/PostureModule/Types/InterpToPosture.h
 *
 * This file declares the class InterpToPosture
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/PostureModule/PostureModule.h"

struct InterpToPostureConfig;

/**
 * @class InterpToPosture
 * @brief A class that interpolates the joints from initial state to
 *   final state based on quintic splines with zero initial and final
 *   velocities and accelerations
 */ 
template <typename Scalar>
class InterpToPosture : public PostureModule<Scalar>
{
public:
  /**
   * @brief InterpToPosture Constructor
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  InterpToPosture(
    MotionModule* motionModule,
    const boost::shared_ptr<InterpToPostureConfig>& config);

  /**
   * @brief ~InterpToPosture Destructor
   */
  ~InterpToPosture() final {}
  
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
  void loadExternalConfig() final {}

private:
  /**
   * Returns the cast of config to MBPostureConfigPtr
   */
  boost::shared_ptr<InterpToPostureConfig> getBehaviorCast();

  //! Joint configuration initial
  Matrix<Scalar, Dynamic, 1> jointsI;

  //! Joint configuration difference
  Matrix<Scalar, Dynamic, 1> jointsDelta;
};
typedef boost::shared_ptr<InterpToPosture<MType> > InterpToPosturePtr;
