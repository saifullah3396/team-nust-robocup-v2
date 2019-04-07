/**
 * @file MotionModule/include/MotionPlayback/Types/ReplayStoredMB.h
 *
 * This file declares the class ReplayStoredMB
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/MotionPlayback/MotionPlayback.h"

struct ReplayStoredMBConfig;

template <typename Scalar>
class ReplayStoredMB : public MotionPlayback<Scalar>
{
public:
  /**
   * @brief ReplayStoredMB Constructor
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  ReplayStoredMB(
    MotionModule* motionModule,
    const boost::shared_ptr<ReplayStoredMBConfig>& config) :
    MotionPlayback<Scalar>(motionModule, config, "ReplayStoredMB"),
    step(0)
  {
  }

  /**
   * @brief ~ReplayStoredMB Destructor
   */
  ~ReplayStoredMB() final {}
  
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
	 * Returns the cast of config to KFMDiveConfigPtr
	 */
  boost::shared_ptr<ReplayStoredMBConfig> getBehaviorCast();

  ///< Total motion playback time
  Scalar mbTime;

  ///< Saved joint position commands
  Matrix<Scalar, Dynamic, Dynamic> jointCmds;

  ///< Saved joint command relative times
  Matrix<Scalar, Dynamic, 1> jointTimes;

  ///< Recorded command step
  unsigned step;
};

typedef boost::shared_ptr<ReplayStoredMB<MType> > ReplayStoredMBPtr;
