/**
 * @file MotionModule/HeadControl/Types/HeadScan.h
 *
 * This file declares the class HeadScan
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/HeadControl/HeadControl.h"

struct HeadScanConfig;

template <typename Scalar>
class HeadScan : public HeadControl<Scalar>
{
public:
  /**
   * @brief HeadScan Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  HeadScan(
    MotionModule* motionModule,
    const boost::shared_ptr<HeadScanConfig>& config);

  /**
   * @brief ~HeadScan Destructor
   */
  ~HeadScan() final {}

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
   * Returns the cast of config to HeadScanConfigPtr
   */
  /**
   * @brief getBehaviorCast Returns the casted config
   * @return boost::shared_ptr<HeadScanConfig>
   */
  boost::shared_ptr<HeadScanConfig> getBehaviorCast();

  /**
   * @brief scanEnv Scans the environment
   */
  void scanEnv();

  Scalar waitTime = {0.0};

  unsigned behaviorState = {midScan};
  enum behaviorState {
    midScan = 0,
    midWait,
    leftScan,
    leftWait,
    rightScan,
    rightWait,
    finishState
  };
};

typedef boost::shared_ptr<HeadScan<MType> > HeadScanPtr;
