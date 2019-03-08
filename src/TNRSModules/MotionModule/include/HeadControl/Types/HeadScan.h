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

template <typename Scalar>
class HeadScan : public HeadControl<Scalar>
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  HeadScan(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config) :
    HeadControl<Scalar>(motionModule, config, "HeadScan"),
    totalWaitTime(Scalar(1.0)),
    waitTime(Scalar(0.0))
  {
    behaviorState = midScan;
  }

  /**
   * Default destructor for this class.
   */
  ~HeadScan()
  {
  }

  /**
   * Derived from Behavior
   */
  bool initiate();
  void update();
  void finish();
private:
  /**
   * Returns the cast of config to HeadScanConfigPtr
   */
  HeadScanConfigPtr getBehaviorCast();
  void scanEnv();

  Scalar waitTime;
  Scalar totalWaitTime; // bconfig
  HeadTargetTypes targetType; // bconfig

  unsigned behaviorState;
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
