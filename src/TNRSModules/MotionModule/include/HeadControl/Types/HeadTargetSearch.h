/**
 * @file MotionModule/HeadControl/Types/HeadTargetSearch.h
 *
 * This file declares the class HeadTargetSearch
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/HeadControl/HeadControl.h"

template <typename Scalar>
class HeadTargetSearch : public HeadControl<Scalar>
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  HeadTargetSearch(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config) :
    HeadControl<Scalar>(motionModule, config, "HeadTargetSearch"),
    totalWaitTime(Scalar(1.0)),
    waitTime(Scalar(0.0)),
    hyCmdResetCount(0),
    hpCmdResetCount(0)
  {
    targetType = HeadTargetTypes::ball;
    behaviorState = midScan;
    intError.setZero();
  }

  /**
   * Default destructor for this class.
   */
  ~HeadTargetSearch()
  {
  }
  
  /**
   * Derived from Behavior
   */ 
  bool initiate();
  void update();
  void finish();
  void loadExternalConfig();
private:
  /**
	 * Returns the cast of config to HeadTargetSearchConfigPtr
	 */
  HeadTargetSearchConfigPtr getBehaviorCast();  
  bool moveHeadToTarget(const Matrix<Scalar, 4, 1>& posCam);
  void scanEnv();
  
  Scalar waitTime;
  Scalar totalWaitTime; // bconfig
  HeadTargetTypes targetType; // bconfig
  Matrix<Scalar, 2, 1> intError;
  static Matrix<Scalar, 3, 1> pidGains;

  Matrix<Scalar, 2, 1> prevCmdError;
  int hyCmdResetCount;
  int hpCmdResetCount;

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

typedef boost::shared_ptr<HeadTargetSearch<MType> > HeadTargetSearchPtr;
