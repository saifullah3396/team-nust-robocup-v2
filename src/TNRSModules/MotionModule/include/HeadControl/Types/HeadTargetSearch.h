/**
 * @file MotionModule/HeadControl/Types/HeadTargetSearch.h
 *
 * This file declares the class HeadTargetSearch
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/HeadControl/HeadControl.h"

template <typename Scalar>
class HeadTargetSearch : public HeadControl<Scalar>
{
public:
  /**
   * @brief HeadTargetSearch Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  HeadTargetSearch(
    MotionModule* motionModule,
    const boost::shared_ptr<HeadTargetSearchConfig>& config);

  /**
   * @brief ~HeadTargetSearch Destructor
   */
  ~HeadTargetSearch() final {}

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
   * @brief getBehaviorCast Returns the casted config
   * @return boost::shared_ptr<HeadTargetSearchConfig>
   */
  boost::shared_ptr<HeadTargetSearchConfig> getBehaviorCast();

  /**
   * @brief moveHeadToTarget Moves the head to target if found
   * @param posCam Position of target in camera frame
   * @return True on success
   */
  bool moveHeadToTarget(const Matrix<Scalar, 4, 1>& posCam);

  /**
   * @brief scanEnv Scans the environment
   */
  void scanEnv();

  Scalar waitTime = {0.0};
  Scalar totalWaitTime = {1.0}; // bconfig
  HeadTargetTypes targetType; // bconfig
  Matrix<Scalar, 2, 1> intError = {Matrix<Scalar, 2, 1>::Zero()};
  static Matrix<Scalar, 3, 1> pidGains;

  Matrix<Scalar, 2, 1> prevCmdError;
  int hyCmdResetCount = {0};
  int hpCmdResetCount = {0};

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
