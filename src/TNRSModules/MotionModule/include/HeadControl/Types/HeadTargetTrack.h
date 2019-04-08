/**
 * @file MotionModule/include/HeadControl/Types/HeadTargetTrack.h
 *
 * This file declares the class HeadTargetTrack
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/HeadControl/HeadControl.h"

struct HeadScanConfig;
struct HeadTargetTrackConfig;

template <typename Scalar>
class HeadTargetTrack : public HeadControl<Scalar>
{
public:
  /**
   * @brief HeadTargetTrack Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  HeadTargetTrack(
    MotionModule* motionModule,
    const boost::shared_ptr<HeadTargetTrackConfig>& config);

  /**
   * @brief ~HeadTargetTrack Destructor
   */
  ~HeadTargetTrack() final {}

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
   * @brief getBehaviorCast Returns the casted config
   * @return boost::shared_ptr<HeadTargetTrackConfig>
   */
  boost::shared_ptr<HeadTargetTrackConfig> getBehaviorCast();

  /**
   * @brief moveHeadToTarget Moves the head to target if found
   * @param posCam Position of target in camera frame
   * @return True on success
   */
  bool trackTarget(const Matrix<Scalar, 4, 1>& posCam);
};
