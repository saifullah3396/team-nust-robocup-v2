/**
 * @file MotionModule/HeadControl/Types/HeadTargetTrack.h
 *
 * This file declares the class HeadTargetTrack
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/HeadControl/HeadControl.h"

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

  /**
   * @brief loadExternalConfig See Behavior::loadExternalConfig()
   */
  void loadExternalConfig() final;
  
private:
  /**
   * @brief getBehaviorCast Returns the casted config
   * @return boost::shared_ptr<HeadTargetSearchConfig>
   */
  boost::shared_ptr<HeadTargetTrackConfig> getBehaviorCast();

  void followTarget(const Matrix<Scalar, 4, 1>& posCam);
  
  static Matrix<Scalar, 3, 1> pidGains;
  Matrix<Scalar, 2, 1> intError = {Matrix<Scalar, 2, 1>::Zero()};
  Matrix<Scalar, 2, 1> prevCommand = {Matrix<Scalar, 2, 1>::Zero()};
  Matrix<Scalar, 2, 1> errorK1 = {Matrix<Scalar, 2, 1>::Zero()};
  Matrix<Scalar, 2, 1> errorK2 = {Matrix<Scalar, 2, 1>::Zero()};
  
  HeadTargetTypes targetType; // bconfig
  Scalar targetLostTime = {0.0};
};

typedef boost::shared_ptr<HeadTargetTrack<MType> > HeadTargetTrackPtr;
