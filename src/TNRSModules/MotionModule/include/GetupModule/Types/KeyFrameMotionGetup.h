/**
 * @file MotionModule/GetupModule/Types/KeyFrameMotionGetup.h
 *
 * This file declares the class KeyFrameMotionGetup
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/GetupModule/GetupModule.h"

struct KFMGetupConfig;

template<typename Scalar>
class KeyFrameMotionGetup : public GetupModule<Scalar>
{
public:
  /**
   * @brief KeyFrameMotionGetup Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  KeyFrameMotionGetup(
    MotionModule* motionModule,
    const boost::shared_ptr<KFMGetupConfig>& config);

  /**
   * @brief ~KeyFrameMotionGetup Destructor
   */
  ~KeyFrameMotionGetup()
  {
  }

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
   * @brief getBehaviorCast Returns the cast of config to
   *   KFMGetupConfigPtr
	 */
  boost::shared_ptr<KFMGetupConfig> getBehaviorCast();

  /**
   * @brief setupGetupMotion Sets up the motion request
   * @param keyFrames Array of keyframe states
   */
  void setupGetupMotion(const Scalar keyFrames[][25]);

  //! Total time to get up
  Scalar getupTime;

  //! Motion execution time updated after each update
  Scalar execTime;
};

typedef boost::shared_ptr<KeyFrameMotionGetup<MType> > KeyFrameMotionGetupPtr;
