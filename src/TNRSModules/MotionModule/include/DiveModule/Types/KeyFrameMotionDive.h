/**
 * @file MotionModule/include/DiveModule/Types/KeyFrameMotionDive.h
 *
 * This file declares the class KeyFrameMotionDive
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/DiveModule/DiveModule.h"

struct KFMDiveConfig;

template <typename Scalar>
class KeyFrameMotionDive : public DiveModule<Scalar>
{
public:
  /**
   * @brief KeyFrameMotionDive Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  KeyFrameMotionDive(
    MotionModule* motionModule,
    const boost::shared_ptr<KFMDiveConfig>& config);

  /**
   * @brief ~KeyFrameMotionDive Destructor
   */
  ~KeyFrameMotionDive() final {}

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
	 * Returns the cast of config to KFMDiveConfigPtr
	 */
  boost::shared_ptr<KFMDiveConfig> getBehaviorCast();

  /**
   * @brief setupDive Sets up the dive motion request
   * @param dive Key frame states for dive
   */
  void setupDive(const Scalar dive[][25]);

  ///< Total time to dive
  Scalar diveTime = {0.0};
};

typedef boost::shared_ptr<KeyFrameMotionDive<MType> > KeyFrameMotionDivePtr;
