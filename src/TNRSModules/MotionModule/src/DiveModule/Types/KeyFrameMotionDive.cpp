/**
 * @file MotionModule/src/DiveModule/Types/KeyFrameMotionDive.cpp
 *
 * This file implements the class KeyFrameMotionDive
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "BehaviorConfigs/include/MBConfigs/MBDiveConfig.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "MotionModule/include/DiveModule/Types/KeyFrameMotionDive.h"
#include "MotionModule/include/MotionGenerator.h"
#include "Utils/include/DataHolders/PostureState.h"

template <typename Scalar>
KeyFrameMotionDive<Scalar>::KeyFrameMotionDive(
  MotionModule* motionModule,
  const boost::shared_ptr<KFMDiveConfig>& config) :
DiveModule<Scalar>(motionModule, config, "KeyFrameMotionDive")
{
}

template <typename Scalar>
bool KeyFrameMotionDive<Scalar>::initiate()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  LOG_INFO("KeyFrameMotionDive.initiate() called...")
  this->mG->closeHand(RobotHands::lHand);
  this->mG->closeHand(RobotHands::rHand);
  KeyFrameDiveTypes type = getBehaviorCast()->keyFrameDiveType;
  if (type == KeyFrameDiveTypes::inPlace) {
    this->endPosture = PostureState::diveInPlace;
    this->diveTime = this->runKeyFrameMotion(diveInPlace);
    return true;
  } else if (type == KeyFrameDiveTypes::sumo) {
    this->endPosture = PostureState::diveSumo;
    this->diveTime = this->runKeyFrameMotion(diveSumo);
    return true;
  } else if (type == KeyFrameDiveTypes::left) {
    this->endPosture = PostureState::diveLeft;
    this->diveTime = this->runKeyFrameMotion(diveLeft);
    return true;
  } else if (type == KeyFrameDiveTypes::right) {
    this->endPosture = PostureState::diveRight;
    this->diveTime = this->runKeyFrameMotion(diveRight);
    return true;
  }
  this->execTime = 0.0;
  #else
  LOG_ERROR("Behavior KeyFrameMotionDive undefined without Naoqi");
  finish();
  #endif
  return false;
}

template <typename Scalar>
void KeyFrameMotionDive<Scalar>::update()
{
   #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  //LOG_INFO("KeyFrameMotionDive.update()")
  if (this->execTime > this->diveTime) {
    POSTURE_STATE_OUT(MotionModule) = this->endPosture;
    finish();
  } else {
    this->execTime += this->cycleTime;
  }
  #else
  LOG_ERROR("Behavior KeyFrameMotionDive undefined without Naoqi")
  finish();
  #endif
}

template <typename Scalar>
void KeyFrameMotionDive<Scalar>::finish()
{
  LOG_INFO("KeyFrameMotionDive.finish() called...")
  this->inBehavior = false;
}

template <typename Scalar>
boost::shared_ptr<KFMDiveConfig> KeyFrameMotionDive<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<KFMDiveConfig> (this->config);
}

template class KeyFrameMotionDive<MType>;
