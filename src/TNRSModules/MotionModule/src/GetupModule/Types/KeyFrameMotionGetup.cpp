/**
 * @file MotionModule/GetupModule/Types/KeyFrameMotionGetup.h
 *
 * This file implements the class KeyFrameMotionGetup
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/MotionModule.h"
#include "BehaviorConfigs/include/MBConfigs/MBGetupConfig.h"
#include "MotionModule/include/GetupModule/Types/KeyFrameMotionGetup.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "Utils/include/HardwareIds.h"

template <typename Scalar>
KeyFrameMotionGetup<Scalar>::KeyFrameMotionGetup(
  MotionModule* motionModule,
  const boost::shared_ptr<KFMGetupConfig>& config) :
  GetupModule<Scalar>(motionModule, config, "KeyFrameMotionGetup")
{
}

template <typename Scalar>
bool KeyFrameMotionGetup<Scalar>::initiate()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  this->closeHand(RobotHands::lHand);
  this->closeHand(RobotHands::rHand);
  KeyFrameGetupTypes type = this->getBehaviorCast()->keyFrameGetupType;
  if (type == KeyFrameGetupTypes::front) {
    this->endPosture = PostureState::stand;
    this->getupTime = this->runKeyFrameMotion(getupFromFront);
    return true;
  } else if (type == KeyFrameGetupTypes::back) {
    this->endPosture = PostureState::stand;
    this->getupTime = this->runKeyFrameMotion(getupFromBack);
    return true;
  } else if (type == KeyFrameGetupTypes::sit) {
    this->endPosture = PostureState::stand;
    this->getupTime = this->runKeyFrameMotion(getupFromSit);
    return true;
  }
  #else
  LOG_ERROR("Behavior KeyFrameMotionDive undefined without Naoqi")
  finish();
  #endif
  return false;
}

template <typename Scalar>
void KeyFrameMotionGetup<Scalar>::update()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  if (this->execTime > this->getupTime + this->cycleTime / 2) {
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
void KeyFrameMotionGetup<Scalar>::finish()
{
  this->inBehavior = false;
}

template <typename Scalar>
KFMGetupConfigPtr KeyFrameMotionGetup<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <KFMGetupConfig> (this->config);
}

template class KeyFrameMotionGetup<MType>;
