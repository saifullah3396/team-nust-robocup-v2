/**
 * @file MotionModule/DiveModule/Types/KeyFrameMotionDive.h
 *
 * This file implements the class KeyFrameMotionDive
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "TNRSBase/include/MemoryIOMacros.h"
#include "MotionModule/include/DiveModule/Types/KeyFrameMotionDive.h"
#include "MotionModule/include/MotionGenerator.h"
#include "Utils/include/DataHolders/PostureState.h"

template <typename Scalar>
KFMDiveConfigPtr KeyFrameMotionDive<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<KFMDiveConfig> (this->config);
}

template <typename Scalar>
void KeyFrameMotionDive<Scalar>::initiate()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  LOG_INFO("KeyFrameMotionDive.initiate() called...")
  this->mG->closeHand(L_HAND);
  this->mG->closeHand(R_HAND);
  KeyFrameDiveTypes type = getBehaviorCast()->keyFrameDiveType;
  if (type == KeyFrameDiveTypes::IN_PLACE) {
    this->endPosture = PostureState::DIVE_IN_PLACE;
    this->diveTime = this->runKeyFrameMotion(diveInPlace);
    this->inBehavior = true;
  } else if (type == KeyFrameDiveTypes::SUMO) {
    this->endPosture = PostureState::DIVE_SUMO;
    this->diveTime = this->runKeyFrameMotion(diveSumo);
    this->inBehavior = true;
  } else if (type == KeyFrameDiveTypes::LEFT) {
    this->endPosture = PostureState::DIVE_LEFT;
    this->diveTime = this->runKeyFrameMotion(diveLeft);
    this->inBehavior = true;
  } else if (type == KeyFrameDiveTypes::RIGHT) {
    this->endPosture = PostureState::DIVE_RIGHT;
    this->diveTime = this->runKeyFrameMotion(diveRight);
    this->inBehavior = true;
  }
  this->execTime = 0.0;
  #else
  LOG_ERROR("Behavior KeyFrameMotionDive undefined without Naoqi")
  finish();
  #endif
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

template class KeyFrameMotionDive<MType>;
