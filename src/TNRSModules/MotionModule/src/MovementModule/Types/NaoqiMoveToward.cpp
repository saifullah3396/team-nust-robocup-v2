/**
 * @file MotionModule/MovementModule/Types/NaoqiMoveToward.cpp
 *
 * This file implements the class NaoqiMoveToward
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MovementModule/Types/NaoqiMoveToward.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/Behaviors/MBConfigs/MBMovementConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBPostureConfig.h"

template <typename Scalar>
NaoqiMoveToward<Scalar>::NaoqiMoveToward(
  MotionModule* motionModule,
  const boost::shared_ptr<NaoqiMoveTowardConfig>& config) :
  MovementModule<Scalar>(motionModule, config, "NaoqiMoveToward")
{
}

template <typename Scalar>
void NaoqiMoveToward<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    //! read parameters from config file:
    //GET_CONFIG(
    //  "MotionBehaviors",
    //  (Scalar, Any variable, Container for that variable),
    //);
    loaded = true;
  }
}

template <typename Scalar>
bool NaoqiMoveToward<Scalar>::initiate()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  LOG_INFO("NaoqiMoveToward.initiate() called...")
  ROBOT_IN_MOTION_OUT(MotionModule) = false;
  if (getBehaviorCast()->startPosture)
    behaviorState = setPosture;
  else
    behaviorState = walk;
  return true;
  #else
  LOG_INFO("NaoqiMoveToward is undefined without naoqi-motion-proxy...")
  #endif
}

template <typename Scalar>
void NaoqiMoveToward<Scalar>::reinitiate(const BehaviorConfigPtr& cfg)
{
  LOG_INFO("NaoqiMoveToward.reinitiate() called...")
  this->config = cfg;
}

template <typename Scalar>
void NaoqiMoveToward<Scalar>::update()
{
  if (behaviorState == setPosture) {
    //cout << "NaoqiMoveToward setPosture" << endl;
    setPostureAction();
  } else if (behaviorState == walk) {
    //cout << "NaoqiMoveToward setupWalk" << endl;
    walkAction();
  }
}

template <typename Scalar>
boost::shared_ptr<NaoqiMoveTowardConfig> NaoqiMoveToward<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <NaoqiMoveTowardConfig> (this->config);
}

template <typename Scalar>
void NaoqiMoveToward<Scalar>::setPostureAction() {
  if (getBehaviorCast()->startPosture) {
    // if posture is not equal to desired one
    if (getBehaviorCast()->startPosture->targetPosture !=
        POSTURE_STATE_OUT(MotionModule))
    {
      this->setupChildRequest(getBehaviorCast()->startPosture);
    }
  }
  behaviorState = walk;
}

template <typename Scalar>
void NaoqiMoveToward<Scalar>::walkAction() {
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  auto& vel = getBehaviorCast()->velocityInput;
  this->naoqiMoveToward(vel.getX(), vel.getY(), vel.getTheta());
  ROBOT_IN_MOTION_OUT(MotionModule) = true;
  #endif
}

template <typename Scalar>
void NaoqiMoveToward<Scalar>::finish()
{
  LOG_INFO("NaoqiMoveToward.finish() called...")
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  this->stopMove();
  #endif
  ROBOT_IN_MOTION_OUT(MotionModule) = false;
  this->inBehavior = false;
}

template class NaoqiMoveToward<MType>;

