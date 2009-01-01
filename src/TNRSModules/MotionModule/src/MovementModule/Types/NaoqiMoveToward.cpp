/**
 * @file MotionModule/src/MovementModule/Types/NaoqiMoveToward.cpp
 *
 * This file implements the class NaoqiMoveToward
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MovementModule/Types/NaoqiMoveToward.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "Utils/include/Constants.h"

template <typename Scalar>
NaoqiMoveToward<Scalar>::NaoqiMoveToward(
  MotionModule* motionModule,
  const boost::shared_ptr<NaoqiMoveTowardConfig>& config) :
  MovementModule<Scalar>(motionModule, config, "NaoqiMoveToward")
{
  moveConfig = this->getMoveConfig();
  //moveConfig.arrayPush(AL::ALValue::array("Frequency", frequency));
  //this->setMoveConfig(moveConfig);
  //cout << "moveConfig" << moveConfig << endl;
}

template <typename Scalar>
void NaoqiMoveToward<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    ///< read parameters from config file:
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
  //LOG_INFO("NaoqiMoveToward.reinitiate() called...")
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
    if (SPC(InterpToPostureConfig, getBehaviorCast()->startPosture)->targetPosture !=
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
  auto vel = getBehaviorCast()->velocityInput;
  this->naoqiMoveToward(vel.getX(), vel.getY(), vel.getTheta());
  cout << "OdomPose:" << this->getRobotPosition(true) << endl;
  if (lastOdomTrans.norm() <= 1e-5) { // Zero
    auto odomPose = this->getRobotPosition(true);
    if (odomPose[0] == odomPose[0]) {
      Matrix<Scalar, 3, 3> odomRot;
      MathsUtils::makeRotationZ(odomRot, odomPose[2]);
      lastOdomTrans = MathsUtils::makeTransformation(
        odomRot, odomPose[0], odomPose[1], 0.0);
    }
  } else {
    auto odomPose = this->getRobotPosition(true);
    if (odomPose[0] == odomPose[0]) {
      Matrix<Scalar, 3, 3> odomRot;
      MathsUtils::makeRotationZ(odomRot, odomPose[2]);
      Matrix<Scalar, 4, 4> odomTrans = MathsUtils::makeTransformation(
        odomRot, odomPose[0], odomPose[1], 0.0);
      Matrix<Scalar, 4, 4> trans =
        MathsUtils::getTInverse(lastOdomTrans) * odomTrans;
      auto pi =
        PositionInput<float>(
          trans(0, 3),
          trans(1, 3),
          MathsUtils::matToEuler((Matrix<Scalar, 3, 3>) trans.block(0, 0, 3, 3))[2]);
      cout << "pi: " << pi.get().transpose() << endl;
      PositionUpdatePtr vu =
        boost::shared_ptr<PositionUpdate>(new PositionUpdate(pi));
      BaseModule::publishModuleRequest(vu);
      lastOdomTrans = odomTrans;
    }
  }
  ROBOT_IN_MOTION_OUT(MotionModule) = true;
  //auto stepPeriod = (maxStepPeriod - minStepPeriod) * (1.0 - frequency) + minStepPeriod;
  //vel.x() *= static_cast<float>(moveConfig[0][1]) / stepPeriod * this->cycleTime;
  //vel.y() *= (static_cast<float>(moveConfig[1][1]) - 0.1) / stepPeriod * this->cycleTime;
  //vel.theta() *= static_cast<float>(moveConfig[2][1]) / stepPeriod * this->cycleTime;
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

