/**
 * @file MotionModule/src/MovementModule/Types/NaoqiMoveTo.cpp
 *
 * This file implements the class NaoqiMoveTo
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MovementModule/Types/NaoqiMoveTo.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "Utils/include/Constants.h"

template <typename Scalar>
NaoqiMoveTo<Scalar>::NaoqiMoveTo(
  MotionModule* motionModule,
  const boost::shared_ptr<NaoqiMoveToConfig>& config) :
  MovementModule<Scalar>(motionModule, config, "NaoqiMoveTo")
{
  DEFINE_FSM_STATE(NaoqiMoveTo<Scalar>, SetPosture, setPosture);
  DEFINE_FSM_STATE(NaoqiMoveTo<Scalar>, MoveTo, moveTo);
  DEFINE_FSM(fsm, NaoqiMoveTo<Scalar>, setPosture);
  moveConfig = this->getMoveConfig();
  //moveConfig.arrayPush(AL::ALValue::array("Frequency", frequency));
  //this->setMoveConfig(moveConfig);
  //cout << "moveConfig" << moveConfig << endl;
}

template <typename Scalar>
void NaoqiMoveTo<Scalar>::loadExternalConfig()
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
bool NaoqiMoveTo<Scalar>::initiate()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  LOG_INFO("NaoqiMoveTo.initiate() called...")
  ROBOT_IN_MOTION_OUT(MotionModule) = false;
  if (getBehaviorCast()->startPosture)
    fsm->state = setPosture.get();
  else
    fsm->state = moveTo.get();
  while (this->naoqiMoveIsActive()) {
    this->stopMove();
  }
  return true;
  #else
  LOG_INFO("NaoqiMoveTo is undefined without naoqi-motion-proxy...")
  #endif
}

template <typename Scalar>
void NaoqiMoveTo<Scalar>::update()
{
  if (fsm->update())
    finish();
}

template <typename Scalar>
boost::shared_ptr<NaoqiMoveToConfig> NaoqiMoveTo<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <NaoqiMoveToConfig> (this->config);
}

template <typename Scalar>
void NaoqiMoveTo<Scalar>::SetPosture::onStart() {
  if (this->bPtr->getBehaviorCast()->startPosture) {
    // if posture is not equal to desired one
    if (SPC(InterpToPostureConfig, this->bPtr->getBehaviorCast()->startPosture)->targetPosture !=
        POSTURE_STATE_OUT_REL(MotionModule, this->bPtr))
    {
      this->bPtr->setupChildRequest(this->bPtr->getBehaviorCast()->startPosture);
    }
  }
}

template <typename Scalar>
void NaoqiMoveTo<Scalar>::SetPosture::onRun() {
  this->nextState = this->bPtr->moveTo.get();
}

template <typename Scalar>
void NaoqiMoveTo<Scalar>::MoveTo::onStart() {
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  auto& goal = this->bPtr->getBehaviorCast()->goal;
  this->bPtr->naoqiMoveTo(goal.getX(), goal.getY(), goal.getTheta());
  ROBOT_IN_MOTION_OUT_REL(MotionModule, this->bPtr) = true;
  #endif
}

template <typename Scalar>
void NaoqiMoveTo<Scalar>::MoveTo::onRun() {
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  if (this->bPtr->lastOdomTrans.norm() <= 1e-5) { // Zero
    auto odomPose = this->bPtr->getRobotPosition(true);
    if (odomPose[0] == odomPose[0]) {
      Matrix<Scalar, 3, 3> odomRot;
      MathsUtils::makeRotationZ(odomRot, odomPose[2]);
      this->bPtr->lastOdomTrans = MathsUtils::makeTransformation(
        odomRot, odomPose[0], odomPose[1], 0.0);
    }
  } else {
    auto odomPose = this->bPtr->getRobotPosition(true);
    if (odomPose[0] == odomPose[0]) {
      Matrix<Scalar, 3, 3> odomRot;
      MathsUtils::makeRotationZ(odomRot, odomPose[2]);
      Matrix<Scalar, 4, 4> odomTrans = MathsUtils::makeTransformation(
        odomRot, odomPose[0], odomPose[1], 0.0);
      Matrix<Scalar, 4, 4> trans =
        MathsUtils::getTInverse(this->bPtr->lastOdomTrans) * odomTrans;
      auto pi =
        PositionInput<float>(
          trans(0, 3),
          trans(1, 3),
          MathsUtils::matToEuler((Matrix<Scalar, 3, 3>) trans.block(0, 0, 3, 3))[2]);
      PositionUpdatePtr vu =
        boost::shared_ptr<PositionUpdate>(new PositionUpdate(pi));
      BaseModule::publishModuleRequest(vu);
      this->bPtr->lastOdomTrans = odomTrans;
    }
  }
  if (!this->bPtr->naoqiMoveIsActive())
    this->nextState = nullptr;
  #endif
}

template <typename Scalar>
void NaoqiMoveTo<Scalar>::finish()
{
  LOG_INFO("NaoqiMoveTo.finish() called...")
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  this->stopMove();
  #endif
  ROBOT_IN_MOTION_OUT(MotionModule) = false;
  this->inBehavior = false;
}

template class NaoqiMoveTo<MType>;

