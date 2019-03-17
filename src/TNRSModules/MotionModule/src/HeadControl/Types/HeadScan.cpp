/**
 * @file MotionModule/HeadControl/Types/HeadScan.h
 *
 * This file implements the class HeadScan
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "BehaviorManager/include/StateMachine.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/HeadControl/Types/HeadScan.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/AngleDefinitions.h"

template <typename Scalar>
HeadScan<Scalar>::HeadScan(
  MotionModule* motionModule,
  const boost::shared_ptr<HeadScanConfig>& config) :
  HeadControl<Scalar>(motionModule, config, "HeadScan")
{
  DEFINE_FSM_STATE(HeadScan<Scalar>, MidScan, midScan);
  DEFINE_FSM_STATE(HeadScan<Scalar>, LeftScan, leftScan);
  DEFINE_FSM_STATE(HeadScan<Scalar>, RightScan, rightScan);
  DEFINE_FSM(fsm, HeadScan<Scalar>, midScan);
}

template <typename Scalar>
bool HeadScan<Scalar>::initiate()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  LOG_INFO("HeadScan.initiate() called...");
  return true;
  #else
  LOG_ERROR("Behavior HeadScan undefined without Naoqi")
  return false;
  #endif
}

template <typename Scalar>
void HeadScan<Scalar>::update()
{
  if (fsm->update())
    return;
}

template <typename Scalar>
void HeadScan<Scalar>::finish()
{
  LOG_INFO("HeadScan.finish() called...");
  this->inBehavior = false;
}

template <typename Scalar>
void HeadScan<Scalar>::setScanTarget(
  const Scalar& yaw, const Scalar& pitch)
{
  this->targetAngles[toUType(Joints::headYaw)] = yaw;
  this->targetAngles[toUType(Joints::headPitch)] = pitch;
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  this->naoqiSetAngles(this->naoqiNames, this->targetAngles, this->fractionMaxSpeed);
  #endif
}

template <typename Scalar>
bool HeadScan<Scalar>::waitOnTargetReached()
{
  static Scalar scanTime = 0.0;
  auto& headYaw = this->kM->getJointState(Joints::headYaw)->position();
  if (getBehaviorCast()->scanLowerArea) {
    auto& headPitch = this->kM->getJointState(Joints::headPitch)->position();
    if (fabsf(headYaw - this->targetAngles[toUType(Joints::headYaw)]) < Angle::DEG_2 &&
        fabsf(headPitch - this->targetAngles[toUType(Joints::headPitch)]) < Angle::DEG_2)
    {
      if (scanTime >= this->getBehaviorCast()->totalWaitTime) {
        return false;
      }
      scanTime += this->cycleTime;
    }
  } else {
    if (fabsf(headYaw - this->targetAngles[toUType(Joints::headYaw)]) < Angle::DEG_2) {
      scanTime += this->cycleTime;
      if (scanTime >= this->getBehaviorCast()->totalWaitTime) {
        return false;
      }
      scanTime += this->cycleTime;
    }
  }
  return true;
}

template <typename Scalar>
void HeadScan<Scalar>::MidScan::onStart()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  if (this->bPtr->getBehaviorCast()->scanLowerArea) {
    this->bPtr->setScanTarget(0.0, this->bPtr->getBehaviorCast()->scanMaxPitch);
  }
  this->bPtr->setScanTarget(0.0, NAN);
  #endif
}

template <typename Scalar>
void HeadScan<Scalar>::MidScan::onRun()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  if (!this->bPtr->waitOnTargetReached())
    this->nextState = this->bPtr->leftScan.get();
  #endif
}

template <typename Scalar>
void HeadScan<Scalar>::LeftScan::onStart()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  if (this->bPtr->getBehaviorCast()->scanLowerArea) {
    this->bPtr->setScanTarget(
      this->bPtr->getBehaviorCast()->scanMaxYaw,
      this->bPtr->getBehaviorCast()->scanMaxPitch);
  }
  this->bPtr->setScanTarget(
    this->bPtr->getBehaviorCast()->scanMaxYaw, NAN);
  #endif
}

template <typename Scalar>
void HeadScan<Scalar>::LeftScan::onRun()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  if (!this->bPtr->waitOnTargetReached())
    this->nextState = this->bPtr->rightScan.get();
  #endif
}

template <typename Scalar>
void HeadScan<Scalar>::RightScan::onStart()
{
  if (this->bPtr->getBehaviorCast()->scanLowerArea) {
    this->bPtr->setScanTarget(
      -this->bPtr->getBehaviorCast()->scanMaxYaw,
      this->bPtr->getBehaviorCast()->scanMaxPitch);
  }
  this->bPtr->setScanTarget(
    -this->bPtr->getBehaviorCast()->scanMaxYaw, NAN);
}

template <typename Scalar>
void HeadScan<Scalar>::RightScan::onRun()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  if (!this->bPtr->waitOnTargetReached())
    this->nextState = this->bPtr->midScan.get();
  #endif
}

template <typename Scalar>
boost::shared_ptr<HeadScanConfig> HeadScan<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<HeadScanConfig> (this->config);
}

template class HeadScan<MType>;
