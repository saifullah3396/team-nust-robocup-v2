/**
 * @file MotionModule/src/HeadControl/Types/HeadScan.cpp
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
#include "Utils/include/PIDController.h"

template <typename Scalar>
HeadScan<Scalar>::HeadScan(
  MotionModule* motionModule,
  const boost::shared_ptr<HeadScanConfig>& config) :
  HeadControl<Scalar>(motionModule, config, "HeadScan")
{
  DEFINE_FSM_STATE(HeadScan<Scalar>, MidScan, midScan);
  DEFINE_FSM_STATE(HeadScan<Scalar>, LeftScan, leftScan);
  DEFINE_FSM_STATE(HeadScan<Scalar>, RightScan, rightScan);
  DEFINE_FSM_STATE(HeadScan<Scalar>, FinishSequence, finishSequence);
  DEFINE_FSM(fsm, HeadScan<Scalar>, midScan);
}

template <typename Scalar>
bool HeadScan<Scalar>::initiate()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  LOG_INFO("HeadScan.initiate() called...");
  this->trackersXY.resize(2);
  for (size_t i = 0; i < this->trackersXY.size(); ++i) {
    this->trackersXY[i] =
      boost::shared_ptr<PIDController<Scalar>>(
        new PIDController<Scalar>(this->cycleTime));
    this->trackersXY[i]->setPidGains(this->pidGains[i]);
  }
  return true;
  #else
  LOG_ERROR("Behavior HeadScan undefined without Naoqi")
  return false;
  #endif
}

template <typename Scalar>
void HeadScan<Scalar>::update()
{
  if (fsm->update()) {
    finish();
  }
}

template <typename Scalar>
void HeadScan<Scalar>::finish()
{
  LOG_INFO("HeadScan.finish() called...");
  if (getBehaviorCast()->resetOnKill)
    fsm->state = finishSequence.get();
  else
    this->inBehavior = false;
}

template <typename Scalar>
void HeadScan<Scalar>::setScanTarget(
  const Scalar& yaw, const Scalar& pitch)
{
  this->targetAngles[toUType(Joints::headYaw)] = yaw;
  this->targetAngles[toUType(Joints::headPitch)] = pitch;
}

template <typename Scalar>
bool HeadScan<Scalar>::trackTarget()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  Matrix<Scalar, 2, 1> meas;
  meas[0] = this->kM->getJointState(Joints::headYaw)->position();
  meas[1] = this->kM->getJointState(Joints::headPitch)->position();
  bool targetsReached = true;
  for (size_t i = 0; i < 2; ++i) { // Moving it about Y has problems, so keep it fixed
    if (this->targetAngles[i] == this->targetAngles[i]) {
      this->trackersXY[i]->setCmd(this->targetAngles[i]);
      auto input = this->trackersXY[i]->update(meas[i]);
      this->naoqiChangeAngles(this->naoqiNames[i], input, this->fractionMaxSpeed);
      if (fabsf(this->trackersXY[i]->prevError1()) > Angle::DEG_2)
        targetsReached = false;
    }
  }
  return targetsReached;
  #endif
}

template <typename Scalar>
bool HeadScan<Scalar>::waitOnTargetReached()
{
  static Scalar scanTime = 0.0;
  if (scanTime >= this->getBehaviorCast()->totalWaitTime) {
    scanTime = 0.0;
    return false;
  }
  scanTime += this->cycleTime;
  return true;
}

template <typename Scalar>
void HeadScan<Scalar>::MidScan::onStart()
{
  if (this->bPtr->getBehaviorCast()->scanLowerArea) {
    this->bPtr->setScanTarget(0.0, this->bPtr->getBehaviorCast()->scanMaxPitch);
  }
  this->bPtr->setScanTarget(0.0, NAN);
}

template <typename Scalar>
void HeadScan<Scalar>::MidScan::onRun()
{
  if (this->bPtr->trackTarget() && !this->bPtr->waitOnTargetReached())
    this->nextState = this->bPtr->leftScan.get();
}

template <typename Scalar>
void HeadScan<Scalar>::LeftScan::onStart()
{
  if (this->bPtr->getBehaviorCast()->scanLowerArea) {
    this->bPtr->setScanTarget(
      this->bPtr->getBehaviorCast()->scanMaxYaw,
      this->bPtr->getBehaviorCast()->scanMaxPitch);
  }
  this->bPtr->yawTargets.clear();
  const auto& divs = this->bPtr->getBehaviorCast()->scanYawDivs;
  const auto& maxYaw = this->bPtr->getBehaviorCast()->scanMaxYaw;
  for (int i = 1; i <= divs; ++i)
    this->bPtr->yawTargets.push_back(maxYaw * i / (float)divs);
  for (int i = divs - 1; i >= 0; --i)
    this->bPtr->yawTargets.push_back(maxYaw * i / (float)divs);
  if (!this->bPtr->yawTargets.empty())
    this->bPtr->setScanTarget(this->bPtr->yawTargets[0], NAN);
  else
    this->bPtr->finish();
}

template <typename Scalar>
void HeadScan<Scalar>::LeftScan::onRun()
{
  static unsigned currTarget = 0;
  if (this->bPtr->trackTarget() && !this->bPtr->waitOnTargetReached()) {
    currTarget++;
    if (currTarget >= this->bPtr->yawTargets.size()) {
      this->nextState = this->bPtr->rightScan.get();
      currTarget = 0;
    } else {
      this->bPtr->setScanTarget(this->bPtr->yawTargets[currTarget], NAN);
    }
  }
}

template <typename Scalar>
void HeadScan<Scalar>::RightScan::onStart()
{
  if (this->bPtr->getBehaviorCast()->scanLowerArea) {
    this->bPtr->setScanTarget(
      -this->bPtr->getBehaviorCast()->scanMaxYaw,
      this->bPtr->getBehaviorCast()->scanMaxPitch);
  }
  this->bPtr->yawTargets.clear();
  const auto& divs = this->bPtr->getBehaviorCast()->scanYawDivs;
  const auto& maxYaw = -this->bPtr->getBehaviorCast()->scanMaxYaw;
  for (int i = 1; i <= divs; ++i)
    this->bPtr->yawTargets.push_back(maxYaw * i / (float)divs);
  for (int i = divs - 1; i >= 0; --i)
    this->bPtr->yawTargets.push_back(maxYaw * i / (float)divs);
  if (!this->bPtr->yawTargets.empty())
    this->bPtr->setScanTarget(this->bPtr->yawTargets[0], NAN);
  else
    this->bPtr->finish();
}

template <typename Scalar>
void HeadScan<Scalar>::RightScan::onRun()
{
  static unsigned currTarget = 0;
  if (this->bPtr->trackTarget() && !this->bPtr->waitOnTargetReached()) {
    currTarget++;
    if (currTarget >= this->bPtr->yawTargets.size()) {
      this->nextState = nullptr;
      currTarget = 0;
    } else {
      this->bPtr->setScanTarget(this->bPtr->yawTargets[currTarget], NAN);
    }
  }
}

template <typename Scalar>
void HeadScan<Scalar>::FinishSequence::onStart()
{
  if (this->bPtr->getBehaviorCast()->scanLowerArea) {
    this->bPtr->setScanTarget(
      -this->bPtr->getBehaviorCast()->scanMaxYaw,
      this->bPtr->getBehaviorCast()->scanMaxPitch);
  }
  this->bPtr->setScanTarget(0.0, NAN);
}

template <typename Scalar>
void HeadScan<Scalar>::FinishSequence::onRun()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  if (this->bPtr->trackTarget() && !this->bPtr->waitOnTargetReached()) {
    this->bPtr->inBehavior = false;
  }
  #endif
}

template <typename Scalar>
boost::shared_ptr<HeadScanConfig> HeadScan<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<HeadScanConfig> (this->config);
}

template class HeadScan<MType>;
