/**
 * @file MotionModule/HeadControl/Types/HeadScan.h
 *
 * This file implements the class HeadScan
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/HeadControl/Types/HeadScan.h"
#include "TNRSBase/include/MemoryIOMacros.h"

#define TARGET_TOL 0.043631944 // radians in 2.5deg

template <typename Scalar>
HeadScanConfigPtr HeadScan<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<HeadScanConfig> (this->config);
}

template <typename Scalar>
void HeadScan<Scalar>::initiate()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  LOG_INFO("HeadScan.initiate() called...");
  this->totalWaitTime = this->getBehaviorCast()->totalWaitTime;
  this->inBehavior = true;
  #else
  LOG_ERROR("Behavior HeadScan undefined without Naoqi")
  finish();
  #endif
}

template <typename Scalar>
void HeadScan<Scalar>::update()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  scanEnv();
  #else
  LOG_ERROR("Behavior HeadScan undefined without Naoqi")
  finish();
  #endif
}

template <typename Scalar>
void HeadScan<Scalar>::finish()
{
  LOG_INFO("HeadScan.finish() called...");
  this->inBehavior = false;
}

template <typename Scalar>
void HeadScan<Scalar>::scanEnv()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  Scalar leftScanMax = getBehaviorCast()->scanMaxYaw;
  Scalar rightScanMax = -getBehaviorCast()->scanMaxYaw;
  Scalar headYaw = this->kM->getJointState(HEAD_YAW)->position();
  Scalar headPitch = this->kM->getJointState(HEAD_PITCH)->position();
  AL::ALValue nameYaw = AL::ALValue::array("HeadYaw");
  AL::ALValue angleYaw = AL::ALValue::array(0.f);
  AL::ALValue namePitch = AL::ALValue::array("HeadPitch");
  AL::ALValue anglePitch = AL::ALValue::array(0.f);
  Scalar fractionMaxSpeed = 0.1f;
  if (getBehaviorCast()->scanLowerArea) {
    anglePitch[0] = getBehaviorCast()->scanMaxPitch;
    this->naoqiSetAngles(namePitch, anglePitch, fractionMaxSpeed);
  }
  if (behaviorState == midScan) {
    angleYaw[0] = 0.f;
    this->naoqiSetAngles(nameYaw, angleYaw, fractionMaxSpeed);
    behaviorState = midWait;
    waitTime = 0;
  } else if (behaviorState == midWait) {
    //cout << "Mid wait..." << endl;
    //cout << "MidWaitTime: " << waitTime << endl;
    if (abs(headYaw) < TARGET_TOL) {
      waitTime += this->cycleTime;
      if (waitTime > totalWaitTime) behaviorState = leftScan;
    }
  } else if (behaviorState == leftScan) {
    //cout << "Left scan..." << endl;
    angleYaw[0] = leftScanMax;
    this->naoqiSetAngles(nameYaw, angleYaw, fractionMaxSpeed);
    behaviorState = leftWait;
    waitTime = 0;
  } else if (behaviorState == leftWait) {
    //cout << "Left wait..." << endl;
    //cout << "LeftWaitTime: " << waitTime << endl;
    if (abs(headYaw - leftScanMax) < TARGET_TOL) {
      waitTime += this->cycleTime;
      if (waitTime > totalWaitTime) behaviorState = rightScan;
    }
  } else if (behaviorState == rightScan) {
    //cout << "Right scan..." << endl;
    angleYaw[0] = rightScanMax;
    this->naoqiSetAngles(nameYaw, angleYaw, fractionMaxSpeed);
    behaviorState = rightWait;
    waitTime = 0;
  } else if (behaviorState == rightWait) {
    //cout << "Right wait..." << endl;
    //cout << "RightWaitTime: " << waitTime << endl;
    if (abs(headYaw - rightScanMax) < TARGET_TOL) {
      waitTime += this->cycleTime;
      if (waitTime > totalWaitTime) {
        // Set head yaw back to zero
        angleYaw[0] = 0.f;
        this->naoqiSetAngles(nameYaw, angleYaw, fractionMaxSpeed);
        behaviorState = finishState;
      }
    }
  } else if (behaviorState == finishState) {
    //cout << "finish state..." << endl;
    //cout << "abs(headYaw): " << abs(headYaw) << endl;
    if (abs(headYaw) < TARGET_TOL)
    {
      finish();
    }
  }
  #endif
}

template class HeadScan<MType>;
