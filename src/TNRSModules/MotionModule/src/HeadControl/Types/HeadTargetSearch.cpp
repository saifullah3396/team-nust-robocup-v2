/**
 * @file MotionModule/HeadControl/Types/HeadTargetSearch.h
 *
 * This file implements the class HeadTargetSearch
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/HeadControl/Types/HeadTargetSearch.h"
#include "TNRSBase/include/MemoryIOMacros.h"

#define TARGET_TOL 0.043631944 // radians in 2.5deg

template <typename Scalar>
Matrix<Scalar, 3, 1> HeadTargetSearch<Scalar>::pidGains;

template <typename Scalar>
HeadTargetSearchConfigPtr HeadTargetSearch<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<HeadTargetSearchConfig> (this->config);
}

template <typename Scalar>
bool HeadTargetSearch<Scalar>::initiate()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  LOG_INFO("HeadTargetSearch.initiate() called...");
  this->totalWaitTime = this->getBehaviorCast()->totalWaitTime;
  this->targetType = getBehaviorCast()->headTargetType;
  hyCmdResetCount = 0;
  hpCmdResetCount = 0;
  return true;
  #else
  LOG_ERROR("Behavior HeadTargetSearch undefined without Naoqi")
  return false;
  #endif
}

template <typename Scalar>
void HeadTargetSearch<Scalar>::update()
{
  //LOG_INFO("HeadTargetSearch.update()...")
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  Scalar targetZ;
  cv::Point_<Scalar> targetXY;
  if (this->findTarget(targetType, targetXY, targetZ)) {
    cout << "Head target track ball found..." << endl;
    if (targetZ < 0.f) { // Used for killing behavior if a target is found
      finish();
    }
    Matrix<Scalar, 4, 1> posCam, posWorld;
    posWorld = Matrix<Scalar, 4, 1>(targetXY.x, targetXY.y, targetZ, 1.f);
    if (
      norm(targetXY) < 0.6 &&
      targetZ < 0.5) 
    { // If targetXY is within 60 cm and z is reasonably low, use lower cam
      posCam = this->kM->getWorldToCam(CameraId::headBottom, posWorld);
    } else { // else use upper cam
      posCam = this->kM->getWorldToCam(CameraId::headTop, posWorld);
    }
    //if (moveHeadToTarget(posCam)) { // Target reached
    finish();
    //}
  } else {
    //LOG_INFO("HeadTargetSearch.scanEnv()...")
    scanEnv();
  }
  #else
  LOG_ERROR("Behavior HeadTargetSearch undefined without Naoqi")
  finish();
  #endif
}

template <typename Scalar>
bool HeadTargetSearch<Scalar>::moveHeadToTarget(const Matrix<Scalar, 4, 1>& posCam)
{
  //LOG_INFO("HeadTargetSearch<Scalar>::moveHeadToTarget>..")
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  Matrix<Scalar, 2, 1> dAngles, aAngles;
  Matrix<Scalar, 2, 1> error, command;
  aAngles[HEAD_YAW] = this->kM->getJointState(HEAD_YAW)->position();
  aAngles[HEAD_PITCH] = this->kM->getJointState(HEAD_PITCH)->position();
  error[HEAD_YAW] =
    atan2(posCam[2], posCam[0]) - M_PI / 2;
  error[HEAD_PITCH] =
    -(atan2(posCam[2], posCam[1]) - M_PI / 2);
  command = pidGains[0] * error + pidGains[1] * intError;
  intError += error;
  Matrix<Scalar, 2, 1> fAngle;
  fAngle[HEAD_YAW] = aAngles[HEAD_YAW] + command[HEAD_YAW];
  fAngle[HEAD_PITCH] = aAngles[HEAD_PITCH] - command[HEAD_PITCH];
  if (fAngle[HEAD_YAW] >= headYawHigh) {
    command[HEAD_YAW] = 0.f;
  } else if (fAngle[HEAD_YAW] <= headYawLow) {
    command[HEAD_YAW] = 0.f;
  }
  if (fAngle[HEAD_PITCH] >= headPitchHigh) {
    command[HEAD_PITCH] = 0.f;
  } else if (fAngle[HEAD_PITCH] <= headPitchLow) {
    command[HEAD_PITCH] = 0.f;
  }
  AL::ALValue names = AL::ALValue::array("HeadYaw", "HeadPitch");
  AL::ALValue angles = AL::ALValue::array(
    command[HEAD_YAW],
    command[HEAD_PITCH]);
  Scalar fractionMaxSpeed = 1.f;
  if (
    abs(error[HEAD_YAW]) < 0.087222222 &&
    abs(error[HEAD_PITCH]) < 0.087222222)
  { // 5 degrees
    this->naoqiChangeAngles(names, angles, fractionMaxSpeed);
    return true;
  }
  return false;
  #endif
}

template <typename Scalar>
void HeadTargetSearch<Scalar>::finish()
{
  LOG_INFO("HeadTargetSearch.finish() called...");
  this->inBehavior = false;
}

template <typename Scalar>
void HeadTargetSearch<Scalar>::scanEnv()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  Scalar leftScanMax = getBehaviorCast()->scanMaxYaw;
  Scalar rightScanMax = -getBehaviorCast()->scanMaxYaw;
  Scalar headYaw = this->kM->getJointState(HEAD_YAW)->position();
  Scalar headPitch = this->kM->getJointState(HEAD_PITCH)->position();
  AL::ALValue nameYaw = AL::ALValue::array("HeadYaw");
  AL::ALValue angleYaw = AL::ALValue::array(0.f);
  AL::ALValue namePitch = AL::ALValue::array("HeadPitch");
  AL::ALValue anglePitch = AL::ALValue::array(16.f * M_PI / 180);
  Scalar fractionMaxSpeed = 0.05f;
  if (getBehaviorCast()->scanLowerArea) {
    //anglePitch[0] = getBehaviorCast()->scanMaxPitch;
    //error[1] = fabsf(anglePitch[0] - headPitch);
    //this->naoqiSetAngles(namePitch, anglePitch, fractionMaxSpeed);
    Scalar error;
    error = fabsf((Scalar)anglePitch[0] - headPitch);
    if (fabsf(prevCmdError[1] - error) < 1e-3)
      hpCmdResetCount++;
    if (hpCmdResetCount > 20) {
      this->naoqiSetAngles(namePitch, anglePitch, fractionMaxSpeed);
      hpCmdResetCount = 0;
    }
    prevCmdError[1] = error;
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
    Scalar error = fabsf((Scalar)angleYaw[0] - headYaw);
    if (fabsf(prevCmdError[0] - error) < 1e-6) {
      if (hyCmdResetCount++ > 20) {
        this->naoqiSetAngles(nameYaw, angleYaw, fractionMaxSpeed);
        hyCmdResetCount = 0;
      }
    } else {
      hyCmdResetCount = 0;
    }
    if (fabsf(headYaw) < TARGET_TOL)
    {
      finish();
    }
    prevCmdError[0] = error;
  }
  #endif
}

template <typename Scalar>
void HeadTargetSearch<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG("MotionBehaviors",
      (Scalar, HeadTargetSearch.kp, pidGains[0]),
      (Scalar, HeadTargetSearch.ki, pidGains[1]),
      (Scalar, HeadTargetSearch.kd, pidGains[2]),
    )
    loaded = true;
  }
}

template class HeadTargetSearch<MType>;
