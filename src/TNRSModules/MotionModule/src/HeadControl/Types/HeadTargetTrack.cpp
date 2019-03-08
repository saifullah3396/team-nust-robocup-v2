/**
 * @file MotionModule/HeadControl/Types/HeadTargetTrack.h
 *
 * This file implements the class HeadTargetTrack
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/HeadControl/Types/HeadTargetTrack.h"
#include "TNRSBase/include/MemoryIOMacros.h"

template <typename Scalar>
Matrix<Scalar, 3, 1> HeadTargetTrack<Scalar>::pidGains;

template <typename Scalar>
HeadTargetTrackConfigPtr HeadTargetTrack<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<HeadTargetTrackConfig> (this->config);
}

template <typename Scalar>
void HeadTargetTrack<Scalar>::initiate()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
	LOG_INFO("HeadTargetTrack.initiate()")
  this->targetType = this->getBehaviorCast()->headTargetType;
  this->inBehavior = true;
  #else
  LOG_ERROR("Behavior HeadTargetTrack undefined without Naoqi")
  finish();
  #endif
}

template <typename Scalar>
void HeadTargetTrack<Scalar>::update()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  Scalar targetZ;
  cv::Point_<Scalar> targetXY;
  if (this->findTarget(targetType, targetXY, targetZ)) {
    if (targetZ < 0.f) { // Used for killing behavior if a target is found and cannot be tracked
      LOG_INFO("HeadTargetTrack.update(): Cannot track target. Finishing behavior...")
      finish();
    }
    Matrix<Scalar, 4, 1> posCam, posWorld;
    posWorld = Matrix<Scalar, 4, 1>(targetXY.x, targetXY.y, targetZ, 1.f);
    if (
      norm(targetXY) < 0.6 &&
      targetZ < 0.5f)
    { // If targetXY is within 60 cm and z is reasonably low, use lower cam
      posCam = this->kM->getWorldToCam(CameraId::headBottom, posWorld);
    } else { // else use upper cam
      posCam = this->kM->getWorldToCam(CameraId::headTop, posWorld);
    }
    followTarget(posCam);
  } else {
  /*  targetLostTime += this->cycleTime;
    if (targetLostTime > 5.f) {
      LOG_INFO("HeadTargetTrack.update(): Target lost. Finishing behavior...")
      finish();
    }
  0} else {*/
    this->setupChildRequest(getBehaviorCast()->htsConfig);
  }
  #else
  LOG_ERROR("Behavior HeadTargetTrack undefined without Naoqi")
  finish();
  #endif
}

template <typename Scalar>
void HeadTargetTrack<Scalar>::followTarget(const Matrix<Scalar, 4, 1>& posCam)
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  // TODO: Fix this. Too messy. Also add the use of pid somewhere e.e 
  Matrix<Scalar, 2, 1> dAngles, aAngles;
  Matrix<Scalar, 2, 1> error, command;
  aAngles[HEAD_YAW] = this->kM->getJointState(HEAD_YAW)->position();
  aAngles[HEAD_PITCH] = this->kM->getJointState(HEAD_PITCH)->position();
  error[HEAD_YAW] =
    atan2(posCam[2], posCam[0]) - M_PI / 2;
  error[HEAD_PITCH] =
    -(atan2(posCam[2], posCam[1]) - M_PI / 2);
  command = pidGains[0] * error + pidGains[1] * intError;
  /*command =
   prevCommand +
   (pidKp + pidKi * this->cycleTime / 2 + pidKd / this->cycleTime) * error +
   (-pidKp + pidKi * this->cycleTime / 2 - 2 * pidKd / this->cycleTime) * errorK1 +
   (pidKd / this->cycleTime) * errorK2;
   prevCommand = command;
   errorK2 = errorK1;
   errorK1 = error;
   command[HEAD_YAW] += aAngles[HEAD_YAW];
   command[HEAD_PITCH] += aAngles[HEAD_PITCH];*/
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
  this->naoqiChangeAngles(names, angles, fractionMaxSpeed);
  #endif
}

template <typename Scalar>
void HeadTargetTrack<Scalar>::finish()
{
  this->inBehavior = false;
}

template <typename Scalar>
void HeadTargetTrack<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG("MotionBehaviors",
      (Scalar, HeadTargetTrack.kp, pidGains[0]),
      (Scalar, HeadTargetTrack.ki, pidGains[1]),
      (Scalar, HeadTargetTrack.kd, pidGains[2]),
    )
    loaded = true;
  }
}

template class HeadTargetTrack<MType>;
