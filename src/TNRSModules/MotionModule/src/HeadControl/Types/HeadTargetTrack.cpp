/**
 * @file MotionModule/src/HeadControl/Types/HeadTargetTrack.cpp
 *
 * This file implements the class HeadTargetTrack
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/HeadControl/Types/HeadTargetTrack.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/PIDController.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/Constants.h"
#include "Utils/include/ConfigMacros.h"

template <typename Scalar>
HeadTargetTrack<Scalar>::HeadTargetTrack(
  MotionModule* motionModule,
  const boost::shared_ptr<HeadTargetTrackConfig>& config) :
  HeadControl<Scalar>(motionModule, config, "HeadTargetTrack")
{
}

template <typename Scalar>
bool HeadTargetTrack<Scalar>::initiate()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  LOG_INFO("HeadTargetTrack.initiate() called...");
  this->trackersXY.resize(2);
  for (size_t i = 0; i < this->trackersXY.size(); ++i) {
    this->trackersXY[i] =
      boost::shared_ptr<PIDController<Scalar>>(
        new PIDController<Scalar>(this->cycleTime));
    this->trackersXY[i]->setPidGains(HeadTargetTrack::pidGains[i]);
  }
  if (this->getBehaviorCast()->scanConfig) {
    this->setupChildRequest(this->getBehaviorCast()->scanConfig, true);
  } else {
    this->getBehaviorCast()->scanConfig =
      boost::shared_ptr<HeadScanConfig>(new HeadScanConfig());
    this->setupChildRequest(this->getBehaviorCast()->scanConfig, true);
  }
  return true;
  #else
  LOG_ERROR("Behavior HeadTargetTrack is undefined without Naoqi motion proxy");
  return false;
  #endif
}

template <typename Scalar>
void HeadTargetTrack<Scalar>::update()
{
  //LOG_INFO("HeadTargetTrack.update()...")
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  Matrix<Scalar, 4, 1> posWorld;
  bool trackable;
  if (this->findTarget(getBehaviorCast()->headTargetType, posWorld, trackable)) {
    if (!trackable) {
      finish();
    }
    CameraId trackingCam = CameraId::headTop;
    if (posWorld.head(2).norm() < this->lowerCamUsageRange && posWorld[2] < this->lowerCamUsageZ) {
      ///< If targetXY is within 60 cm and z is reasonably low, use lower cam
      trackingCam = CameraId::headBottom;
    }
    if (trackTarget(this->kM->getWorldToCam(trackingCam, posWorld))) { ///< Target reached
      // finish();
      // keep tracking until killed
    }
    this->killChild();
  } else {
    this->setupChildRequest(this->getBehaviorCast()->scanConfig, true);
  }
  #else
  LOG_ERROR("Behavior HeadTargetTrack undefined without Naoqi");
  finish();
  #endif
}

template <typename Scalar>
bool HeadTargetTrack<Scalar>::trackTarget(const Matrix<Scalar, 4, 1>& posCam)
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  Matrix<Scalar, 2, 1> posFromCenter, meas;
  posFromCenter[0] = atan2(posCam[2], posCam[0]) - M_PI / 2; ///< X-angle
  posFromCenter[1] = -(atan2(posCam[2], posCam[1]) - M_PI / 2); ///< Y-angle
  meas[0] = this->kM->getJointState(Joints::headYaw)->position();
  meas[1] = this->kM->getJointState(Joints::headPitch)->position();
  bool targetsReached = true;
  for (size_t i = 0; i < 1; ++i) { // Moving it about Y has problems, so keep it fixed
    this->trackersXY[i]->setCmd(posFromCenter[i] + meas[i]);
    auto input = this->trackersXY[i]->update(meas[i]);
    this->naoqiChangeAngles(this->naoqiNames[i], input, this->fractionMaxSpeed);
    if (fabsf(this->trackersXY[i]->prevError1()) > Angle::DEG_2)
      targetsReached = false;
  }
  return targetsReached;
  #endif
}

template <typename Scalar>
void HeadTargetTrack<Scalar>::finish()
{
  LOG_INFO("HeadTargetTrack.finish() called...");
  this->inBehavior = false;
}

template <typename Scalar>
HeadTargetTrackConfigPtr HeadTargetTrack<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<HeadTargetTrackConfig> (this->config);
}

template class HeadTargetTrack<MType>;
