/**
 * @file MotionModule/src/FallDetector/FallDetector.cpp
 *
 * This file implements the class FallDetector
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 27 Sep 2017
 */

#include "TNRSBase/include/MemoryIOMacros.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/FallDetector/FallDetector.h"
#include "MotionModule/include/KinematicsModule/TorsoState.h"

template <typename Scalar>
FallDetector<Scalar>::FallDetector(MotionModule* motionModule) :
    MemoryBase(motionModule), bufferSize(BUFFER_SIZE)
{
  kM = motionModule->getKinematicsModule();
  torsoAccBuffer.set_capacity(bufferSize);
}

template <typename Scalar>
void FallDetector<Scalar>::update()
{
  //auto torsoAcceleration = kM->getTorsoState()->accel;
  //torsoAccBuffer.push_back(torsoAcceleration);
  //if (torsoAccBuffer.size() >= bufferSize) {
   // Matrix<Scalar, 3, 1>  avgAcc = Matrix<Scalar, 3, 1> ::Zero();
   // for (int i = 0; i < torsoAccBuffer.size(); ++i)
   //   avgAcc = avgAcc + torsoAccBuffer[i];
   // avgAcc = avgAcc / bufferSize;
   // float angleY(
   //   atan2(avgAcc[0], sqrt(avgAcc[1] * avgAcc[1] + avgAcc[2] * avgAcc[2])));
    //float angleYZ(atan2(-avgAcc[1], -avgAcc[2]));
    //cout << "kM->getFootOnGround(): " << kM->getFootOnGround() << endl;
    //cout << "avgAcc: " << avgAcc << endl;
  auto& inertial = INERTIAL_SENSORS_OUT(MotionModule);
  auto& angleY = inertial[toUType(InertialSensors::torsoAngleY)];
  //cout << "angleY: " << (Scalar) inertial[TORSO_ANGLE_Y]  * 180 / M_PI << endl;
  //cout << "angleX: " << (Scalar) inertial[TORSO_ANGLE_X]  * 180 / M_PI  << endl;
    //cout << "angleYZ: " << angleYZ * 180 / M_PI << endl;
  //if (kM->getFootOnGround() == -1) {
  PostureState posture = POSTURE_STATE_OUT(MotionModule);
  if (angleY > M_PI / 4) { // 45 degrees
    if (angleY > M_PI / 3) {
      posture = PostureState::fallFront;
      ROBOT_FALLEN_OUT(MotionModule) = true;
    } else {
      posture = PostureState::fallingFront;
    }
  } else if (angleY < -M_PI / 4) { // -45 degrees
    if (angleY < -M_PI / 3) {
      posture = PostureState::fallBack;
      ROBOT_FALLEN_OUT(MotionModule) = true;
    } else {
      posture = PostureState::fallingBack;
    }
  }// else {
    //Fix this/
    //posture = PostureState::FALL_SIT;
    //ROBOT_FALLEN_OUT(MotionModule) = true;
  //}
  POSTURE_STATE_OUT(MotionModule) = posture;
  //} else {
  //  ROBOT_FALLEN_OUT(MotionModule) = false;
  //}
  //} else {
  //  ROBOT_FALLEN_OUT(MotionModule) = false;
  //}
}

template class FallDetector<MType>;
