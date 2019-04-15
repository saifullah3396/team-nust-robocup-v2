/**
 * @file MotionModule/src/MovementModule/WalkZmpRefGen.cpp
 *
 * This file implements the class WalkZmpRefGen
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include <boost/range/adaptor/reversed.hpp>
#include "MotionModule/include/KinematicsModule/ComState.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/MovementModule/WalkZmpRefGen.h"
#include "Utils/include/DataHolders/TNRSFootstep.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/Constants.h"

template<typename Scalar>
WalkZmpRefGen<Scalar>::WalkZmpRefGen(
  MotionModule* motionModule,
  const RobotFeet& refFrame,
  const unsigned& nReferences,
  std::deque<boost::shared_ptr<TNRSFootstep<Scalar>>>* footsteps) :
  ZmpRefGenerator<Scalar>(motionModule, refFrame, nReferences),
  footsteps(footsteps)
{
}

template<typename Scalar>
bool WalkZmpRefGen<Scalar>::initiate()
{
  Matrix<Scalar, 2, 1> zmp =
    this->kM->getComStateWrtFrame(
      static_cast<LinkChains>(this->refFrame), toUType(LegEEs::footCenter)).zmp;

  auto footstep = footsteps->begin();
  auto time = 0.0;
  globalTransPose = RobotPose2D<Scalar>(0.0, 0.0, 0.0);
  for (size_t i = 0; i < this->nReferences; ++i) {
    if (footstep != footsteps->end() && fabsf(time - (*footstep)->timeAtFinish) <= 1e-3) {
      auto currentOffset = refOffset;
      if ((*footstep)->foot == RobotFeet::rFoot)
        currentOffset[1] *= -1;
      globalTransPose = globalTransPose.transform((*footstep)->pose2D);
      zmp[0] = globalTransPose.getX();
      zmp[1] = globalTransPose.getY();
      zmp += currentOffset;
      footstep++;
    }
    this->zmpRef->x.push_back(zmp[0]);
    this->zmpRef->y.push_back(zmp[1]);
    //LOG_INFO(i << " this->zmpRef->y.front(): " << this->zmpRef->y.back());
    time += this->cycleTime;
  }
  return true;
}

template<typename Scalar>
void WalkZmpRefGen<Scalar>::retransform(Scalar timeStep)
{
  auto footstep = footsteps->begin();
  auto transPose = (*footstep)->pose2D.getInverse();
  footstep++;
  Matrix<Scalar, 2, 1> zmp;
  zmp[0] = this->zmpRef->x.front();
  zmp[1] = this->zmpRef->y.front();
  //for (size_t i = 0; i < this->nReferences; ++i) {
   //LOG_INFO("zmp:" << this->zmpRef->x[i] << ", " << this->zmpRef->y[i]);
  //}
  //LOG_INFO("zmp: " << zmp);
  zmp = transPose.transform(zmp);
  //LOG_INFO("zmp transformed: " << zmp);
  this->zmpRef->x.clear();
  this->zmpRef->y.clear();
  transPose = RobotPose2D<Scalar>(0.0, 0.0, 0.0);
  for (size_t i = 0; i < this->nReferences; ++i) {
    if (footstep != footsteps->end() && fabsf(timeStep - (*footstep)->timeAtFinish) <= 1e-3) {
      auto currentOffset = refOffset;
      if ((*footstep)->foot == RobotFeet::rFoot)
        currentOffset[1] *= -1;
      transPose = transPose.transform((*footstep)->pose2D);
      zmp[0] = transPose.getX();
      zmp[1] = transPose.getY();
      zmp += currentOffset;
      footstep++;
    }
    this->zmpRef->x.push_back(zmp[0]);
    this->zmpRef->y.push_back(zmp[1]);
    timeStep += this->cycleTime;
  }
  this->globalTransPose = RobotPose2D<Scalar>(0.0, 0.0, 0.0);
}

template<typename Scalar>
void WalkZmpRefGen<Scalar>::update(const Scalar& timeStep)
{
  auto previewEndTime = timeStep + this->nReferences * this->cycleTime;
  //LOG_INFO("previewEndTime: "<< previewEndTime);
  for (const auto& fs : boost::adaptors::reverse(*footsteps))
  {
    ///< Matching floats here, this can be dangerous
    if (fabsf(previewEndTime - fs->timeAtFinish) <= 1e-3) {
      //LOG_INFO("fs->timeAtFinish: " << fs->timeAtFinish);
      auto currentOffset = refOffset;
      if (fs->foot == RobotFeet::rFoot)
        currentOffset[1] *= -1;
      globalTransPose = globalTransPose.transform(fs->pose2D);
      this->zmpRef->x.push_back(globalTransPose.getX() + currentOffset[0]);
      this->zmpRef->y.push_back(globalTransPose.getY() + currentOffset[1]);
      //LOG_INFO("latestZmpRefY: " << this->zmpRef->y.back());
      return;
    }
  }
  this->zmpRef->x.push_back(this->zmpRef->x.back());
  this->zmpRef->y.push_back(this->zmpRef->y.back());
  //LOG_INFO("latestZmpRefY: " << this->zmpRef->y.back());
  //cout << "previewsAvailable():" << previewsAvailable(timeStep) << endl;
}

template<typename Scalar>
bool WalkZmpRefGen<Scalar>::previewsAvailable(const Scalar& timeStep)
{
  if ((footsteps->back()->timeAtFinish - timeStep) / this->cycleTime <= this->nReferences)
    return false;
  return true;
}

template class WalkZmpRefGen<MType>;

