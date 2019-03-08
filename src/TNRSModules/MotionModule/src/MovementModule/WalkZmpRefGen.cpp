/**
 * @file MotionModule/WalkZmpRefGen/WalkZmpRefGen.h
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
#include "MotionModule/include/MovementModule/TNRSFootstep.h"
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
  transPose = RobotPose2D<Scalar>(0.0, 0.0, 0.0);
  for (size_t i = 0; i < this->nReferences; ++i) {
    this->zmpRef->x.push_back(zmp[0]);
    this->zmpRef->y.push_back(zmp[1]);
    cout << "this->zmpRef->x.front(): " << this->zmpRef->x.back() << endl;
    cout << "this->zmpRef->y.front(): " << this->zmpRef->y.back() << endl;
    if (footstep != footsteps->end() && time >= (*footstep)->timeAtFinish) {
      transPose = transPose.transform((*footstep)->pose2D);
      zmp[0] = transPose.getX();
      zmp[1] = transPose.getY();
      footstep++;
    }
    time += this->cycleTime;
  }
  return true;
}

template<typename Scalar>
void WalkZmpRefGen<Scalar>::update(const Scalar& timeStep)
{
  auto previewEndTime = timeStep + this->nReferences * this->cycleTime;
  //cout << "previewEndTime: "<< previewEndTime << endl;
  for (const auto& fs : boost::adaptors::reverse(*footsteps))
  {
    if (previewEndTime == fs->timeAtFinish) {
      transPose = transPose.transform(fs->pose2D);
      this->zmpRef->x.push_back(transPose.getX());
      this->zmpRef->y.push_back(transPose.getY());
      return;
    }
  }
  this->zmpRef->x.push_back(this->zmpRef->x.back());
  this->zmpRef->y.push_back(this->zmpRef->y.back());
  cout << "this->zmpRef->x.front(): " << this->zmpRef->x.back() << endl;
  cout << "this->zmpRef->y.front(): " << this->zmpRef->y.back() << endl;
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

