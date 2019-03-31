/**
 * @file MotionModule/BalanceZmpRefGen/BalanceZmpRefGen.h
 *
 * This file implements the class BalanceZmpRefGen
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include "MotionModule/include/KinematicsModule/ComState.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/BalanceModule/BalanceZmpRefGen.h"
#include "Utils/include/PrintUtils.h"

template<typename Scalar>
BalanceZmpRefGen<Scalar>::BalanceZmpRefGen(
  MotionModule* motionModule,
  const RobotFeet& refFrame,
  const unsigned& nReferences,
  const Scalar& totalTime,
  const Matrix<Scalar, 2, 1>& targetZmp) :
  ZmpRefGenerator<Scalar>(motionModule, refFrame, nReferences),
  totalTime(totalTime),
  targetZmp(targetZmp)
{
}

template<typename Scalar>
bool BalanceZmpRefGen<Scalar>::initiate() {
  initZmpPosition =
    this->kM->getComStateWrtFrame(
      static_cast<LinkChains>(this->refFrame), toUType(LegEEs::footBase)).zmp;
  LOG_INFO("initZmpPosition: " << initZmpPosition.transpose());
  auto shiftStep = this->totalTime / this->cycleTime * 0.5;
  for (size_t i = 0; i < this->nReferences; ++i) {
    if (i < shiftStep) {
      this->zmpRef->x.push_back(initZmpPosition[0]);
      this->zmpRef->y.push_back(initZmpPosition[1]);
    } else {
      this->zmpRef->x.push_back(targetZmp[0]);
      this->zmpRef->y.push_back(targetZmp[1]);
    }
  }
  return true;
}

template<typename Scalar>
void BalanceZmpRefGen<Scalar>::update(const Scalar& timeStep) {
  if (timeStep + this->nReferences * this->cycleTime < this->totalTime / 2) {
    this->zmpRef->x.push_back(initZmpPosition[0]);
    this->zmpRef->y.push_back(initZmpPosition[1]);
  } else {
    this->zmpRef->x.push_back(targetZmp[0]);
    this->zmpRef->y.push_back(targetZmp[1]);
  }
}

template class BalanceZmpRefGen<MType>;
