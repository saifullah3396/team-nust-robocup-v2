/**
 * @file MotionModule/src/BalanceModule/ZmpRefGenerator.cpp
 *
 * This file implements the class ZmpRefGenerator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/BalanceModule/ZmpRef.h"
#include "MotionModule/include/BalanceModule/ZmpRefGenerator.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"

template <typename Scalar>
ZmpRefGenerator<Scalar>::ZmpRefGenerator(
  MotionModule* motionModule,
  const RobotFeet& refFrame,
  const unsigned& nReferences) :
  kM(motionModule->getKinematicsModule()),
  refFrame(refFrame),
  nReferences(nReferences)
{
  cycleTime = kM->getCycleTime();
  zmpRef = boost::shared_ptr<ZmpRef<Scalar>>(new ZmpRef<Scalar>(nReferences));
}

template class ZmpRefGenerator<MType>;
typedef boost::shared_ptr<ZmpRefGenerator<MType> > ZmpRefGeneratorPtr;
