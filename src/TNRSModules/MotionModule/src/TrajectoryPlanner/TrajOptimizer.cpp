/**
 * @file MotionModule/src/TrajectoryPlanner/TrajOptimizer.cpp
 *
 * This file implements the class TrajOptimizer
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/TrajectoryPlanner/TrajOptimizer.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/KinematicsModule/Joint.h"

template <typename Scalar>
TrajOptimizer<Scalar>::TrajOptimizer(
  MotionModule* motionModule,
  const LinkChains& chainIndex,
  const LinkChains& baseLeg) :
  kM(motionModule->getKinematicsModule()), 
  chainIndex(chainIndex),
  baseLeg(baseLeg),
  maxVelCutoff(0.95)
{
  stepSize = kM->getCycleTime();
  auto chain = kM->getLinkChain(chainIndex);
  chainStart = chain->start;
  chainSize = chain->size;
  
  auto joints =
    kM->getJoints(static_cast<Joints>(chain->start), chain->size);
  velLimits.resize(chain->size);
  for (size_t i = 0; i < chain->size; ++i) {
    velLimits[i] = joints[i]->maxVelocity * maxVelCutoff;
    //velLimits[i] -= 0.15;
  }
}

template <typename Scalar>
TrajOptimizer<Scalar>::~TrajOptimizer()
{
}

template class TrajOptimizer<MType>;
