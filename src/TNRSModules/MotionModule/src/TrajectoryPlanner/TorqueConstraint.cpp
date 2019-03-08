
/**
 * @file MotionModule/src/TrajectoryPlanner/TorqueConstraint.cpp
 *
 * This file implements the class TorqueConstraint
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/LinkInfo.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/TrajectoryPlanner/TorqueConstraint.h"
#include "Utils/include/Constants.h"

template <typename Scalar>
TorqueConstraint<Scalar>::TorqueConstraint(
  MotionModule* motionModule,
  const LinkChains& supportLeg,
  const JointStateType& type,
  const vector<bool>& activeJoints) :
  NloptConstraint<Scalar>("TorqueConstraint"),
  kM(motionModule->getKinematicsModule()),
  supportLeg(supportLeg),
  type(type),
  activeJoints(activeJoints)
{
  torques.resize(toUType(Joints::count));
  torques.setZero();
  maxTorques.resize(toUType(Joints::count));
  for (size_t i = 0; i < toUType(Joints::count); ++i) {
    maxTorques[i] = Constants::jointMaxTorques[i];
  }
}

template <typename Scalar>
void TorqueConstraint<Scalar>::init()
{
  activeChains.resize(toUType(LinkChains::count));
  unsigned count = 0;
  for (const auto& joint : Joints()) {
    if (activeJoints[toUType(joint)]) {
      activeChains[toUType(kM->getJoint(joint)->link->chain->index)] = true;
      this->tols.push_back(1e-3);
      count++;
    }
  }
  nCons = count;
}

template <typename Scalar>
void TorqueConstraint<Scalar>::update()
{
  cons.clear();
  // Finding all static chain forces once at start
  Matrix<Scalar, 3, 1> extForces;
  Matrix<Scalar, 3, 1> extMoments;
  Matrix<Scalar, 3, 1> chainForces;
  Matrix<Scalar, 3, 1> chainMoments;
  extForces.setZero(); // No external forces acting
  extMoments.setZero(); // No external forces acting
  for (const auto& chain : LinkChains()) {
    if (activeChains[toUType(chain)]) {
      Matrix<Scalar, Dynamic, 1> torque =
        kM->newtonEulerForces(chain, extForces, extMoments, chainForces, chainMoments, supportLeg, type);
      torques.block(kM->getLinkChain(chain)->start, 0, torque.size(), 1) = torque;
    }
  }
}

template <typename Scalar>
vector<Scalar> TorqueConstraint<Scalar>::computeConstraintMatrix()
{
  //cout << "torque constraints:" << endl;
  for (size_t i = 0; i < toUType(Joints::count); ++i) {
    if (activeJoints[i]) {
      cons.push_back(fabsf(torques[i]) - maxTorques[i]);
      //cout << fabsf(torques[i]) - maxTorques[i] << endl;
    }
  }
  return cons;
}

template class TorqueConstraint<MType>;
