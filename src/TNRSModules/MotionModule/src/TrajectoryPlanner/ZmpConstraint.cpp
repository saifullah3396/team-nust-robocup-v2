
/**
 * @file MotionModule/src/TrajectoryPlanner/ZmpConstraint.cpp
 *
 * This file implements the class ZmpConstraint
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/TrajectoryPlanner/ZmpConstraint.h"

template <typename Scalar>
ZmpConstraint<Scalar>::ZmpConstraint(
  MotionModule* motionModule,
  const LinkChains& supportLeg,
  const JointStateType& type,
  const vector<bool>& activeChains) :
  NloptConstraint<Scalar>("ZmpConstraint"),
  kM(motionModule->getKinematicsModule()),
  supportLeg(supportLeg),
  type(type),
  activeChains(activeChains)
{
  minMaxX[0] = -0.015;
  minMaxX[1] = 0.015;
  minMaxY[0] = -0.015;
  minMaxY[1] = 0.015;
}

template <typename Scalar>
void ZmpConstraint<Scalar>::init()
{
  // Finding all static chain forces once at start
  Matrix<Scalar, 3, 1> extForces;
  Matrix<Scalar, 3, 1> extMoments;
  Matrix<Scalar, 3, 1> chainForces;
  Matrix<Scalar, 3, 1> chainMoments;
  extForces.setZero(); // No external forces acting
  extMoments.setZero(); // No external forces acting
  initTorsoForces.setZero();
  initTorsoMoments.setZero();
  for (const auto& chain : LinkChains()) {
    if (!activeChains[toUType(chain)]) { // if not an active chain then solve once at start
      kM->newtonEulerForces(
        chain, extForces, extMoments, chainForces, chainMoments, supportLeg, type);
      initTorsoForces += chainForces;
      initTorsoMoments += chainMoments;
    }
  }
  //cout << "initTorsoForces: " << initTorsoForces << endl;
  //cout << "initTorsoMoments: " << initTorsoMoments << endl;
  cons.resize(4);
  this->tols.resize(4);
  for (int i = 0; i < this->tols.size(); ++i) {
    this->tols[i] = 1e-4;
  }
}

template <typename Scalar>
void ZmpConstraint<Scalar>::update()
{
  Matrix<Scalar, 3, 1> torsoForces;
  Matrix<Scalar, 3, 1> torsoMoments;
  Matrix<Scalar, 3, 1> extForces;
  Matrix<Scalar, 3, 1> extMoments;
  Matrix<Scalar, 3, 1> chainForces;
  Matrix<Scalar, 3, 1> chainMoments;
  extForces.setZero(); // No external forces acting
  extMoments.setZero(); // No external forces acting
  torsoForces.setZero();
  torsoMoments.setZero();
  for (const auto& chain : LinkChains()) {
    if (activeChains[toUType(chain)]) { // if not an active chain then solve once at start
      //cout << "active Chain: " << i << endl;
      kM->newtonEulerForces(
        chain, extForces, extMoments, chainForces, chainMoments, supportLeg, type);
      //cout << "chainForces: " << chainForces << endl;
      //cout << "chainMoments: " << chainMoments << endl;
      torsoForces += chainForces;
      torsoMoments += chainMoments;
    }
  }
  torsoForces += initTorsoForces;
  torsoMoments += initTorsoMoments;
  zmp = this->kM->computeZmpWrtForces(supportLeg, torsoForces, torsoMoments, JointStateType::sim);
  //cout << "Zmpx: " << zmp[0] << endl;
}

template <typename Scalar>
vector<Scalar> ZmpConstraint<Scalar>::computeConstraintMatrix()
{
  cons[0] = minMaxX[0] - zmp[0]; //x lower bound | value >= min ==> min - value <= 0
  cons[1] = zmp[0] - minMaxX[1]; //x upper bound | value <= max ==> value - max <= 0
  cons[2] = minMaxY[0] - zmp[1]; //y lower bound |
  cons[3] = zmp[1] - minMaxY[1]; //y upper bound |
  //cout << "zmp xmin cons; " << cons[0] << endl;
  //cout << "zmp xmax cons; " << cons[1] << endl;
  //cout << "zmp ymin cons; " << cons[2] << endl;
  //cout << "zmp ymax cons; " << cons[3] << endl;
  return cons;
}

template class ZmpConstraint<MType>;

