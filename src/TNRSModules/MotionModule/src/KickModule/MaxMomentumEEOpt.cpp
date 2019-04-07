/**
 * @file MotionModule/src/KickModule/MaxMomentumEEOpt.cpp
 *
 * This file implements the class MaxMomentumEEOpt
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */

#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KickModule/MaxMomentumEEOpt.h"
#include "MotionModule/include/KickModule/KickModule.h"

template <typename Scalar>
MaxMomentumEEOpt<Scalar>::MaxMomentumEEOpt(KickModule<Scalar>* kickModule) :
  kickModulePtr(kickModule),
  kM(kickModule->getKinematicsModule())
{
}

template <typename Scalar>
double MaxMomentumEEOpt<Scalar>::costFunction(
  const vector<double>& vars, vector<double>& grad, void *data)
{
  //cout << "Finding f.. " << endl;
  double f = 0;
  // Set end-effector position based on bezier parameter t.
  kickModulePtr->setEndEffectorZX(vars[2]);
  Matrix<Scalar, 4, 4> tX, tY;
  Matrix<Scalar, 4, 4> newPose = kickModulePtr->impactPose;
  MathsUtils::makeRotationX(tX, vars[0]);
  MathsUtils::makeRotationY(tY, vars[1]);
  newPose.block(0, 0, 3, 3) = (tY * tX).block(0, 0, 3, 3);
  newPose = kickModulePtr->torsoToSupport * newPose;
  vector <Matrix<Scalar, Dynamic, 1>> angles;
  if (kickModulePtr->kickLeg == LinkChains::lLeg) angles =
    kM->inverseLeftLeg(kickModulePtr->endEffector, newPose);
  else angles = 
    kM->inverseRightLeg(kickModulePtr->endEffector, newPose);
  Scalar vm;
  Matrix<Scalar, 3, 1> direction = 
    kickModulePtr->torsoToSupport.block(0, 0, 3, 3) * kickModulePtr->ballToTargetUnit;
  kM->setChainPositions(kickModulePtr->kickLeg, angles[0], JointStateType::sim);
  kM->computeVirtualMass(
    kickModulePtr->kickLeg,
    direction,
    kickModulePtr->endEffector,
    vm,
    JointStateType::sim);
  Scalar vel = vars[3];
  f = 1 / (vel * vm) + 1 / kickModulePtr->endEffector(0, 3);
  return f;
}

template <typename Scalar>
void MaxMomentumEEOpt<Scalar>::ineqConstraints(unsigned nCons, double *result, unsigned nOptVars,
  const double* vars, double* grad, void* data)
{
  //cout << "Solving Inequality Constraints..." << endl;
  Matrix<Scalar, Dynamic, 1> fVel;
  Scalar vel = vars[3];
  fVel.resize(3);
  fVel.setZero();
  fVel[0] = vel * kickModulePtr->ballToTargetUnit[0];
  fVel[1] = vel * kickModulePtr->ballToTargetUnit[1];
  fVel[2] = 0.0;
  fVel.block(0, 0, 3, 1) = kickModulePtr->torsoToSupport.block(0, 0, 3, 3) * fVel.segment(0, 3);
  /*kickModulePtr->setEndEffectorZX(vars[2]);
  Matrix<Scalar, 4, 4> tX, tY;
  Matrix<Scalar, 4, 4> newPose = kickModulePtr->impactPose;
  MathsUtils::makeRotationX(tX, vars[0]);
  MathsUtils::makeRotationY(tY, vars[1]);
  newPose.block(0, 0, 3, 3) = (tY * tX).block(0, 0, 3, 3);
  newPose = kickModulePtr->torsoToSupport * newPose;
  vector <Matrix<Scalar, Dynamic, 1>> angles;
  if (kickModulePtr->kickLeg == LinkChains::lLeg) angles =
    kM->inverseLeftLeg(kickModulePtr->endEffector, newPose);
  else angles = 
    kM->inverseRightLeg(kickModulePtr->endEffector, newPose);
  kM->setChainPositions(kickModulePtr->kickLeg, angles[0], JointStateType::sim);*/
  Matrix<Scalar, Dynamic, Dynamic> jacobian = kM->computeLimbJ(
    kickModulePtr->kickLeg,
    kickModulePtr->endEffector,
    JointStateType::sim).block(0, 0, 3, 6);
  Matrix<Scalar, Dynamic, 1> jVels = MathsUtils::pseudoInverseSolve(jacobian, fVel);
  Matrix<Scalar, Dynamic, 1> velCons = jVels.cwiseAbs() - velLimits;
  for (size_t i = 0; i < nCons; ++i) {
    result[i] = velCons[i];
  }
  //cout << "Finished solving Inequality Constraints..." << endl;
}

template <typename Scalar>
void MaxMomentumEEOpt<Scalar>::optDef()
{
  ///<Objective function to minimize is virtualMass x velocity;
  ///<Hessian for this objective function is unknown.
  ///<Gradient for this function is unknown.
  ///<4 variables; 2xEuler Angles, end-effector contour parameter t, and velocity.
  nlopt::opt opt(nlopt::LN_COBYLA, 4);
  vector<double> lb(4), ub(4), var0, constraintTols;
  //These are not euler angles rather they are fixed angle rotations.
  lb[0] = -5.0 * M_PI / 180.0; ///< Lower bound for x-angle. 
  lb[1] = -10.0 * M_PI / 180.0; ///< Lower bound for y-angle. 
  lb[2] = 0.0; ///< Lower bound for parameterized curve [0...1].
  lb[3] = 0.01; ///< Lower bound for velocity in given direction.
  ub[0] = 5.0 * M_PI / 180.0; ///< Upper bound for x-angle. 
  ub[1] = 10.0 * M_PI / 180.0; ///< Upper bound for y-angle. 
  ub[2] = 1.0; ///< Upper bound for parameterized curve [0...1].
  ub[3] = 2.0; ///< Upper bound for velocity in given direction.
  for (size_t i = 0; i < lb.size(); ++i)
    var0.push_back(lb[i]);

  ///< Joint velocity contraint for leg joints;
  auto chainStart = kM->getLinkChain(kickModulePtr->kickLeg)->start;
  auto chainSize = kM->getLinkChain(kickModulePtr->kickLeg)->size;
  velLimits.resize(chainSize);
  for (size_t i = 0; i < chainSize; ++i) {
    auto joint = kM->getJoint(static_cast<Joints>(chainStart + i));
    velLimits[i] = joint->maxVelocity;
  }
  unsigned nCons = chainSize;
  for (size_t i = 0; i < nCons; ++i) {
    constraintTols.push_back(1e-8);
  }
  opt.add_inequality_mconstraint(MaxMomentumEEOpt<Scalar>::ineqWrapper, this, constraintTols);
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_min_objective(MaxMomentumEEOpt<Scalar>::objWrapper, this);
  opt.set_xtol_rel(1e-4);
  opt.set_maxeval(500);
  double minf;
  cout << "Starting optimization... " << endl;
  nlopt::result result = opt.optimize(var0, minf);
  if (result < 0) {
    cout << "nlopt failed!" << endl;
  } else {
    cout << "Found minimum at the variables:" << var0[0] << endl << var0[1] << endl << var0[2] << endl << var0[3] << endl << "with f:" << endl << minf << endl;
  }

  kickModulePtr->setEndEffectorZX(var0[2]);
  Matrix<Scalar, 4, 4> tX, tY;
  MathsUtils::makeRotationX(tX, var0[0]);
  MathsUtils::makeRotationY(tY, var0[1]);
  kickModulePtr->impactPose.block(0, 0, 3, 3) = (tY * tX).block(0, 0, 3, 3);
  cout << "Optimized endEffector pose:\n " << kickModulePtr->endEffector << endl;
  cout << "Max Velocity in Given Direction:\n " << var0[3] << endl;
  cout << "Best EndEffector Position: " << kickModulePtr->endEffector << endl;
}

template class MaxMomentumEEOpt<MType>;
