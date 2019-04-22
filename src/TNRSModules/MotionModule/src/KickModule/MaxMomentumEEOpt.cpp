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
#include "Utils/include/PrintUtils.h"

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
  double f = 0;
  // Set end-effector position based on bezier parameter t.
  kickModulePtr->setEndEffectorZX(vars[2]);
  Matrix<Scalar, 4, 4> tX, tY;
  Matrix<Scalar, 4, 4> newPose = kickModulePtr->impactPose;
  MathsUtils::makeRotationX(tX, vars[0]);
  MathsUtils::makeRotationY(tY, vars[1]);
  newPose.block(0, 0, 3, 3) = (tY * tX).block(0, 0, 3, 3);
  Matrix<Scalar, Dynamic, 1> angles =
    this->kM->solveJacobianIK(
      kickModulePtr->kickLeg, kickModulePtr->endEffector, newPose, 100, activeJoints, activeResidual);
  Scalar vm;
  kM->setChainPositions(kickModulePtr->kickLeg, angles, JointStateType::sim);
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
  Matrix<Scalar, 3, 1> cVel = vars[3] * kickModulePtr->ballToTargetUnit;
  cVel[2] = 0.1;
  //! Get velocity in torso frame
  cVel = kickModulePtr->torsoToSupport.block(0, 0, 3, 3) * cVel;
  Matrix<Scalar, 3, Dynamic> jacobian = kM->computeLimbJ(
    kickModulePtr->kickLeg,
    kickModulePtr->endEffector,
    JointStateType::sim).block(0, 0, 3, chainSize);
  JacobiSVD<Matrix<Scalar, 3, Dynamic>> svd(jacobian, ComputeThinU | ComputeThinV);
  Matrix<Scalar, Dynamic, 1> jVels = svd.solve(cVel);
  for (size_t i = 0; i < chainSize; ++i) {
    result[i] = fabsf(jVels[i]) - velLimits[i];
  }
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
  lb[1] = 0.0 * M_PI / 180.0; ///< Lower bound for y-angle.
  lb[2] = 0.0; ///< Lower bound for parameterized curve [0...1].
  lb[3] = 0.25; ///< Lower bound for velocity in given direction.
  ub[0] = 5.0 * M_PI / 180.0; ///< Upper bound for x-angle.
  ub[1] = 10.0 * M_PI / 180.0; ///< Upper bound for y-angle.
  ub[2] = 1.0; ///< Upper bound for parameterized curve [0...1].
  ub[3] = 2.0; ///< Upper bound for velocity in given direction.
  for (size_t i = 0; i < lb.size(); ++i)
    var0.push_back(lb[i]);

  ///< Joint velocity contraint for leg joints;
  chainStart = kM->getLinkChain(kickModulePtr->kickLeg)->start;
  chainSize = kM->getLinkChain(kickModulePtr->kickLeg)->size;

  activeJoints = vector<bool>(chainSize, true);
  //! Removing hipyawpitch from computations
  activeJoints[0] = false;

  //! Velocity direction
  direction =
    kickModulePtr->torsoToSupport.block(0, 0, 3, 3) *
    kickModulePtr->ballToTargetUnit;

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
  try {
    nlopt::result result = opt.optimize(var0, minf);
    if (result < 0) {
      kickModulePtr->kickFailed = true;
    } else {
      kickModulePtr->setEndEffectorZX(var0[2]);
      Matrix<Scalar, 4, 4> tX, tY;
      MathsUtils::makeRotationX(tX, var0[0]);
      MathsUtils::makeRotationY(tY, var0[1]);
      kickModulePtr->impactPose.block(0, 0, 3, 3) = (tY * tX).block(0, 0, 3, 3);
      if (kickModulePtr->desImpactVelKnown) {
        if (kickModulePtr->desImpactVel.norm() >= var0[3])
          kickModulePtr->desImpactVel = var0[3] * kickModulePtr->ballToTargetUnit;
      }
      cout << "kickModulePtr->desImpactVel:" << kickModulePtr->desImpactVel << endl;
      cout << "Best velocity :" << var0[3]<< endl;
    }
  } catch (const exception& e) {
    LOG_EXCEPTION(e.what());
    kickModulePtr->kickFailed = true;
  }
}

template class MaxMomentumEEOpt<MType>;
