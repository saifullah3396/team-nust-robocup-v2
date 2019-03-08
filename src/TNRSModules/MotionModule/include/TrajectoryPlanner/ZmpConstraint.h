/**
 * @file MotionModule/TrajectoryPlanner/ZmpConstraint.h
 *
 * This file declares the class ZmpConstraint
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include "MotionModule/include/KinematicsModule/JointStateType.h"
#include "MotionModule/include/TrajectoryPlanner/NloptConstraint.h"
#include "Utils/include/HardwareIds.h"

template <typename Scalar>
class KinematicsModule;

/**
 * @class ZmpConstraint
 * @brief The class to define a zero-moment point constraint handler for nlopt based
 *   optimization problems
 */
template <typename Scalar>
class ZmpConstraint : public NloptConstraint<Scalar>
{
public:
  /**
   * Constructor.
   *
   * @param motionModule Pointer to base motion module
   */
  ZmpConstraint(
    MotionModule* motionModule,
    const LinkChains& supportLeg = LinkChains::lLeg,
    const JointStateType& type = JointStateType::sim,
    const vector<bool>& activeChains = vector<bool>(toUType(LinkChains::count), true));

  /**
   * Default destructor for this class.
   */
  ~ZmpConstraint()
  {
  }

  void init();
  void update();
  vector<Scalar> computeConstraintMatrix();

  void setMinMaxX(const Matrix<Scalar, 2, 1>& minMaxX) {
    this->minMaxX = minMaxX;
  }

  void setMinMaxY(const Matrix<Scalar, 2, 1>& minMaxY) {
    this->minMaxY = minMaxY;
  }

  unsigned getNConstraints() { return cons.size(); }
  Matrix<Scalar, Dynamic, 1> getValue() { return zmp; }
private:
  //! Active chains for computation
  vector<bool> activeChains;

  //! Index of the support leg frame of reference
  LinkChains supportLeg;

  //! Known forces and moments acting at the torso for newton-euler
  Matrix<Scalar, 3, 1> initTorsoForces;
  Matrix<Scalar, 3, 1> initTorsoMoments;

  //! Constraint max/mins
  Matrix<Scalar, 2, 1> minMaxX;
  Matrix<Scalar, 2, 1> minMaxY;

  //! Constraint matrix
  vector<Scalar> cons;

  //! Zmp position
  Matrix<Scalar, 2, 1> zmp;

  //! Type of joint usage for all computations
  JointStateType type;

  //! Pointer to kinematics module
  boost::shared_ptr<KinematicsModule<Scalar> > kM;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
