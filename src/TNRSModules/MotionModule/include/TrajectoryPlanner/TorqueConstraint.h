/**
 * @file MotionModule/include/TrajectoryPlanner/TorqueConstraint.h
 *
 * This file declares the class TorqueConstraint
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
 * @class TorqueConstraint
 * @brief The class to define a torque constraint handler for nlopt based
 *   optimization problems
 */
template <typename Scalar>
class TorqueConstraint : public NloptConstraint<Scalar>
{
public:
  /**
   * Constructor.
   *
   * @param motionModule Pointer to base motion module
   */
  TorqueConstraint(
    MotionModule* motionModule,
    const LinkChains& supportLeg = LinkChains::lLeg,
    const JointStateType& type = JointStateType::sim,
    const vector<bool>& activeJoints = vector<bool>(toUType(Joints::count), true));

  /**
   * Default destructor for this class.
   */
  ~TorqueConstraint()
  {
  }

  void init();
  void update();
  vector<Scalar> computeConstraintMatrix();
  unsigned getNConstraints() { return nCons; }
  Matrix<Scalar, Dynamic, 1> getValue() { return torques; }
private:
  ///< Active joints for computation
  vector<bool> activeJoints;

  ///< Active chains based on active joints
  vector<bool> activeChains;

  ///< Index of the support leg frame of reference used for finding gravity vector
  LinkChains supportLeg;

  ///< Number of constraints
  unsigned nCons;

  ///< Constraint max/mins
  Matrix<Scalar, Dynamic, 1> minTorques;
  Matrix<Scalar, Dynamic, 1> maxTorques;

  ///< Constraint matrix
  vector<Scalar> cons;

  ///< Joint torques
  Matrix<Scalar, Dynamic, 1> torques;

  ///< Type of joint usage for all computations
  JointStateType type;

  ///< Pointer to kinematics module
  boost::shared_ptr<KinematicsModule<Scalar> > kM;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
