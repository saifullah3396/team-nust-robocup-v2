/**
 * @file MotionModule/include/TrajectoryPlanner/TrajOptimizer.h
 *
 * This file declares the class TrajOptimizer
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/MTypeHeader.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/Solvers/NLOptimizer.h"

/**
 * @class TrajOptimizer
 * @brief The base class for optimization of joint trajectories
 */
template <typename Scalar>
class TrajOptimizer : public NLOptimizer
{
public:
  /**
   * Constructor for this class with given knots.
   */
  TrajOptimizer(
    MotionModule* motionModule,
    const LinkChains& chainIndex,
    const LinkChains& baseLeg);
  
  /**
   * Default destructor for this class.
   */
  virtual ~TrajOptimizer();
  
protected:
  //! Taken as either left leg or right leg. Used for zmp constraints
  //! The base leg is considered as the inertial frame of reference for 
  //! whole body motion.
  LinkChains baseLeg;
  LinkChains chainIndex;
  unsigned chainStart;
  unsigned chainSize;
  Scalar stepSize;
  //! Velocity bounds for the current chain.
  Matrix<Scalar, 1, Dynamic> velLimits;
  Scalar maxVelCutoff;

  KinematicsModulePtr kM;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

