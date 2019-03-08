/**
 * @file MotionModule/TrajectoryPlanner/CbOptimizer.h
 *
 * This file declares the class CbOptimizer
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once

#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/TrajectoryPlanner/TrajOptimizer.h"
#include "MotionModule/include/TrajectoryPlanner/NloptConstraint.h"
#include "Utils/include/DebugUtils.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/Splines/CubicSpline.h"

/**
 * @class CbOptimizer
 * @brief The class to generate optimize cubic spline based joint
 *   trajectories.
 */
template <typename Scalar>
class CbOptimizer : public TrajOptimizer<Scalar>
{
public:
  /**
   * Constructor.
   *
   * @param motionModule Pointer to base motion module 
   * @param chainIndex leg chain index according to kinematics module
   * @param chainIndex base leg chain index according to kinematics module 
   *   if balancing is needed
   * @param cb The cubic spline object which is to be optimized.
   * @param innerPointsPerKnot Number of inner points to be used for evaluation
   *   for each knot
   */
  CbOptimizer(
    MotionModule* motionModule,
    const LinkChains& chainIndex,
    const LinkChains& baseLeg,
    CubicSpline<Scalar>* cb,
    const unsigned& innerPointsPerKnot = 20) :
    TrajOptimizer<Scalar>(motionModule, chainIndex, baseLeg),
    cb(cb),
    innerPointsPerKnot(innerPointsPerKnot)
  {
    ASSERT(this->chainSize == this->cb->getSplineDim());
  }
  
  /**
   * Constructor.
   *
   * @param motionModule Pointer to base motion module 
   * @param chainIndex leg chain index according to kinematics module
   * @param chainIndex base leg chain index according to kinematics module 
   *   if balancing is needed
   * @param endEffector endeffector transformation matrix from chain end
   * @param eeVelMax maximum velocity constraints on end-effector
   * @param cb The cubic spline object which is to be optimized.
   * @param innerPointsPerKnot Number of inner points to be used for evaluation
   *   for each knot
   */
  CbOptimizer(
    MotionModule* motionModule,
    const LinkChains& chainIndex,
    const LinkChains& baseLeg,
    const Matrix<Scalar, Dynamic, Dynamic>& endEffector,
    const Matrix<Scalar, Dynamic, 1>& eeVelMax,
    CubicSpline<Scalar>* cb,
    const unsigned& innerPointsPerKnot = 10) :
    TrajOptimizer<Scalar>(motionModule, chainIndex, baseLeg),
    cb(cb),
    innerPointsPerKnot(innerPointsPerKnot),
    endEffector(endEffector)
  {
    ASSERT(this->chainSize == this->cb->getSplineDim());
  }
  
  /**
   * Default destructor for this class.
   */
  ~CbOptimizer()
  {
  }

  /**
   * Sets the cb spline object.
   */ 
  void setCB(CubicSpline<Scalar>* cb) 
  { 
    this->cb = cb;
    ASSERT(this->chainSize == this->cb->getSplineDim());
  }

  /**
   * Optimizes the knots for minimum time under given constraints.
   */
  void optDef();

  /**
   * @brief Plots the magnitudes of constrained values
   * @param innerPointsPerKnot plotting resolution
   * @param logsPath base logs path
   */
  void logConstraints(
    const unsigned& innerPointsPerKnot,
    const Scalar& startTime,
    const string& logsPath,
    const bool& reset = false);

  /**
   * Adds a constraint to the constraints vector
   */
  void addConstraint(boost::shared_ptr<NloptConstraint<Scalar>> cons) {
    constraints.push_back(cons);
  }

private:
  /**
   * NLOPT based function for defining inequality constraints
   */
  void
  ineqConstraints(unsigned nCons, double *result, unsigned nVars,
    const double* knots, double* grad, void* data);
    
  /**
   * Evaluates the minimum time objective function
   */
  double
  costFunction(const vector<double> &knots, vector<double> &grad, void *data);

  /**
   * Computes dynamic constraints such as zmp or torque using inverse dynamics.
   */  
  vector<Scalar> computeDynCons(
    const Matrix<Scalar, Dynamic, 1>& times,
    const vector<Matrix<Scalar, Dynamic, Dynamic> >& coeffs);

  CubicSpline<Scalar>* cb;

  //! Vector of constraints
  vector<boost::shared_ptr<NloptConstraint<Scalar>> > constraints;

  //! Matrix defining the end-effector transformation frame from chain end
  Matrix<Scalar, 4, 4> endEffector;
  
  //! Number of inner points to be used for evaluation for each knot
  unsigned innerPointsPerKnot;

  //! Total number of inner points at which constraints are evaluated
  unsigned nInnerPoints;

  Matrix<Scalar, Dynamic, 1> timesSeqU, timesSeqU2, timesSeqU3, timesSeqL, timesSeqL2, timesSeqL3;
  Matrix<Scalar, Dynamic, Dynamic> pos, vel, acc;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

