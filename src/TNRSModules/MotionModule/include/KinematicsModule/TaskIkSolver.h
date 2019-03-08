/**
 * @file MotionModule/include/KinematicsModule/TaskIkSolver.h
 *
 * This file declares the class TaskIkSolver
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once
#include <qpOASES.hpp>
#include "Utils/include/MathsUtils.h"
#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/KinematicsModule/JointStateType.h"

using namespace std;

namespace qpOASES {
  class SQProblem;
}
template <typename Scalar>
class KinematicsModule;

/**
 * @class TaskIkSolver
 * @brief A class to solve for the best end-effector for a kick with 
 *   maximimum product of virtual mass and possible velocity hence 
 *   momentum.
 */
template <typename Scalar>
class TaskIkSolver
{
public:
  /**
   * Constructor
   * 
   * @param km: Pointer to kinematics module 
   */
   TaskIkSolver(
     const boost::shared_ptr<KinematicsModule<Scalar> >& km,
     const unsigned& maxIterations,
     const vector<bool>& activeJoints,
     const bool& addFinalCheck,
     const Scalar& dt = 5e-3,
     const Scalar& costThres = 1e-10,
     const Scalar& costVarThres = 1e-5);
  
  /**
   * Destructor
   */
  ~TaskIkSolver();

  /**
  * Initiates the ik solver
  */
  void init();

  /**
   * Solve the ik problem using qpoases
   *
   * @maxIterations: Max allowable iterations to solve the ik
   *
   * @return joint values
   */
  Eigen::Matrix<Scalar, Dynamic, 1> solve(const unsigned& maxIterations);
  
  /**
   * Update step for the ik solver
   *
   * @return true if a solution is found
   */ 
  bool step(Eigen::Matrix<Scalar, Dynamic, 1>& qd, const bool& final = false);
  
  /**
   * Computes the total integrated cost for all the tasks and returns it
   * 
   * @return Scalar
   */ 
  Scalar computeCost();
  
  /**
   * Adds a task to the vector of tasks to be solved
   * 
   * @param task: Added task
   */ 
  void addTask(const MotionTaskPtr& taskPtr)
    { this->tasks.push_back(taskPtr); }

  /**
   * Resets the solver by removing previously added tasks
   */
  void reset(const bool& addFinalCheck)
  {
    this->addFinalCheck = addFinalCheck;
    this->tasks.clear();
    this->initiated = false;
  }

  /**
   * Set active joints
   *
   * @param activeJoints
   */
  void setActiveJoints(const vector<bool>& activeJoints)
    { this->activeJoints = activeJoints; }

  /**
   * Set max velocity limit
   *
   * @param maxVelocityLimitGain: Ratio of maximum joint velocities to be used
   */
  void setMaxVelocityLimitGain(const double& maxVelocityLimitGain)
    { this->maxVelocityLimitGain = maxVelocityLimitGain; }

  /**
   * @brief setMaxJointLimit Sets the maximum joint limit
   * @param limit Maximum allowable position value
   * @param index Joint index
   */
  void setMaxJointLimit(const Scalar& limit, const unsigned& index) {
    maxP[index] = limit;
  }

  /**
   * @brief setMinJointLimit Sets the minimum joint limit
   * @param limit Minimum allowable position value
   * @param index Joint index
   */
  void setMinJointLimit(const Scalar& limit, const unsigned& index) {
    minP[index] = limit;
  }

  /**
   * @brief setMaxVelocityLimit Sets the maximum joint velocity limit
   * @param limit Maximum allowable velocity value
   * @param index Joint index
   */
  void setMaxVelocityLimit(const Scalar& limit, const unsigned& index) {
    maxV[index] = limit;
  }

  /**
   * @brief setMinVelocityLimit Sets the minimum joint velocity limit
   * @param limit Minimum allowable velocity value
   * @param index Joint index
   */
  void setMinVelocityLimit(const Scalar& limit, const unsigned& index) {
    minV[index] = limit;
  }

  void resetJointLimits();

private:
  //! Pointer to SQP problem
  qpOASES::SQProblem* qp;
  
  //! QP Problem matrices
  Eigen::Matrix<Scalar, Dynamic, Dynamic, RowMajor> P; // 1/2 x' P x
  Eigen::Matrix<Scalar, Dynamic, 1> v; // -K x'v
  
  //! QP Constraint matrices
  Eigen::Matrix<Scalar, Dynamic, Dynamic, RowMajor> G; // Gx <= h
  Eigen::Matrix<Scalar, Dynamic, 1> h; // Gx <= h

  //! Velocity constraints for each dof
  Eigen::Matrix<Scalar, Dynamic, 1> maxV, minV, maxP, minP, maxVIter, minVIter;

  //! Type of joints to be used for dof computations
  JointStateType type;

  //! Number of active joints
  unsigned nDof;
  
  //! Contains a list of booleans telling if the joint of given index is active or not
  vector<bool> activeJoints;

  //! Minmax dof positional limit gain
  double dofPLimitGain;

  //! Ratio of maximum velocity to be used
  double maxVelocityLimitGain;

  //! Division of dofPLimitGain to dt
  Scalar dofPLimitGainOverdt;

  //! Time step size for solving the ik
  Scalar dt;

  //! Maximum number of iterations to solve ik
  unsigned maxIterations;
  
  //! Whether the solver has been initiated once
  bool initiated;

  //! If cost is below this value then stop
  Scalar costThres;
  
  //! Threshold for checking if cost variation is below this then stop
  Scalar costVarThres;
  
  //! A vector of tasks to be solved
  vector<MotionTaskPtr> tasks;

  //! Final task
  boost::shared_ptr<PostureTask<Scalar> > finalPostureTask;

  //! Whether to use the final posture task
  bool addFinalCheck;
  
  //! Kinematics module
  boost::shared_ptr<KinematicsModule<Scalar> > km;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
