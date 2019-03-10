/**
 * @file MotionModule/include/KinematicsModule/MotionTask.h
 *
 * This file defines the struct MotionTask
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "MotionModule/include/KinematicsModule/JointStateType.h"
#include "MotionModule/include/MTypeHeader.h"
#include "Utils/include/HardwareIds.h"

using namespace std;
using namespace Eigen;

template <typename Scalar>
class KinematicsModule;

enum class MotionTaskType {
  posture,
  com,
  cartesian,
  contact,
  torso,
};

/**
 * @struct MotionTask
 * @brief A base class for all kinds of tasks to be provided to the task
 *   based ik solver
 */
template <typename Scalar>
class MotionTask
{
public:

  /**
   * Constructor
   * 
   * @param weight: MotionTask priority weight
   * @param gain: MotionTask gain
   * @param nResidual: Number of residual outputs
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param activeJoints: Active residual
   * @param km: Robot kinematics module
   * @param jsType: Type of joint state to be used for kinematic computation
   * @param taskType: Type of task
   */
  MotionTask(
    const Scalar& weight, 
    const Scalar& gain, 
    const unsigned& nResidual,
    const vector<bool>& activeJoints,
    const boost::shared_ptr<KinematicsModule<Scalar> >& km,
    const JointStateType& jsType,
    const MotionTaskType& taskType,
    const vector<bool>& activeResidual = vector<bool>());
  
  virtual ~MotionTask() {}
  
  /**
   * Computes the jacobian matrix for the given task
   * 
   * @returns task jacobian
   */
  virtual Matrix<Scalar, Dynamic, Dynamic>
    computeJacobian() = 0;
  
  /**
   * Computes the residual for the given task
   */
  virtual Matrix<Scalar, Dynamic, Dynamic>
    computeResidual(const Scalar& dt) = 0;
  
  /**
   * Computes the current cost based on residual
   * @param dt Time step for residual
   * @return task cost
   */
  Scalar computeCost(const Scalar& dt);

  /**
   * Computes the current cost based on already calculated residual
   * @return task cost
   */
  Scalar computeCost();
  
  /**
   * Checks whether the given task has any conflict with this task
   * @return true if the conflict exists
   */
  virtual bool checkConflict(const boost::shared_ptr<MotionTask<Scalar> >& task);

  //! Setters
  void setJSType(const JointStateType& jsType) { this->jsType = jsType; }
  void setGain(const Scalar& gain) { this->gain = gain; }
  void setWeight(const Scalar& weight) { this->weight = weight; }
  void setActiveJoints(const vector<bool>& activeJoints) { this->activeJoints = activeJoints; }
  void setActiveResidual(const vector<bool>& activeResidual) { this->activeResidual = activeResidual; }
  
  //! Getters
  Scalar getGain() const { return gain; }
  Scalar getWeight() const { return weight; }
  MotionTaskType getType() const { return taskType; }
  vector<bool> getActiveJoints() const { return activeJoints; }
  vector<bool> getActiveResidual() const { return activeResidual; }
  Matrix<Scalar, Dynamic, Dynamic> getJacobian();
  Matrix<Scalar, Dynamic, Dynamic> getResidual(const Scalar& dt);
  Matrix<Scalar, Dynamic, Dynamic> getResidual();
 
protected:  
  //! A pointer to robot kinematics
  boost::shared_ptr<KinematicsModule<Scalar> > km;
  
  //! Type of joint state to be used for kinematic computation
  JointStateType jsType;

  //! Number of degrees of freedom possible for this task
  unsigned nDof;

  //! Output residual size
  unsigned nResidual;

  //! Current task residual
  Matrix<Scalar, Dynamic, Dynamic> residual;

  //! Type of the task
  MotionTaskType taskType;

  //! Active degrees of freedom
  vector<bool> activeJoints;

  //! Active residual components
  vector<bool> activeResidual;
  
  //! MotionTask gain
  Scalar gain;
  
  //! MotionTask priority weight
  Scalar weight;
};
typedef boost::shared_ptr<MotionTask<MType> > MotionTaskPtr;

/**
 * @struct PostureTask
 * @brief A task that defines whole-body posture tracking
 */
template <typename Scalar>
class PostureTask : public MotionTask<Scalar>
{
public:

  /**
   * Constructor
   * @param targetJoints: Target joints for the required posture
   * @param weight: MotionTask priority weight
   * @param gain: MotionTask gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param jsType: Type of joint state to be used for kinematic computation
   */
  PostureTask(
    const Matrix<Scalar, Dynamic, 1>& targetJoints,
    const Scalar& weight,
    const Scalar& gain,
    const vector<bool>& activeJoints,
    const boost::shared_ptr<KinematicsModule<Scalar> >& km,
    const vector<bool>& activeResidual = vector<bool>(),
    const JointStateType& jsType = JointStateType::sim);

  /**
   * Checks whether the given task has any conflict with this posture task
   * @return true if the given task is also a posture task and has conflicting active
   *   joints
   */
  bool checkConflict(const boost::shared_ptr<MotionTask<Scalar> >& task);

  Matrix<Scalar, Dynamic, Dynamic> computeJacobian();

  Matrix<Scalar, Dynamic, Dynamic> computeResidual(const Scalar& dt);

  //! Setters
  void setTargetPosture(
    const Matrix<Scalar, Dynamic, 1>& targetJoints)
    { this->targetJoints = targetJoints; }

private:
  //! Target joints for the required posture
  Matrix<Scalar, Dynamic, 1> targetJoints;
};
typedef boost::shared_ptr<PostureTask<MType> > PostureTaskPtr;


/**
 * @struct ComTask
 * @brief A task that defines center of mass tracking
 */
template <typename Scalar>
class ComTask : public MotionTask<Scalar>
{
public:

  /**
   * Constructor
   * @param baseFrame: Frame of reference for center of mass tracking.
   * @param baseEE: Index of the base frame end-effector
   * @param targetCom: Target center of mass position from ref frame
   * @param weight: MotionTask priority weight
   * @param gain: MotionTask gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param jsType: Type of joint state to be used for kinematic computation
   */
  ComTask(
    const RobotFeet& baseFrame,
    const LegEEs& baseEE,
    const Matrix<Scalar, 3, 1>& targetCom,
    const Scalar& weight, 
    const Scalar& gain,
    const vector<bool>& activeJoints,
    const boost::shared_ptr<KinematicsModule<Scalar> >& km,
    const vector<bool>& activeResidual = vector<bool>(),
    const JointStateType& jsType = JointStateType::sim);
  
  Matrix<Scalar, Dynamic, Dynamic> computeJacobian();
  
  Matrix<Scalar, Dynamic, Dynamic> computeResidual(const Scalar& dt);
  
  /**
   * Checks whether the given task has any conflict with this com task
   * @return true if the given task is also a com task
   *   joints
   */
  bool checkConflict(const boost::shared_ptr<MotionTask<Scalar> >& task);

  //! Setters
  void setTargetCom(
    const Matrix<Scalar, 3, 1>& targetCom) 
    { this->targetCom = targetCom; }

  void setFirstStep(const bool& firstStep)
    { this->firstStep = firstStep; }

  void setBaseFrame(const RobotFeet& baseFrame)
    { this->baseFrame = baseFrame; }

private:
  //! Reference frame for com computations
  RobotFeet baseFrame;

  //! Index of base chain end-effector
  LegEEs baseEE;

  //! Target center of mass position
  Matrix<Scalar, 3, 1> targetCom;

  //! First step uses com estimate
  bool firstStep;
};
typedef boost::shared_ptr<ComTask<MType> > ComTaskPtr;

/**
 * @struct CartesianTask
 * @brief A task that defines the tracking of a limb end-effector
 *   in cartesian space
 */
template <typename Scalar>
class CartesianTask : public MotionTask<Scalar>
{
public:
  /**
   * Constructor
   * 
   * @param chainIndex: Index of the chain for which this task is 
   *   defined
   * @param endEffector: Transformation frame of the end-effector to be 
   *   moved
   * @param target: Target transformation to be reached by the 
   *   end-effector
   * @param weight: MotionTask priority weight
   * @param gain: MotionTask gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param jsType: Type of joint state to be used for kinematic computation
   * @param taskType: Type of task
   */
  CartesianTask(
    const LinkChains& chainIndex,
    const Matrix<Scalar, 4, 4> endEffector,
    const Matrix<Scalar, 4, 4> target,
    const Scalar& weight, 
    const Scalar& gain,
    const vector<bool>& activeJoints,
    const boost::shared_ptr<KinematicsModule<Scalar> >& km,
    const vector<bool>& activeResidual = vector<bool>(),
    const JointStateType& jsType = JointStateType::sim,
    const MotionTaskType& taskType = MotionTaskType::cartesian);
  
  Matrix<Scalar, Dynamic, Dynamic> computeJacobian();
  
  Matrix<Scalar, Dynamic, Dynamic> computeResidual(const Scalar& dt);
  /**
   * Checks whether the given task has any conflict with this cartesian task
   * @return true if the given task is also a cartesian task for the same chain
   */
  bool checkConflict(const boost::shared_ptr<MotionTask<Scalar> >& task);
  
  //! Setters
  void setTargetRel(
    const Matrix<Scalar, 4, 4>& target) 
    { this->target = this->km->getGlobalToBody() * target; }

  //! Setters
  void setTarget(const Matrix<Scalar, 4, 4>& target)
    { this->target = target; }

  void setChainIndex(const LinkChains& chainIndex)
    { this->chainIndex = chainIndex; }

  void setEndEffector(const Matrix<Scalar, 4, 4>& endEffector)
    { this->endEffector = endEffector; }

  //! Getters
  Matrix<Scalar, 4, 4> getTarget() { return target; }
  
protected:
  //! Index of the chain for which this task is defined
  LinkChains chainIndex;

  //! End effector transformation for this chain
  Matrix<Scalar, 4, 4> endEffector;

  //! Target end-effector position
  Matrix<Scalar, 4, 4> target;
};
typedef boost::shared_ptr<CartesianTask<MType> > CartesianTaskPtr;

/**
 * @struct ContactTask
 * @brief A task that defines the fixed end-effector contact for
 *   the given limb in cartesian space
 */
template <typename Scalar>
class ContactTask : public CartesianTask<Scalar>
{
public:
  /**
   * Constructor
   *
   * @param chainIndex: Index of the chain for which this task is
   *   defined
   * @param endEffector: Transformation frame of the end-effector to be
   *   moved
   * @param target: Target transformation to be reached by the
   *   end-effector
   * @param weight: MotionTask priority weight
   * @param gain: MotionTask gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param jsType: Type of joint state to be used for kinematic computation
   */
  ContactTask(
    const LinkChains& chainIndex,
    const Matrix<Scalar, 4, 4> endEffector,
    const Scalar& weight,
    const Scalar& gain,
    const vector<bool>& activeJoints,
    const boost::shared_ptr<KinematicsModule<Scalar> >& km,
    const vector<bool>& activeResidual = vector<bool>(),
    const JointStateType& jsType = JointStateType::sim);
};
typedef boost::shared_ptr<ContactTask<MType> > ContactTaskPtr;

/**
 * @struct TorsoTask
 * @brief A task that defines the tracking of desired torso pose
 */
template <typename Scalar>
class TorsoTask : public MotionTask<Scalar>
{
public:
  /**
   * Constructor
   *
   * @param baseFrame: Index of the base chain
   * @param baseEE: Index of the base end-effector
   * @param target: Target transformation to be reached by the
   *   torso
   * @param weight: MotionTask priority weight
   * @param gain: MotionTask gain
   * @param activeJoints: Active joint degrees of freedom for this task
   * @param km: Robot kinematics module
   * @param jsType: Type of joint state to be used for kinematic computation
   */
  TorsoTask(
    const RobotFeet& baseFrame,
    const LegEEs& baseEE,
    const Matrix<Scalar, 4, 4> target,
    const Scalar& weight,
    const Scalar& gain,
    const vector<bool>& activeJoints,
    const boost::shared_ptr<KinematicsModule<Scalar> >& km,
    const vector<bool>& activeResidual = vector<bool>(),
    const JointStateType& jsType = JointStateType::sim);

  Matrix<Scalar, Dynamic, Dynamic> computeJacobian();

  Matrix<Scalar, Dynamic, Dynamic> computeResidual(const Scalar& dt);

  /**
   * Checks whether the given task has any conflict with this torso task
   * @return true if the given task is also a torso task
   */
  bool checkConflict(const boost::shared_ptr<MotionTask<Scalar> >& task);

  //! Getters
  Matrix<Scalar, 4, 4> getTarget()
    { return target; }

  //! Setters
  void setTarget(
    const Matrix<Scalar, 4, 4>& target)
    { this->target = target; }

  void setBaseFrame(const RobotFeet& baseFrame)
    { this->baseFrame = baseFrame; }

  void setEndEffector(const Matrix<Scalar, 4, 4>& endEffector)
    { this->endEffector = endEffector; }

protected:
  //! Index of the base chain
  RobotFeet baseFrame;

  //! Index of base chain end-effector
  LegEEs baseEE;

  //! Transformation frame of the end-effector to be used as base frame
  Matrix<Scalar, 4, 4> endEffector;

  //! Target transformation to be reached by the torso
  Matrix<Scalar, 4, 4> target;
};
typedef boost::shared_ptr<TorsoTask<MType> > TorsoTaskPtr;
