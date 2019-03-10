/**
 * @file MotionModule/src/KinematicsModule/MotionTask.cpp
 *
 * This file defines the struct MotionTask
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include <iostream>
#include "Utils/include/MathsUtils.h"
#include "Utils/include/PrintUtils.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/JointStateType.h"
#include "MotionModule/include/KinematicsModule/TorsoState.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/KinematicsModule/ComState.h"
#include "Utils/include/DebugUtils.h"

using namespace std;

template <typename Scalar>
MotionTask<Scalar>::MotionTask(
  const Scalar& weight,
  const Scalar& gain,
  const unsigned& nResidual,
  const vector<bool>& activeJoints,
  const boost::shared_ptr<KinematicsModule<Scalar> >& km,
  const JointStateType& jsType,
  const MotionTaskType& taskType,
  const vector<bool>& activeResidual) :
  weight(weight),
  gain(gain),
  nResidual(nResidual),
  activeJoints(activeJoints),
  activeResidual(activeResidual),
  km(km),
  jsType(jsType),
  taskType(taskType),
  nDof(km->getNJoints())
{
  if (this->activeJoints.empty())
    this->activeJoints = vector<bool>(nDof, true);
  else
    ASSERT(activeJoints.size() == nDof);
  if (this->activeResidual.empty())
    this->activeResidual = vector<bool>(nResidual, true);
 else
    ASSERT(this->activeResidual.size() == nResidual);
  residual.resize(nResidual, 1);
  residual.setZero();
}

template <typename Scalar>
Scalar MotionTask<Scalar>::computeCost(const Scalar& dt) {
  return weight * gain * getResidual(dt).squaredNorm();
}

template <typename Scalar>
Scalar MotionTask<Scalar>::computeCost() {
  return weight * gain * getResidual().squaredNorm();
}

template <typename Scalar>
bool MotionTask<Scalar>::checkConflict(const boost::shared_ptr<MotionTask<Scalar> >& task) {
  return false;
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
MotionTask<Scalar>::getJacobian()
{
  Matrix<Scalar, Dynamic, Dynamic> jacobian = computeJacobian();
  for (size_t i = 0; i < nDof; ++i) {
    if (!activeJoints[i]) {
      jacobian.col(i).setZero();
    }
  }
  return jacobian;
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
MotionTask<Scalar>::getResidual()
{
  return residual;
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
MotionTask<Scalar>::getResidual(const Scalar& dt)
{
  residual = computeResidual(dt);
  for (size_t i = 0; i < nResidual; ++i) {
    if (!activeResidual[i]) {
      residual.row(i).setZero();
    }
  }
  return residual;
}

template class MotionTask<MType>;


template <typename Scalar>
PostureTask<Scalar>::PostureTask(
  const Matrix<Scalar, Dynamic, 1>& targetJoints,
  const Scalar& weight,
  const Scalar& gain,
  const vector<bool>& activeJoints,
  const boost::shared_ptr<KinematicsModule<Scalar> >& km,
  const vector<bool>& activeResidual,
  const JointStateType& jsType) :
  MotionTask<Scalar>(weight, gain, 24, activeJoints, km, jsType, MotionTaskType::posture, activeResidual),
  targetJoints(targetJoints)
{
}

template <typename Scalar>
bool PostureTask<Scalar>::checkConflict(const boost::shared_ptr<MotionTask<Scalar> >& task) {
  if (task->getType() == this->taskType) {
    auto taskAj = task->getActiveJoints();
    for (size_t i = 0; i < this->activeJoints.size(); ++i) {
      if (this->activeJoints[i] && taskAj[i])
        return true;
    }
  }
  return false;
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
PostureTask<Scalar>::computeJacobian()
{
  return Matrix<Scalar, Dynamic, Dynamic>::Identity(toUType(Joints::count), toUType(Joints::count));
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
PostureTask<Scalar>::computeResidual(const Scalar& dt)
{
  Matrix<Scalar, Dynamic, 1> actualJoints =
    this->km->getJointPositions(
      Joints::first, toUType(Joints::count), this->jsType);
  //LOG_INFO("posture diff:" << (targetJoints - actualJoints).transpose())
  return (targetJoints - actualJoints) / dt;
}

template class PostureTask<MType>;

template <typename Scalar>
ComTask<Scalar>::ComTask(
  const RobotFeet& baseFrame,
  const LegEEs& baseEE,
  const Matrix<Scalar, 3, 1>& targetCom,
  const Scalar& weight,
  const Scalar& gain,
  const vector<bool>& activeJoints,
  const boost::shared_ptr<KinematicsModule<Scalar> >& km,
  const vector<bool>& activeResidual,
  const JointStateType& jsType) :
  MotionTask<Scalar>(weight, gain, 3, activeJoints, km, jsType, MotionTaskType::com, activeResidual),
  baseFrame(baseFrame), baseEE(baseEE), targetCom(targetCom), firstStep(true)
{
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
ComTask<Scalar>::computeJacobian()
{
  //this->km->setGlobalBase(baseFrame, baseEE);
  return this->km->computeComJacobian(this->jsType);
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
ComTask<Scalar>::computeResidual(const Scalar& dt)
{
  Matrix<Scalar, 3, 1> diff;
  if (firstStep) {
    firstStep = false;
    diff =
      (targetCom - this->km->getComStateWrtFrame(static_cast<LinkChains>(baseFrame), toUType(LegEEs::footBase)).position);
  } else {
    diff =
      (targetCom - this->km->computeComWrtBase(static_cast<LinkChains>(baseFrame), toUType(LegEEs::footBase), this->jsType));
  }
  /*for (size_t i = 0; i < this->nResidual; ++i) {
    if (this->activeResidual[i]) {
      diff[i] = fabsf(diff[i]) < 5e-4 ? 0.0 : diff[i];
    }
  }
  //LOG_INFO("diff:" << diff.transpose())*/
  return diff / dt;
}

template <typename Scalar>
bool ComTask<Scalar>::checkConflict(const boost::shared_ptr<MotionTask<Scalar> >& task) {
  if (task->getType() == this->taskType) {
    return true;
  }
  return false;
}

template class ComTask<MType>;

template <typename Scalar>
CartesianTask<Scalar>::CartesianTask(
  const LinkChains& chainIndex,
  const Matrix<Scalar, 4, 4> endEffector,
  const Matrix<Scalar, 4, 4> target,
  const Scalar& weight,
  const Scalar& gain,
  const vector<bool>& activeJoints,
  const boost::shared_ptr<KinematicsModule<Scalar> >& km,
  const vector<bool>& activeResidual,
  const JointStateType& jsType,
  const MotionTaskType& taskType) :
  MotionTask<Scalar>(weight, gain, 6, activeJoints, km, jsType, taskType, activeResidual),
  endEffector(endEffector),
  target(target),
  chainIndex(chainIndex)
{
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
CartesianTask<Scalar>::computeJacobian()
{
  Matrix<Scalar, Dynamic, Dynamic> j;
  j.resize(6, this->nDof);
  j.setZero();
  auto ch = this->km->getLinkChain(chainIndex);
  j.block(0, ch->start, 6, ch->size) =
    this->km->computeLimbJ(chainIndex, endEffector, this->jsType);
  Matrix<Scalar, 6, Dynamic> baseJ = this->km->computeGlobalBaseJ(this->jsType);
  Matrix<Scalar, 3, 1> ee = (this->km->getGlobalToBody(this->jsType) *
    this->km->getForwardEffector(chainIndex, endEffector, this->jsType)).block(0, 3, 3, 1);
  //Matrix<Scalar, 3, 1> ee = (this->km->getGlobalToBody(this->jsType)).block(0, 3, 3, 1);
  auto baseChain = this->km->getGlobalBase();
  unsigned baseStart = baseChain->start;
  unsigned baseSize = baseChain->size;
  j.block(0, baseStart, 3, baseSize) = j.block(0, baseStart, 3, baseSize)
    -baseJ.block(0, 0, 3, baseSize) + MathsUtils::makeSkewMat(ee) * baseJ.block(3, 0, 3, baseSize);
  j.block(3, baseStart, 3, baseSize) = j.block(3, baseStart, 3, baseSize) - baseJ.block(3, 0, 3, baseSize);
  return j;
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
CartesianTask<Scalar>::computeResidual(const Scalar& dt)
{
  //LOG_INFO("Joints: " << this->km->getJointPositions(0, toUType(Joints::count), this->jsType).transpose() * 180 / M_PI);
  Matrix<Scalar, 4, 4> pos = this->km->getGlobalToBody(this->jsType) *
    this->km->getForwardEffector(chainIndex, endEffector, this->jsType);
  Matrix<Scalar, 6, 1> diff;
  diff.setZero();
  //LOG_INFO("supportToTorso:\n" << this->km->getGlobalToBody(this->jsType));
  //LOG_INFO("TorsotoEE:\n" << this->km->getForwardEffector(chainIndex, endEffector, this->jsType));
  //LOG_INFO("cartesian pos:\n" << pos);
  //LOG_INFO("cartesian target:\n" << target);
  diff.block(0, 0, 3, 1) = target.block(0, 3, 3, 1) - pos.block(0, 3, 3, 1);
  diff.block(3, 0, 3, 1) = MathsUtils::getOrientationDiff(pos, target);
  /*for (size_t i = 0; i < this->nResidual; ++i) {
    if (this->activeResidual[i]) {
      diff[i] = fabsf(diff[i]) < 1e-4 ? 0.0 : diff[i];
    }
  }*/
  //LOG_INFO("diff: " << diff.transpose())
  return diff / dt;
}

template <typename Scalar>
bool CartesianTask<Scalar>::checkConflict(
    const boost::shared_ptr<MotionTask<Scalar> >& task)
{
  if (task->getType() == this->taskType) {
    if (boost::static_pointer_cast<CartesianTask<Scalar> >(task)->chainIndex ==
        this->chainIndex)
      return true;
  }
  return false;
}

template class CartesianTask<MType>;


template <typename Scalar>
ContactTask<Scalar>::ContactTask(
  const LinkChains& chainIndex,
  const Matrix<Scalar, 4, 4> endEffector,
  const Scalar& weight,
  const Scalar& gain,
  const vector<bool>& activeJoints,
  const boost::shared_ptr<KinematicsModule<Scalar> >& km,
  const vector<bool>& activeResidual,
  const JointStateType& jsType) :
  CartesianTask<Scalar>(
    chainIndex,
    endEffector,
    Matrix<Scalar, 4, 4>::Identity(),
    weight,
    gain,
    activeJoints,
    km,
    activeResidual,
    jsType,
    MotionTaskType::contact)
{
  this->target = // Get first position using actual states
    this->km->getGlobalToBody() *
    this->km->getForwardEffector(chainIndex, endEffector, JointStateType::actual);
}
template class ContactTask<MType>;


template <typename Scalar>
TorsoTask<Scalar>::TorsoTask(
  const RobotFeet& baseFrame,
  const LegEEs& baseEE,
  const Matrix<Scalar, 4, 4> target,
  const Scalar& weight,
  const Scalar& gain,
  const vector<bool>& activeJoints,
  const boost::shared_ptr<KinematicsModule<Scalar> >& km,
  const vector<bool>& activeResidual,
  const JointStateType& jsType) :
  MotionTask<Scalar>(weight, gain, 6, activeJoints, km, jsType, MotionTaskType::torso, activeResidual),
  endEffector(this->km->getEndEffector(static_cast<LinkChains>(baseFrame), toUType(baseEE))),
  target(target),
  baseFrame(baseFrame)
{
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
TorsoTask<Scalar>::computeJacobian()
{
  Matrix<Scalar, 6, Dynamic> j;
  j.resize(6, this->nDof);
  j.setZero();
  auto ch = this->km->getLinkChain(LinkChains::lLeg);
  j.block(0, ch->start, 6, ch->size) =
    -this->km->computeLimbJ(LinkChains::lLeg, endEffector, this->jsType).block(0, 0, 6, ch->size);
  ch = this->km->getLinkChain(LinkChains::rLeg);
  j.block(0, ch->start, 6, ch->size) =
    -this->km->computeLimbJ(LinkChains::rLeg, endEffector, this->jsType).block(0, 0, 6, ch->size);
  return j;
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic>
TorsoTask<Scalar>::computeResidual(const Scalar& dt)
{
  Matrix<Scalar, 4, 4> pose =
    MathsUtils::getTInverse(
      this->km->getForwardEffector(
        static_cast<LinkChains>(baseFrame), endEffector, this->jsType));
  Matrix<Scalar, 6, 1> diff;
  diff.setZero();
  diff.block(0, 0, 3, 1) = target.block(0, 3, 3, 1) - pose.block(0, 3, 3, 1);
  diff.block(3, 0, 3, 1) = MathsUtils::getOrientationDiff(pose, target);
  return diff / dt;
}

template <typename Scalar>
bool TorsoTask<Scalar>::checkConflict(
  const boost::shared_ptr<MotionTask<Scalar> >& task)
{
  if (task->getType() == this->taskType) {
    return true;
  }
  return false;
}

template class TorsoTask<MType>;
