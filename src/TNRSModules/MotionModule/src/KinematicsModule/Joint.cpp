/**
 * @file MotionModule/src/KinematicsModule/Joint.cpp
 *
 * This file implements the structs JointState and Joint
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include <boost/make_shared.hpp>
#include "MotionModule/include/KinematicsModule/JointEstimator.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/LinkInfo.h"
#include "MotionModule/include/MTypeHeader.h"
#include "Utils/include/Constants.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/JsonUtils.h"
#include "Utils/include/HardwareIds.h"

template<typename Scalar>
DHParams<Scalar>::DHParams(
  const Scalar& a,
  const Scalar& alpha,
  const Scalar& d,
  const Scalar& theta) :
  a(a),
  alpha(alpha),
  d(d),
  theta(theta)
{
  calpha = cos(alpha);
  salpha = sin(alpha);
}
template struct DHParams<MType>;

template<typename Scalar>
ActualJointState<Scalar>::ActualJointState(
  const Joints& jointIndex,
  const Scalar& initPosition,
  const Scalar& cycleTime)
{
  Matrix<Scalar, 2, 1> initState;
  initState << initPosition, 0.0;
  _estimator = boost::shared_ptr<JointEstimator<Scalar>>(new JointEstimator<Scalar>());
  _estimator->init(jointIndex, initState, cycleTime);
}

template<typename Scalar>
void ActualJointState<Scalar>::update(const Scalar& sensedPosition)
{
  this->_sensedPosition = sensedPosition;
  _estimator->update(sensedPosition);
}

template<typename Scalar>
void ActualJointState<Scalar>::setPosition(const Scalar& position)
  { _estimator->setState(position, 0); }

template<typename Scalar>
void ActualJointState<Scalar>::setVelocity(const Scalar& velocity)
  { _estimator->setState(velocity, 1); }

template<typename Scalar>
void ActualJointState<Scalar>::setAccel(const Scalar& accel)
  { _estimator->setInput(accel); }

template<typename Scalar>
const Scalar& ActualJointState<Scalar>::position()
  { return _estimator->getState()[0]; }

template<typename Scalar>
const Scalar& ActualJointState<Scalar>::velocity()
  { return _estimator->getState()[1]; }

template<typename Scalar>
const Scalar& ActualJointState<Scalar>::accel()
  { return _estimator->getInput()[0]; }

template<typename Scalar>
const Scalar& ActualJointState<Scalar>::sensedPosition()
  { return _sensedPosition; }

template<typename Scalar>
const boost::shared_ptr<JointEstimator<Scalar>>&
ActualJointState<Scalar>::estimator()
  { return _estimator; }

template struct ActualJointState<MType>;

template<typename Scalar>
SimJointState<Scalar>::SimJointState(
  const Scalar& initPosition) : _position(initPosition)
{}

template<typename Scalar>
void SimJointState<Scalar>::setPosition(const Scalar& position)
  { this->_position = position; }

template<typename Scalar>
void SimJointState<Scalar>::setVelocity(const Scalar& velocity)
  { this->_velocity = velocity; }

template<typename Scalar>
void SimJointState<Scalar>::setAccel(const Scalar& accel)
  { this->_accel = accel; }

template<typename Scalar>
const Scalar& SimJointState<Scalar>::position()
  { return _position; }

template<typename Scalar>
const Scalar& SimJointState<Scalar>::velocity()
  { return _velocity; }

template<typename Scalar>
const Scalar& SimJointState<Scalar>::accel()
  { return _accel; }

template struct SimJointState<MType>;

template<typename Scalar>
Joint<Scalar>::Joint(
  const Joints& jointIndex,
  const Scalar& maxPosition,
  const Scalar& minPosition,
  const Scalar& maxVelocity,
  DHParams<Scalar>* dhParams,
  const Scalar& initPosition,
  const Scalar& cycleTime) :
  name(Constants::jointNames[toUType(jointIndex)]),
  maxPosition(maxPosition),
  minPosition(minPosition),
  maxVelocity(maxVelocity),
  dhParams(dhParams)
{
  states.resize(static_cast<int>(JointStateType::count));
  states[static_cast<int>(JointStateType::actual)] =
    boost::shared_ptr<ActualJointState<Scalar> >(
      new ActualJointState<Scalar>(jointIndex, initPosition, cycleTime));
  makeDHMatrix(states[static_cast<int>(JointStateType::actual)]->trans);
  states[static_cast<int>(JointStateType::sim)] =
    boost::shared_ptr<SimJointState<Scalar> >(
      new SimJointState<Scalar>(initPosition));
  makeDHMatrix(states[static_cast<int>(JointStateType::sim)]->trans);
}

template<typename Scalar>
Joint<Scalar>::~Joint()
{
  delete dhParams;
  dhParams = NULL;
}

template<typename Scalar>
void Joint<Scalar>::makeDHMatrix(Matrix<Scalar, 4, 4>& mat)
{
  mat(0, 2) = 0;
  mat(0, 3) = dhParams->a;
  mat(1, 2) = -dhParams->salpha;
  mat(1, 3) = -dhParams->salpha * dhParams->d;
  mat(2, 2) = dhParams->calpha;
  mat(2, 3) = dhParams->calpha * dhParams->d;
}

template<typename Scalar>
void Joint<Scalar>::makeDHMatrixSym()
{
  #ifdef ALLOW_SYMBOLIC_COMPUTATIONS
  //! Symbolic transformation matrix for this joint
  symTrans =
    DenseMatrix(
      4, 4,
      {number(1), number(0), number(0), number(0),
       number(0), number(1), number(0), number(0),
       number(0), number(0), number(1), number(0),
       number(0), number(0), number(0), number(1)});
  #endif
  #ifdef ALLOW_SYMBOLIC_COMPUTATIONS
  symTrans.set(0, 2, number(0));
  symTrans.set(0, 3, number(dhParams->a));
  symTrans.set(1, 2, number(-dhParams->salpha));
  symTrans.set(1, 3, number(-dhParams->salpha * dhParams->d));
  symTrans.set(2, 2, number(dhParams->calpha));
  symTrans.set(2, 3, number(dhParams->calpha * dhParams->d));
  RCP<const Basic> symct = cos(symPos);
  RCP<const Basic> symst = sin(symPos);
  symTrans.set(0, 0, symct);
  symTrans.set(0, 1, mul(number(-1), symst));
  symTrans.set(1, 0, mul(symst, number(dhParams->calpha)));
  symTrans.set(1, 1, mul(symct, number(dhParams->calpha)));
  symTrans.set(2, 0, mul(symst, number(dhParams->salpha)));
  symTrans.set(2, 1, mul(symct, number(dhParams->salpha)));
  #endif
}

template<typename Scalar>
void Joint<Scalar>::updateDHMatrix(
  Matrix<Scalar, 4, 4>& mat,
  const Scalar& theta)
{
  auto ct = cos(theta);
  auto st = sin(theta);
  mat(0, 0) = ct;
  mat(0, 1) = -st;
  mat(1, 0) = st * dhParams->calpha;
  mat(1, 1) = ct * dhParams->calpha;
  mat(2, 0) = st * dhParams->salpha;
  mat(2, 1) = ct * dhParams->salpha;
}

template<typename Scalar>
const Matrix<Scalar, 4, 4>& Joint<Scalar>::computeLinkTrans(const JointStateType& type)
{
  updateDHMatrix(
    states[(unsigned)type]->trans,
    states[(unsigned)type]->position() + dhParams->theta
  );
  return states[(unsigned)type]->trans;
}

template<typename Scalar>
void Joint<Scalar>::setTransInBase(
  const Matrix<Scalar, 4, 4> T,
  const JointStateType& type)
{
  states[(unsigned)type]->transInBase = T;
  states[(unsigned)type]->posInBase = T.template block<3, 1>(0, 3);
  states[(unsigned)type]->zInBase = T.template block<3, 1>(0, 2);
  states[(unsigned)type]->comInBase = (T * link->com).template block<3, 1>(0, 0);
}

template<typename Scalar>
void Joint<Scalar>::logJointState(Json::Value& root, const Scalar& time)
{
  auto cmd = boost::static_pointer_cast<ActualJointState<Scalar> >(
    states[static_cast<int>(JointStateType::actual)])->estimator()->getCmd();
  auto sensed = boost::static_pointer_cast<ActualJointState<Scalar> >(
    states[static_cast<int>(JointStateType::actual)])->sensedPosition();
  auto position = states[static_cast<int>(JointStateType::actual)]->position();
  auto velocity = states[static_cast<int>(JointStateType::actual)]->velocity();
  auto accel = states[static_cast<int>(JointStateType::actual)]->accel();
  Json::Value state;
  JSON_ASSIGN(state, "time", time);
  JSON_ASSIGN(state, "cmd", cmd);
  JSON_ASSIGN(state, "sensed", sensed);
  JSON_ASSIGN(state, "position", position);
  JSON_ASSIGN(state, "velocity", velocity);
  JSON_ASSIGN(state, "accel", accel);
  JSON_APPEND(root[name], "state", state);
}
template struct Joint<MType>;
