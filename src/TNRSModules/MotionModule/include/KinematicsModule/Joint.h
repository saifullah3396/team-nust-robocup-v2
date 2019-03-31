/**
 * @file MotionModule/include/KinematicsModule/Joint.h
 *
 * This file defines the structs JointState and Joint
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include <Eigen/Dense>
#include <string>
#include <boost/make_shared.hpp>
#include "Utils/include/JsonUtils.h"
#include "MotionModule/include/KinematicsModule/JointStateType.h"

enum class Joints : unsigned int;
template<typename Scalar>
class JointEstimator;
template<typename Scalar>
class LinkInfo;

//#ifdef MODULE_IS_REMOTE
//#define ALLOW_SYMBOLIC_COMPUTATIONS
//#endif

#ifdef ALLOW_SYMBOLIC_COMPUTATIONS
#include <symengine/matrix.h>
#include <symengine/add.h>
#include <symengine/pow.h>
#include <symengine/symengine_exception.h>
#include <symengine/visitor.h>
using SymEngine::symbol;
using SymEngine::number;
using SymEngine::DenseMatrix;
using SymEngine::Symbol;
using SymEngine::sin;
using SymEngine::cos;
using SymEngine::RCP;
using SymEngine::Basic;
#endif
/**
 * @struct DHParams
 * @brief Definition of dh parameters for revolute joints
 */
template<typename Scalar>
struct DHParams
{
/**
   * Constructor
   *
   * @param a: Link length
   * @param alpha: Link twist
   * @param d: Joint offset
   * @param theta: Joint angle offset
   */
  DHParams(
    const Scalar& a,
    const Scalar& alpha,
    const Scalar& d,
    const Scalar& theta);

  //! Link length
  Scalar a;

  //! Link twist
  Scalar alpha;

  //! Joint offset
  Scalar d;

  //! Joint angle offset
  Scalar theta;

  //! Cosine of alpha
  Scalar calpha;

  //! Sine of alpha
  Scalar salpha;
};

/**
 * @struct JointState
 * @brief Definition of a joint state
 */
template<typename Scalar>
struct JointState
{
  /**
   * @brief JointState Constructors
   */
  JointState() = default;
  JointState(const JointState&) = default;
  JointState(JointState&&) = default;
  JointState& operator=(const JointState&) & = default;
  JointState& operator=(JointState&&) & = default;

  /**
   * @brief ~JointState Destructor
   */
  virtual ~JointState() {}

  //! Transformation matrix for this joint
  Eigen::Matrix<Scalar, 4, 4> trans = {Eigen::Matrix<Scalar, 4, 4>::Identity()};

  //! Transformation matrix of the current joint in base frame of
  //! the chain
  Eigen::Matrix<Scalar, 4, 4> transInBase = {Eigen::Matrix<Scalar, 4, 4>::Identity()};

  //! Joint position in base;
  Eigen::Matrix<Scalar, 3, 1> posInBase = {Eigen::Matrix<Scalar, 3, 1>::Zero()};

  //! A vector representing the joint rotation axis in base frame;
  Eigen::Matrix<Scalar, 3, 1> zInBase = {Eigen::Matrix<Scalar, 3, 1>::Zero()};

  //! A vector representing the center of mass of this link in base frame;
  Eigen::Matrix<Scalar, 3, 1> comInBase = {Eigen::Matrix<Scalar, 3, 1>::Zero()};

  virtual void setPosition(const Scalar& position) = 0;
  virtual void setVelocity(const Scalar& velocity) = 0;
  virtual void setAccel(const Scalar& accel) = 0;
  virtual const Scalar& position() = 0;
  virtual const Scalar& velocity() = 0;
  virtual const Scalar& accel() = 0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @struct JointState
 * @brief Actual state of a joint
 */
template<typename Scalar>
struct ActualJointState : public JointState<Scalar>
{
  /**
   * @brief ActualJointState Constructor
   * @param jointIndex Index of the joint
   * @param initPosition Initial joint position
   * @param cycleTime Update cycle time
   */
  ActualJointState(
    const Joints& jointIndex,
    const Scalar& initPosition,
    const Scalar& cycleTime);

  /**
   * @brief update Updates the joint estimator model with the current sensed
   *   position of the joint
   * @param sensedPosition Sensor measured joint position
   */
  void update(const Scalar& sensedPosition);

  //! Setters
  void setPosition(const Scalar& position) final;
  void setVelocity(const Scalar& velocity) final;
  void setAccel(const Scalar& accel) final;

  //! Getters
  const Scalar& position() final;
  const Scalar& velocity() final;
  const Scalar& accel() final;
  const Scalar& sensedPosition();
  const boost::shared_ptr<JointEstimator<Scalar> >& estimator();

private:
  //! Sensor position
  Scalar _sensedPosition;

  //! Joint state estimator
  boost::shared_ptr<JointEstimator<Scalar> > _estimator;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @struct SimJointState
 * @brief Simulated state of a joint
 */
template<typename Scalar>
struct SimJointState : public JointState<Scalar>
{
  /**
   * @brief SimJointState Constructor
   * @param initPosition Initial joint position
   */
  SimJointState(const Scalar& initPosition);

  //! Setters
  void setPosition(const Scalar& position) final;
  void setVelocity(const Scalar& velocity) final;
  void setAccel(const Scalar& accel) final;

  //! Getters
  const Scalar& position() final;
  const Scalar& velocity() final;
  const Scalar& accel() final;

private:
  Scalar _position = {0.0}; //! Joint position
  Scalar _velocity = {0.0}; //! Joint velocity
  Scalar _accel = {0.0}; //! Joint acceleration

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @struct Joint
 * @brief Definition of a joint
 */
template<typename Scalar>
struct Joint
{
  /**
   * Constructor
   */
  Joint(
    const Joints& jointIndex,
    const Scalar& maxPosition,
    const Scalar& minPosition,
    const Scalar& maxVelocity,
    DHParams<Scalar>* dhParams,
    const Scalar& initPosition,
    const Scalar& cycleTime);

  /**
   * Destructor
   */
  ~Joint();

  /**
   * Sets up the link transformation matrix as a dh matrix with
   * unchanging variables set up on start
   *
   * @param mat: The matrix to be updated
   */
  void makeDHMatrix(Eigen::Matrix<Scalar, 4, 4>& mat);

  void makeDHMatrixSym();

  /**
   * Updates the link transformation matrix for the given angle
   *
   * @param mat: The matrix to be updated
   * @param theta: The given angle
   */
  void updateDHMatrix(
    Eigen::Matrix<Scalar, 4, 4>& mat,
    const Scalar& theta);

  const Eigen::Matrix<Scalar, 4, 4>& computeLinkTrans(const JointStateType& type);

  void setTransInBase(
    const Eigen::Matrix<Scalar, 4, 4> T,
    const JointStateType& type);

  void logJointState(Json::Value& root, const Scalar& time);

  //! Joint name
  std::string name;

  //! Upper limit of joint position
  Scalar maxPosition;

  //! Upper limit of joint position
  Scalar minPosition;

  //! Joint velocity limit
  Scalar maxVelocity;

  #ifdef ALLOW_SYMBOLIC_COMPUTATIONS
  //! Symbolic transformation matrix for this joint
  SymEngine::DenseMatrix symTrans;
  RCP<const Symbol> symPos;
  RCP<const Symbol> symVel;
  RCP<const Symbol> symAcc;
  #endif

  //! Dh parameters
  DHParams<Scalar>* dhParams;

  //! Joint state vectors
  std::vector<boost::shared_ptr<JointState<Scalar> > > states;

  //! Associated link
  boost::shared_ptr<LinkInfo<Scalar> > link;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
