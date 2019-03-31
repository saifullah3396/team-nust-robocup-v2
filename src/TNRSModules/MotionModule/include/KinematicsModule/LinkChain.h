/**
 * @file MotionModule/include/KinematicsModule/LinkChain.h
 *
 * This file defines the struct LinkChain
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include <boost/make_shared.hpp>
#include "MotionModule/include/KinematicsModule/JointStateType.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/HardwareIds.h"

/**
 * @struct JacobianInfo
 * @brief A struct that holds information about a link jacobian
 */
template<typename Scalar>
struct JacobianInfo
{
  JacobianInfo(
    const unsigned& chainSize) : solved(false), eeIndex(0)
  {
    jacobian.resize(6, chainSize);
  }

  bool getJacobian(
    Matrix<Scalar, 6, Dynamic>& jacobian,
    const unsigned& eeIndex)
  {
    if (this->solved &&
        this->eeIndex == eeIndex)
    {
      jacobian = this->jacobian;
      return true;
    }
    return false;
  }

  void setJacobian(
    const Matrix<Scalar, 6, Dynamic>& jacobian,
    const unsigned& eeIndex) {
    this->jacobian = jacobian;
    this->eeIndex = eeIndex;
    this->solved = true;
  }

  void reset()
  {
    this->solved = false;
  }

  bool solved;
  unsigned eeIndex;
  Matrix<Scalar, 6, Dynamic> jacobian;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @struct LinkChain
 * @brief A struct that holds information about a links chain
 */
template<typename Scalar>
struct LinkChain
{
  /**
   * Constructor
   */
  LinkChain(
    const LinkChains& index,
    const HardwareIds& start,
    const HardwareIds& size,
    const Scalar& mass = Scalar(0),
    const Matrix<Scalar, 4, 4>& startT = Matrix<Scalar, 4, 4>::Identity(),
    const Matrix<Scalar, 4, 4>& endT = Matrix<Scalar, 4, 4>::Identity()) :
    index(index),
    start(toUType(start)),
    size(toUType(size)),
    end(toUType(start) + toUType(size)),
    mass(mass),
    startT(startT),
    endT(endT)
  {
    for (size_t i = 0; i < toUType(JointStateType::count); ++i) {
      jacobianInfo.push_back(
        boost::shared_ptr<JacobianInfo<Scalar> >(new JacobianInfo<Scalar>(toUType(size))));
    }
  }

  //! Index of this chain
  LinkChains index;

  //! LinkChain start index
  unsigned start;

  //! LinkChain size
  unsigned size;

  //! LinkChain size
  unsigned end;

  //! Total mass of the chain
  Scalar mass;

  //! Total masses of all the chains
  static Scalar totalChainsMass;

  //! LinkChain start transformation matrix
  Matrix<Scalar, 4, 4> startT;

  //! LinkChain end transformation matrix
  Matrix<Scalar, 4, 4> endT;

  //! LinkChain end effector definitions
  vector<Matrix<Scalar, 4, 4> > endEffectors;

  //! Vector of structs that hold information about current link jacobian
  vector<boost::shared_ptr<JacobianInfo<Scalar> > > jacobianInfo;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename Scalar> Scalar LinkChain<Scalar>::totalChainsMass = 0;

template struct LinkChain<float>;
template struct LinkChain<double>;
