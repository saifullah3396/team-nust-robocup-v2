/**
 * @file MotionModule/include/BalanceModule/BalanceZmpRefGen.h
 *
 * This file declares the class BalanceZmpRefGen
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/BalanceModule/ZmpRefGenerator.h"

/**
 * @class BalanceZmpRefGen
 * @brief The class for generating simple zmp reference for shifting
 *   robot balance to the required support leg
 */
template <typename Scalar>
class BalanceZmpRefGen : public ZmpRefGenerator<Scalar>
{
public:
  /**
   * @brief BalanceZmpRefGen Constructor
   * @param motionModule Pointer to base motion module
   * @param refFrame Base reference foot
   * @param nReferences Number of references to be generated
   * @param totalTime Total time for shifting balance
   * @param targetZmp Target zmp positions
   */
  BalanceZmpRefGen(
    MotionModule* motionModule,
    const RobotFeet& refFrame,
    const unsigned& nReferences,
    const Scalar& totalTime,
    const Matrix<Scalar, 2, 1>& targetZmp);

  /**
   * @brief ~BalanceZmpRefGen Destructor
   */
  ~BalanceZmpRefGen() final {}

  /**
   * @brief update See ZmpRefGenerator::initiate()
   */
  bool initiate() final;

  /**
   * @brief update See ZmpRefGenerator::update()
   */
  void update(const Scalar& timeStep) final;

private:
  Scalar totalTime = {0.0}; ///< Total time to shift balance
  Matrix<Scalar, 2, 1> targetZmp = Matrix<Scalar, 2, 1>::Zero(); ///< Target zmp position
  Matrix<Scalar, 2, 1> initZmpPosition = Matrix<Scalar, 2, 1>::Zero(); ///< Initial zmp position

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<BalanceZmpRefGen<MType> > BalanceZmpRefGenPtr;
