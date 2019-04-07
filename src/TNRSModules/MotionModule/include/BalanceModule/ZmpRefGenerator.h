/**
 * @file MotionModule/include/BalanceModule/ZmpRefGenerator.h
 *
 * This file declares the class ZmpRefGenerator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include "MotionModule/include/BalanceModule/ZmpRef.h"

class MotionModule;
template <typename Scalar>
class KinematicsModule;
enum class RobotFeet : unsigned int;

/**
 * @class ZmpRefGenerator
 * @brief The base class for Zero moment point reference generators
 */
template <typename Scalar>
class ZmpRefGenerator
{
public:
  /**
   * @brief ZmpRefGenerator Constructor
   * @param motionModule Pointer to base motion module class
   * @param refFrame Global reference frame
   * @param nReferences Total number of previewable references
   */
  ZmpRefGenerator(
    MotionModule* motionModule,
    const RobotFeet& refFrame,
    const unsigned& nReferences);

  /**
   * @brief ~ZmpRefGenerator Destructor
   */
  virtual ~ZmpRefGenerator() {}

  /**
   * @brief initiate Initiates the zmp reference generator
   */
  virtual bool initiate() = 0;

  /**
   * @brief update Finds the next zmp references and updates the output
   * @param timeStep Update cycle time
   */
  virtual void update(const Scalar& timeStep) = 0;

  ///< Getters
  const boost::shared_ptr<ZmpRef<Scalar>>& getCurrentRef() const { return zmpRef; }
  const RobotFeet& getRefFrame() const { return refFrame; }

protected:
  boost::shared_ptr<KinematicsModule<Scalar> > kM; ///< Kinematics module
  boost::shared_ptr<ZmpRef<Scalar>> zmpRef; ///< References container
  Scalar cycleTime; ///< The time step between each reference generated
  unsigned nReferences; ///< Number of previewable references
  RobotFeet refFrame; ///< Base frame of reference

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
