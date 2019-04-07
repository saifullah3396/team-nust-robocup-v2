/**
 * @file Utils/include/PotentialField2D.h
 *
 * This file declares the class PotentialField2D
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <string>
#include <iostream>
#include <boost/shared_array.hpp>
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/Obstacle.h"

/**
 * @struct PotentialField2D
 * @brief This struct defines a potential field for robot pose in two-dimensions
 */
template <typename Scalar>
class PotentialField2D
{
public:
  /**
   * @brief PotentialField2D Constructor
   */
  PotentialField2D(
    const Scalar& kAtt = 1.0,
    const Scalar&kRep = 0.01,
    const Scalar& kRepPerp = 0.01,
    const Scalar& distThresholdAtt = 1.0, // 1m range
    const Scalar& distThresholdRep = 0.5);

  /**
   * @brief ~PotentialField2D Destructor
   */
  virtual ~PotentialField2D();

  VelocityInput<Scalar> update(
    const RobotPose2D<Scalar>& robotPose,
    const RobotPose2D<Scalar>& goalPose,
    const vector<Obstacle<Scalar>>& obstacles);

private:
  vector<Obstacle<Scalar>> obstacles;
  RobotPose2D<Scalar> goalPose;
  RobotPose2D<Scalar> robotPose;

  ///< Attractive forces constant
  Scalar kAtt;
  Scalar kRep;
  Scalar kRepPerp;
  Scalar distThresholdAtt;
  Scalar distThresholdRep;
};
