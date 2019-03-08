/**
 * @file Utils/include/DataHolders/PositionInput.h
 *
 * This file defines the struct PositionInput
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include "Utils/include/DataHolders/RobotPose2D.h"

/**
 * @struct PositionInput
 * @brief The struct for defining a position update of the robot
 *   in 2D environment.
 */
template<typename Scalar = float>
struct PositionInput : public RobotPose2D<Scalar>
{
  /**
   * @brief Constructor.
   * @param dX X-coordinate of the input velocity.
   * @param dY Y-coordinate of the input velocity.
   * @param dTheta Theta-coordinate of the input velocity.
   */
  PositionInput(const Scalar& x, const Scalar& y, const Scalar& theta) :
    RobotPose2D<Scalar>(x, y, theta)
  {
  }

  /**
   * @brief print Self-explanatory
   */
  void print() const  final
  {
    PRINT_DATA(
      PositionInput,
      (input, this->get().transpose()),
      ("", "")
    );
  }

  Json::Value getJson() const final
  {
    Json::Value val;
    JSON_ASSIGN_(val, input, JsonUtils::MatrixToJson(this->get().transpose()));
    return val;
  }
};
