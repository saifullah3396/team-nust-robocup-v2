/**
 * @file Utils/include/DataHolders/VelocityInput.h
 *
 * This file defines the struct VelocityInput
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/JsonUtils.h"

/**
 * @struct VelocityInput
 * @brief The struct for defintion of the velocity control input to the
 *   humanoid in 2D.
 */
template<typename Scalar = float>
struct VelocityInput : RobotPose2D<Scalar>
{
  /**
   * @brief VelocityInput Constructor.
   */
  VelocityInput() = default;

  /**
   * @brief VelocityInput Constructor.
   * @param dX X-coordinate of the input velocity.
   * @param dY Y-coordinate of the input velocity.
   * @param dTheta Theta-coordinate of the input velocity.
   */
  VelocityInput(const Scalar& dX, const Scalar& dY, const Scalar& dTheta) :
    RobotPose2D<Scalar>(dX, dY, dTheta)
  {
  }

  /**
   * @brief print Self-explanatory
   */
  void print() const final
  {
    PRINT_DATA(
      VelocityInput,
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
