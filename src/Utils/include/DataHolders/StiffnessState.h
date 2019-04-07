/**
 * @file Utils/include/DataHolders/StiffnessState.h
 *
 * This file declares the enumeration StiffnessState
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

/**
 * @brief Enumeration for all the states of the robot
 *   stiffness configurations
 *
 * @enum StiffnessState
 */
enum class StiffnessState
: unsigned int {
  min,
  max,
  robocup,
  getup,
  unknown,
  count
};
