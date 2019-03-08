/**
 * @file Utils/include/DataHolders/RobocupRole.h
 *
 * This file declares the enumeration RobocupRole
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

/**
 * Enumeration for all the states of the robot stiffnesses.
 *
 * @enum Stiffness
 */
enum class RobocupRole
: unsigned int {
  goalKeeper = 0,
  defender,
  defenseSupport,
  offenseSupport,
  attacker,
  count
};
