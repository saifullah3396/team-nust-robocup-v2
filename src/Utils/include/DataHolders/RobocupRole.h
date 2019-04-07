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
 * @brief Enumeration for all the robocup roles
 *
 * @enum RobocupRole
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
