/**
 * @file Utils/include/DataHolders/PlanningState.h
 *
 * This file declares the enumeration PlanningState
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

/**
 * Enumeration for all the major states of the robot planning.
 *
 * @enum PlanningState
 */
enum class PlanningState
: unsigned int { // Update this cuz this has no purpose for now
  startup = 0,
  robocup,
  robocupPenalties,
  unknown,
  count
};
