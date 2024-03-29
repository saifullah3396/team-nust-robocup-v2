/**
 * @file PlanningModule/include/PlanningBehaviorIds.h
 *
 * This file declares the enumeration for all the behavior planning 
 * behaviors and their childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#pragma once

/**
 * Enumeration for all the planning behaviors
 *
 * @enum PBIds
 */
enum class PBIds
: unsigned int {
  startup = 0,
  navigation,
  robocup,
  kickSequence,
  externalInterface,
  testSuite
};

/**
 * Enumeration for all possible types of startup behavior
 *
 * @enum PBStartupTypes
 */
enum class PBStartupTypes
: unsigned int
{
  requestBehavior = 0
};

/**
 * Enumeration for all possible types of navigation behaviors
 *
 * @enum PBNavigationTypes
 */
enum class PBNavigationTypes
: unsigned int
{
  goToTarget = 0
};

/**
 * Enumeration for all possible types of robocup behavior
 *
 * @enum PBRobocupTypes
 */
enum class PBRobocupTypes
: unsigned int
{
  robocupSetup = 0,
  goalKeeper,
  defender,
  attacker,
  penalties
};

/**
 * Enumeration for all possible types of kick sequence behavior
 *
 * @enum PBKickSequenceTypes
 */
enum class PBKickSequenceTypes
: unsigned int
{
  ballIntercept = 0,
  findAndKick
};

/**
 * Enumeration for all possible types of external interface behaviors
 *
 * @enum PBExternalInterfaceTypes
 */
enum class PBExternalInterfaceTypes
: unsigned int
{
  nihaCognition = 0,
  userReqHandler
};

/**
 * Enumeration for all possible types of TestSuite behaviors
 *
 * @enum TestSuiteTypes
 */
enum class TestSuiteTypes
: unsigned int
{
  vision = 0,
  localization,
  motion,
  navigation
};

