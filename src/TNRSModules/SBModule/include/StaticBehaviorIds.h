/**
 * @file SBModule/include/StaticBehaviorIds.h
 *
 * This file defines the enumerations for all static behavior ids and
 * their child types
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

/**
 * @enum SBIds
 * @brief Enumeration for all the available static behaviors
 */
enum class SBIds
: unsigned int {
  stiffnessModule,
  ledsModule,
  whistleDetector,
};

/**
 * @enum SBStiffnessTypes
 * @brief Enumeration for all possible stiffness behavior types
 */
enum class SBStiffnessTypes
: unsigned int {
  stiffnessInterp
};

/**
 * @enum SBLedsTypes
 * @brief Enumeration for all possible led behavior types
 */
enum class SBLedsTypes
: unsigned int {
  directLeds,
  interpolateLeds
};

/**
 * @enum SBWDTypes
 * @brief Enumeration for all possible whistle detection behavior types
 */
enum class SBWDTypes
: unsigned int {
  akWhistleDetector
};
