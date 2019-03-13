/**
 * @file GBModule/include/GeneralBehaviorIds.h
 *
 * This file defines the enumerations for all static behavior ids and
 * their child types
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

/**
 * @enum GBIds
 * @brief Enumeration for all the available static behaviors
 */
enum class GBIds
: unsigned int {
  stiffnessModule,
  ledsModule,
  whistleDetector,
};

/**
 * @enum GBStiffnessTypes
 * @brief Enumeration for all possible stiffness behavior types
 */
enum class GBStiffnessTypes
: unsigned int {
  stiffnessInterp
};

/**
 * @enum GBLedsTypes
 * @brief Enumeration for all possible led behavior types
 */
enum class GBLedsTypes
: unsigned int {
  directLeds,
  interpolateLeds
};

/**
 * @enum GBWDTypes
 * @brief Enumeration for all possible whistle detection behavior types
 */
enum class GBWDTypes
: unsigned int {
  akWhistleDetector
};
