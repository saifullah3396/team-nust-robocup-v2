/**
 * @file Utils/include/Behaviors/SBConfigs/SBWDConfig.h
 *
 * This file defines the class SBWDConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "Utils/include/Behaviors/SBConfigs/SBConfig.h"

/**
 * @struct SBWDConfig
 * @brief The whistle detector behavior configuration
 */
struct SBWDConfig : SBConfig
{
	/**
	 * Constructor
	 * 
	 * @param type: Type of the led behavior
	 * @param timeToDetect: Max time given for whistle detection
	 */
  SBWDConfig(
    const SBWDTypes& type = SBWDTypes::akWhistleDetector,
    const float& timeToDetect = 10.f) :
    SBConfig(SBIds::whistleDetector, 10.f, (int)type),
    timeToDetect(timeToDetect)
  {
  }

  /**
	 * Validates the given configuration parameters
	 */
  void validate() {
    if (timeToDetect <= 0.f) // Undefined time given
    {
      throw
        BConfigException(
          this,
          "Invalid behavior configuration parameters passed.",
          false,
          EXC_INVALID_BCONFIG_PARAMETERS
        );
    }
  }
  
  //! Max time for whistle detection
  float timeToDetect;
};

typedef boost::shared_ptr<SBWDConfig> SBWDConfigPtr;
