/**
 * @file BehaviorConfigs/include/GBConfigs/GBWDConfig.h
 *
 * This file defines the class GBWDConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "BehaviorConfigs/include/GBConfigs/GBConfig.h"

/**
 * @struct GBWDConfig
 * @brief The whistle detector behavior configuration
 */
struct GBWDConfig : GBConfig
{
	/**
	 * Constructor
	 * 
	 * @param type: Type of the led behavior
	 * @param timeToDetect: Max time given for whistle detection
	 */
  GBWDConfig(
    const GBWDTypes& type = GBWDTypes::akWhistleDetector,
    const float& timeToDetect = 10.f) :
    GBConfig(GBIds::whistleDetector, 10.f, (int)type),
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

typedef boost::shared_ptr<GBWDConfig> GBWDConfigPtr;
