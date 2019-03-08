/**
 * @file Utils/include/Behaviors/SBConfigs.h
 *
 * This file declares the class SBConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "Utils/include/Behaviors/BehaviorConfig.h"
#include "SBModule/include/StaticBehaviorIds.h"
#include "Utils/include/EnumUtils.h"

/**
 * @struct SBConfig
 * @brief Base static behavior configuration
 */
struct SBConfig : BehaviorConfig
{
	/**
	 * Constructor
	 * 
	 * @param id: Id of the behavior
	 * @param maxRunTime: Max running time for the behavior
	 * @param type: Type of the behavior
	 */
  SBConfig(
    const SBIds& id,
    const float& maxRunTime,
    const int& type) :
  BehaviorConfig(toUType(id), BaseBehaviorType::sb, maxRunTime, type)
  {
  }

  /**
   * @brief makeFromJson Returns a child config of given type
   * @param obj Json object of configuration
   * @return SBConfigPtr
   */
  static boost::shared_ptr<SBConfig>
    makeFromJson(const Json::Value& obj);

  /**
   * @brief assignFromJson Assigns configuration parameters from json
   * @param obj Json configuration
   * @return true if successful
   */
  virtual bool assignFromJson(const Json::Value& obj)
  {
    if (!BehaviorConfig::assignFromJson(obj))
      return false;
    return true;
  }

  /**
   * @brief getJson Makes a json object from config paramters
   * @return Json object
   */
  virtual Json::Value getJson() {
    return BehaviorConfig::getJson();
  }
};

typedef boost::shared_ptr<SBConfig> SBConfigPtr;
