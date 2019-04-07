/**
 * @file TNRSModules/BehaviorConfigs/include/GBConfigs/GBConfig.h
 *
 * This file declares the class GBConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "BehaviorConfigs/include/BehaviorConfig.h"
#include "GBModule/include/GeneralBehaviorIds.h"
#include "Utils/include/EnumUtils.h"

/**
 * @struct GBConfig
 * @brief Base static behavior configuration
 */
struct GBConfig : BehaviorConfig
{
  /**
   * Constructor
   *
   * @param id: Id of the behavior
   * @param maxRunTime: Max running time for the behavior
   * @param type: Type of the behavior
   */
  GBConfig(
    const GBIds& id,
    const float& maxRunTime,
    const int& type) :
  BehaviorConfig(toUType(id), BaseBehaviorType::general, maxRunTime, type)
  {
  }

  /**
   * @brief makeFromJson Returns a child config of given type
   * @param obj Json object of configuration
   * @return GBConfigPtr
   */
  static boost::shared_ptr<GBConfig>
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

typedef boost::shared_ptr<GBConfig> GBConfigPtr;
