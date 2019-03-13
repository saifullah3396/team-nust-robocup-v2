/**
 * @file PlanningModule/include/PBConfigs/PBConfig.h
 *
 * This file declares the struct PBConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "BehaviorConfigs/include/BehaviorConfig.h"
#include "PlanningModule/include/PlanningBehaviorIds.h"
#include "Utils/include/EnumUtils.h"

/**
 * @struct PBConfig
 * @brief Base planning behavior configuration
 */
struct PBConfig : BehaviorConfig
{
  /**
   * Constructor
   * 
   * @param id: Id of the behavior
   * @param maxRunTime: Maximum allowed time for the behavior
   * @param type: Type of the behavior
   */ 
  PBConfig(
    const PBIds& id,
    const float& maxRunTime,
    const int& type) : 
  BehaviorConfig(toUType(id), BaseBehaviorType::planning, maxRunTime, type)
  {}

  /**
   * @brief makeFromJson Returns a child config of given type
   * @param obj Json object of configuration
   * @return MBConfigPtr
   */
  static boost::shared_ptr<PBConfig>
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
typedef boost::shared_ptr<PBConfig> PBConfigPtr;
