/**
 * @file MotionModule/include/MotionConfigs/MBConfig.h
 *
 * This file declares the struct MBConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <boost/make_shared.hpp>
#include "MotionModule/include/MotionBehaviorIds.h"
#include "BehaviorConfigs/include/BehaviorConfig.h"

/**
 * @struct MBConfig
 * @brief Base class for all motion behavior configurations
 */
struct MBConfig : BehaviorConfig
{
	/**
	 * Constructor
	 * 
	 * @param id: Id of the behavior
	 * @param maxRunTime: Max running time for the behavior
	 * @param type: Type of the behavior
	 */
  MBConfig(
    const MBIds& id, 
    const float& maxRunTime,
    const int& childType = -1);

  /**
   * @brief makeFromJson Returns a child config of given type
   * @param obj Json object of configuration
   * @return MBConfigPtr
   */
  static boost::shared_ptr<MBConfig>
    makeFromJson(const Json::Value& obj);

  /**
   * @brief assignFromJson Assigns configuration parameters from json
   * @param obj Json configuration
   * @return true if successful
   */
  virtual bool assignFromJson(const Json::Value& obj);

  /**
   * @brief getJson Makes a json object from config paramters
   * @return Json object
   */
  virtual Json::Value getJson();
};
typedef boost::shared_ptr<MBConfig> MBConfigPtr;
