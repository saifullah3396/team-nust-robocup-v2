/**
 * @file Utils/include/Behaviors/MBConfigs/MBBallThrowConfig.h
 *
 * This file declares the struct MBBallThrowConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MBConfig.h"

/**
 * @struct MBBallThrowConfig
 * @brief Ball throw behavior configuration
 */
struct MBBallThrowConfig : MBConfig
{
  /**
   * @param type: Type of the ball throw behavior
   * @param timeToThrow: Total ball throw execution time
   * @param headTapToStart: Whether to execute behavior on head tap
   * @param 
   */ 
  MBBallThrowConfig(
    const MBBallThrowTypes& type = MBBallThrowTypes::wbBallThrow,
    const float& timeToThrow = 2.f,
    const bool& headTapToStart = false);

  /**
   * @derived
   */ 
  void validate();

  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */
  virtual Json::Value getJson();
  
  /**
   * Makes an object of type this and returns it if valid
   */ 
  static boost::shared_ptr<MBBallThrowConfig> 
    makeFromJson(const Json::Value& obj);

  //! Total ball throw execution time
  float timeToThrow;
  
  //! Whether to execute behavior on head tap
  bool headTapToStart;
};
typedef boost::shared_ptr<MBBallThrowConfig> MBBallThrowConfigPtr;

/**
 * @struct WBBallThrowConfig
 * @brief Ball throwing behavior using whole body motion
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  WBBallThrowConfig,
  MBBallThrowConfig,
  MBBallThrowTypes::wbBallThrow,
  WBBallThrowConfigPtr,
  (float, throwVelocity, 0.5),
  (float, throwAngle, 0.0),
)
