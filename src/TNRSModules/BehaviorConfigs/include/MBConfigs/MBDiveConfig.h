/**
 * @file BehaviorConfigs/include/MBConfigs/MBDiveConfig.h
 *
 * This file declares the structs MBDiveConfig and KFMDiveConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MBConfig.h"
#include "MotionModule/include/DiveModule/KeyFrameDiveTypes.h"
#include "Utils/include/HardwareIds.h"

/**
 * @struct MBDiveConfig
 * @brief Dive behavior configuration
 */
struct MBDiveConfig : MBConfig
{
  /**
   * Constructor
   * 
   * @param type: Type of the dive behavior
   */ 
  MBDiveConfig(const MBDiveTypes& type);
  
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
  static boost::shared_ptr<MBDiveConfig> makeFromJson(const Json::Value& obj);
};
typedef boost::shared_ptr<MBDiveConfig> MBDiveConfigPtr;

/**
 * @struct KFMDiveConfig
 * @brief Key frame motion dive behavior configuration
 */
struct KFMDiveConfig : MBDiveConfig
{
  /**
   * Constructor
   * 
   * @param keyFrameDiveType: Type of the key frame dive
   */ 
  KFMDiveConfig(
    const KeyFrameDiveTypes& keyFrameDiveType = KeyFrameDiveTypes::inPlace);
  
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
  
  //! Type of the key frame dive
  KeyFrameDiveTypes keyFrameDiveType;
};
typedef boost::shared_ptr<KFMDiveConfig> KFMDiveConfigPtr;

/**
 * @struct HandSaveDiveConfig
 * @brief Whole body motion based dive with hand movement for saving
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  HandSaveDiveConfig,
  MBDiveConfig,
  MBDiveTypes::handSaveDive,
  HandSaveDiveConfigPtr,
  (LinkChains, supportLeg, LinkChains::lLeg),
)
