/**
 * @file BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h
 *
 * This file declares the structs MBBalanceConfig, MPComControlConfig,
 * PIDComControlConfig, and ZmpControlConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "BehaviorConfigs/include/MBConfigs/MBConfig.h"
#include "Utils/include/HardwareIds.h"

template <typename Scalar>
class ZmpRefGenerator;

/**
 * @struct MBBalanceConfig
 * @brief Balance behavior configuration
 */
struct MBBalanceConfig : MBConfig
{
  /**
   * Constructor
   * 
   * @param type: Type of the balance behavior
   * @param supportLeg: The support leg to shift the balance to
   * @param timeToReachB: Time to shift balance
   */ 
  MBBalanceConfig(
    const MBBalanceTypes& type,
    const LinkChains& supportLeg,
    const float& timeToReachB = 2.0);
  
  /**
   * Makes an object of type this and returns it if valid
   */ 
  static boost::shared_ptr<MBBalanceConfig> 
    makeFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */
  virtual Json::Value getJson();
  
  //! The support leg to shift the balance to
  LinkChains supportLeg;

  //! Time to shift the balance to support leg
  float timeToReachB;
};
typedef boost::shared_ptr<MBBalanceConfig> MBBalanceConfigPtr;

/**
 * @struct MPComControlConfig
 * @brief Configuration for motion primitive based center of mass 
 *  control behavior
 */
struct MPComControlConfig : MBBalanceConfig
{
  /**
   * Constructor
   * 
   * @param supportLeg: The support leg to shift the balance to
   * @param timeToReachB: Time to shift the balance to support leg
   */ 
  MPComControlConfig(
    const LinkChains& supportLeg = LinkChains::lLeg,
    const float& timeToReachB = 1.f);
  
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
};
typedef boost::shared_ptr<MPComControlConfig> MPComControlConfigPtr;

/**
 * @struct PIDComControlConfig
 * @brief Configuration for pid based center of mass 
 *  control behavior
 */
struct PIDComControlConfig : MBBalanceConfig
{
  /**
   * Constructor
   * 
   * @param supportLeg: The support leg to shift the balance to
   */ 
  PIDComControlConfig(const LinkChains& supportLeg = LinkChains::lLeg);
  
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
};
typedef boost::shared_ptr<PIDComControlConfig> PIDComControlConfigPtr;

/**
 * @struct ZmpControlConfig
 * @brief Configuration for zmp controller based balance
 */
struct ZmpControlConfig : MBBalanceConfig
{
  /**
   * Constructor
   * 
   * @param supportLeg: The support leg to shift the balance to
   * @param refGenerator: The zmp reference values generator
   */ 
  ZmpControlConfig(
    const LinkChains& supportLeg = LinkChains::lLeg,
    const bool& keepOtherLegContact = true,
    const bool& regularizePosture = true,
    const bool& keepTorsoUpright = false);
  
  /**
   * Constructor
   * 
   * @param supportLeg: The support leg to shift the balance to
   * @param refGenerator: The zmp reference values generator
   */ 
  ZmpControlConfig(
    const LinkChains& supportLeg,
    const boost::shared_ptr<ZmpRefGenerator<float> >& refGenerator,
    const bool& keepOtherLegContact = true,
    const bool& regularizePosture = true,
    const bool& keepTorsoUpright = false);

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

  //! Whether to keep torso upright while balancing
  bool keepTorsoUpright;

  //! Whether to track a target posture while balancing
  bool regularizePosture;

  //! Whether to keep the other leg fixed on ground while balancing
  bool keepOtherLegContact;

  //! Joints used for balancing
  vector<bool> activeJoints;

  //! Target for ZMP in x-direction
  float targetX;

  //! Target for ZMP in y-direction
  float targetY;

  //! Pointer to the zmp references generator
  boost::shared_ptr<ZmpRefGenerator<float> > refGenerator;
};
typedef boost::shared_ptr<ZmpControlConfig> ZmpControlConfigPtr;
