/**
 * @file GBModule/include/LedsModule/LedsModule.h
 *
 * This file declares the class LedsModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include "GBModule/include/GeneralBehavior.h"
#include "GBModule/include/LedRequest.h"

struct GBLedsConfig;

/**
 * @class LedsModule
 * @brief A class for generating led requests to control robot leds
 */ 
class LedsModule : public GeneralBehavior
{
public:
  /**
   * @brief LedsModule Constructor
   * 
   * @param gbModule Pointer to base static behaviors module
   * @param config Configuration of the behavior
   * @param name Name of the behavior
   */
  LedsModule(
    GBModule* gbModule,
    const boost::shared_ptr<GBLedsConfig>& config,
    const string& name = "LedsModule");
  
  /**
   * @brief ~LedsModule Destructor
   */
  virtual ~LedsModule() {}

  /**
   * @brief getType Returns its own child based on the given type
   * 
   * @param gbModule: Pointer to base static behaviors module
   * @param cfg: Config of the requested behavior
   * 
   * @return boost::shared_ptr<LedsModule>
   */
  static boost::shared_ptr<LedsModule> getType(
    GBModule* gbModule, const BehaviorConfigPtr& cfg);

protected:
 /**
  * @brief getBehaviorCast Returns the casts of config to GBLedsConfigPtr
  */ 
  boost::shared_ptr<GBLedsConfig> getBehaviorCast();
  
  //! Led request
  LedRequestPtr ledRequest;
};

typedef boost::shared_ptr<LedsModule> LedsModulePtr;
