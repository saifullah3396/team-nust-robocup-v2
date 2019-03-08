/**
 * @file SBModule/include/LedsModule/LedsModule.h
 *
 * This file declares the class LedsModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include "SBModule/include/StaticBehavior.h"
#include "SBModule/include/LedRequest.h"

struct SBLedsConfig;

/**
 * @class LedsModule
 * @brief A class for generating led requests to control robot leds
 */ 
class LedsModule : public StaticBehavior
{
public:
  /**
   * @brief LedsModule Constructor
   * 
   * @param sbModule Pointer to base static behaviors module
   * @param config Configuration of the behavior
   * @param name Name of the behavior
   */
  LedsModule(
    SBModule* sbModule,
    const boost::shared_ptr<SBLedsConfig>& config,
    const string& name = "LedsModule");
  
  /**
   * @brief ~LedsModule Destructor
   */
  virtual ~LedsModule() {}

  /**
   * @brief getType Returns its own child based on the given type
   * 
   * @param sbModule: Pointer to base static behaviors module
   * @param cfg: Config of the requested behavior
   * 
   * @return boost::shared_ptr<LedsModule>
   */
  static boost::shared_ptr<LedsModule> getType(
    SBModule* sbModule, const BehaviorConfigPtr& cfg);

protected:
 /**
  * @brief getBehaviorCast Returns the casts of config to SBLedsConfigPtr
  */ 
  boost::shared_ptr<SBLedsConfig> getBehaviorCast();
  
  //! Led request
  LedRequestPtr ledRequest;
};

typedef boost::shared_ptr<LedsModule> LedsModulePtr;
