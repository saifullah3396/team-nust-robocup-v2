/**
 * @file GBModule/include/StiffnessModule/StiffnessModule.h
 *
 * This file declares the class StiffnessModule
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017 
 */

#pragma once

#include "GBModule/include/GeneralBehavior.h"

struct GBStiffnessConfig;
enum class StiffnessState : unsigned;

/**
 * @class StiffnessModule
 * @brief The base class for creating behaviors to control robot 
 *   joint stiffnesses
 */
class StiffnessModule : public GeneralBehavior
{
public:
  /**
   * @brief StiffnessModule Constructor
   * @param gbModule Pointer to base sb module
   * @param config Configuration of this behavior
   * @param name Behavior name
   */
  StiffnessModule(
    GBModule* gbModule,
    const boost::shared_ptr<GBStiffnessConfig>& config,
    const string& name = "StiffnessModule");

  /**
   * @brief ~StiffnessModule Destructor
   */
  virtual ~StiffnessModule() {}
  
  /**
   * @brief getType Returns its own child based on the given type
   * 
   * @param gbModule: Pointer to base static behaviors module
   * @param cfg: Config of the requested behavior
   * 
   * @return boost::shared_ptr<StiffnessModule>
   */ 
  static boost::shared_ptr<StiffnessModule> getType(
    GBModule* gbModule, const BehaviorConfigPtr& cfg);

protected:
	/**
   * @brief getBehaviorCast Returns the cast of config to GBStiffnessConfigPtr
	 */ 
  boost::shared_ptr<GBStiffnessConfig> getBehaviorCast();
};

typedef boost::shared_ptr<StiffnessModule> StiffnessModulePtr;
