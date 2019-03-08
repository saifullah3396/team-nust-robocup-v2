/**
 * @file SBModule/include/StiffnessModule/StiffnessModule.h
 *
 * This file declares the class StiffnessModule
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017 
 */

#pragma once

#include "SBModule/include/StaticBehavior.h"

struct SBStiffnessConfig;
enum class StiffnessState : unsigned;

/**
 * @class StiffnessModule
 * @brief The base class for creating behaviors to control robot 
 *   joint stiffnesses
 */
class StiffnessModule : public StaticBehavior
{
public:
  /**
   * @brief StiffnessModule Constructor
   * @param sbModule Pointer to base sb module
   * @param config Configuration of this behavior
   * @param name Behavior name
   */
  StiffnessModule(
    SBModule* sbModule,
    const boost::shared_ptr<SBStiffnessConfig>& config,
    const string& name = "StiffnessModule");

  /**
   * @brief ~StiffnessModule Destructor
   */
  virtual ~StiffnessModule() {}
  
  /**
   * @brief getType Returns its own child based on the given type
   * 
   * @param sbModule: Pointer to base static behaviors module
   * @param cfg: Config of the requested behavior
   * 
   * @return boost::shared_ptr<StiffnessModule>
   */ 
  static boost::shared_ptr<StiffnessModule> getType(
    SBModule* sbModule, const BehaviorConfigPtr& cfg);

protected:
	/**
   * @brief getBehaviorCast Returns the cast of config to SBStiffnessConfigPtr
	 */ 
  boost::shared_ptr<SBStiffnessConfig> getBehaviorCast();
};

typedef boost::shared_ptr<StiffnessModule> StiffnessModulePtr;
