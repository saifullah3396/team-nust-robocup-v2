/**
 * @file PlanningModule/include/PlanningBehaviors/RobotStartup/RobotStartup.h
 *
 * This file declares the class RobotStartup
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017 
 */

#pragma once

#include "Utils/include/ConfigMacros.h"
#include "PlanningModule/include/PlanningBehavior.h"

struct PBStartupConfig;

/** 
 * @class RobotStartup
 * @brief The class for defining the stratup behavior of the robot when 
 *   the code first initiates on the robot.
 */
class RobotStartup : public PlanningBehavior
{
public:
  /**
   * @brief RobotStartup Constructor
   * @param planningModule Pointer to base planning module
   * @param config Configuration of this behavior
   * @param name Behavior name
   */
  RobotStartup(
    PlanningModule* planningModule, 
    const boost::shared_ptr<PBStartupConfig>& config,
    const string& name = "RobotStartup");

  /**
   * @brief ~RobotStartup Destructor
   */
  virtual ~RobotStartup() {}
  
  /**
   * @brief getType Returns its own child based on the given type
   * 
   * @param planningModule Pointer to base planning module
   * @param cfg Config of the requested behavior
   * 
   * @return boost::shared_ptr<RobotStartup>
   */
  static boost::shared_ptr<RobotStartup> getType(
    PlanningModule* planningModule, const BehaviorConfigPtr& cfg);

private:
  /**
   * @brief getBehaviorCast Returns the cast of config as PBStartupConfigPtr
   */ 
  boost::shared_ptr<PBStartupConfig> getBehaviorCast();
};

typedef boost::shared_ptr<RobotStartup> RobotStartupPtr;
