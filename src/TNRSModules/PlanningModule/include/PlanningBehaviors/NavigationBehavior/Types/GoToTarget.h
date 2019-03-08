/**
 * @file PlanningModule/NavigationBehavior/GoToTarget.h
 *
 * This file declares the class GoToTarget
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/NavigationBehavior.h"

/**
 * @class GoToTarget
 * @brief The behavior for going to a target spot in field
 */
class GoToTarget : public NavigationBehavior
{
public:
  /**
   * Constructor
   * 
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   */
  GoToTarget(
    PlanningModule* planningModule, 
    const BehaviorConfigPtr& config) :
    NavigationBehavior(planningModule, config, "GoToTarget")
  {
  }

  /**
   * Destructor
   */
  ~GoToTarget()
  {
  }
 
protected:
  /**
   * Derived from NavigationBehavior
   */
  void executeMotionAction();

private:
  /**
   * * Returns the config casted as GoToTargetConfigPtr
   */ 
  GoToTargetConfigPtr getBehaviorCast();
};

typedef boost::shared_ptr<GoToTarget> GoToTargetPtr;
