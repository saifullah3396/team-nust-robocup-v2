/**
 * @file PlanningBehaviors/NavigationBehavior/Types/GoToTarget.h
 *
 * This file declares the class GoToTarget
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/NavigationBehavior.h"

struct GoToTargetConfig;

/**
 * @class GoToTarget
 * @brief The behavior for going to a target spot in field
 */
class GoToTarget : public NavigationBehavior
{
public:
  /**
   * @brief GoToTarget Constructor
   * @param planningModule Pointer to base planning module
   * @param config Configuration of this behavior
   */
  GoToTarget(
    PlanningModule* planningModule,
    const boost::shared_ptr<GoToTargetConfig>& config);

  /**
   * @brief ~GoToTaget Destructor
   */
  ~GoToTarget() final {}

protected:
  /**
   * @brief executeMotionAction See NavigationBehavior::executeMotionAction()
   */
  void executeMotionAction() final;

private:
  /**
   * * Returns the config casted as GoToTargetConfigPtr
   */
  boost::shared_ptr<GoToTargetConfig> getBehaviorCast();
};

typedef boost::shared_ptr<GoToTarget> GoToTargetPtr;
