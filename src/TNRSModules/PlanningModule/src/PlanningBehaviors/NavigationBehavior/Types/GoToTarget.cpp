/**
 * @file PlanningBehaviors/NavigationBehavior/Types/GoToTarget.cpp
 *
 * This file implements the class GoToTarget
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "TNRSBase/include/MemoryIOMacros.h"
#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/Types/GoToTarget.h"
#include "BehaviorConfigs/include/PBConfigs/PBNavigationConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"

GoToTarget::GoToTarget(
  PlanningModule* planningModule,
  const boost::shared_ptr<GoToTargetConfig>& config) :
  NavigationBehavior(planningModule, config, "GoToTarget")
{
}

boost::shared_ptr<GoToTargetConfig> GoToTarget::getBehaviorCast()
{
  return SPC(GoToTargetConfig, config);
}

void GoToTarget::executeMotionAction()
{
  cout << "GoToTarget::executeMotionAction()\n";
  auto mConfig =
    boost::make_shared<NaoqiFootstepsConfig>(
      TNRSFootstep<float>::fromPathPlannerStates(this->plannedPath)
    );
  mConfig->startPosture = getBehaviorCast()->startPosture;
  mConfig->endPosture = getBehaviorCast()->endPosture;
  setupMBRequest(MOTION_1, mConfig);
  MOVE_TARGET_OUT(PlanningModule) = goal;
  behaviorState = validatePath;
}
