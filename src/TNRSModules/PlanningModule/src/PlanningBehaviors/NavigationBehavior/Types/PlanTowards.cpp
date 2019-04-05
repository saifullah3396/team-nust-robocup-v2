#include "TNRSBase/include/MemoryIOMacros.h"
#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/Types/PlanTowards.h"
#include "BehaviorConfigs/include/PBConfigs/PBNavigationConfig.h"
#include "Utils/include/PotentialField2D.h"
#include "TNRSModules/BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"

PlanTowards::PlanTowards(PlanningModule* planningModule,
                         const boost::shared_ptr<PlanTowardsConfig>& config) :
                         NavigationBehavior(planningModule, config, "PlanTowards") {}

boost::shared_ptr<PlanTowardsConfig> PlanTowards::getBehaviorCast()
{
    return SPC(PlanTowardsConfig, config);
}


bool PlanTowards::initiate()
{
    LOG_INFO("PlanToward.initiate() called...");
    goal = getBehaviorCast()->goal;
    if (!NavigationBehavior.setGoal(goal)) {
      if (getBehaviorCast()->reachClosest) {
        if (findPossibleGoal(goal)) {
        } else {
          LOG_ERROR("NavigationBehavior.setGoal() failed. Exiting...")
          return false;
        }
      } else {
        return false;
        LOG_ERROR("NavigationBehavior.setGoal() failed. Exiting...")
      }
    }
}

bool PlanTowards::update()
{
    auto obj = OBSTACLES_OBS_IN(PlanningModule);
    auto currentPose = ROBOT_POSE_2D_IN(PlanningModule);
    velocity = PotentialField2D.update(currentPose, goal, obj->data);
    executeMotionAction();
}

void PlanTowards::executeMotionAction()
{
    auto mConfig = boost::make_shared<NaoqiMoveTowardConfig>(this->velocityInput);
    mConfig->startPosture = getBehaviorCast()->startPosture;
    mConfig->endPosture = getBehaviorCast()->endPosture;
    setupMBRequest(0, mConfig);
    MOVE_TARGET_OUT(PlanningModule) = goal;
}
