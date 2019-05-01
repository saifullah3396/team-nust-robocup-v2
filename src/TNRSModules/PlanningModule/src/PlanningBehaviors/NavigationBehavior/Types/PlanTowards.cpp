/**
 * @file PlanningBehaviors/NavigationBehavior/Types/PlanTowards.cpp
 *
 * This file implements the class PlanTowards
 *
 * @author <A href="mailto:">Umaid Muhammad Zaffar</A>
 * @author <A href="mailto:">Marium Aslam</A>
 * @date 7 April 2019
 */

#include "BehaviorConfigs/include/PBConfigs/PBNavigationConfig.h"
#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/Types/PlanTowards.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "TNRSModules/BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"
#include "Utils/include/DataHolders/VelocityInput.h"
#include "Utils/include/PotentialField2D.h"
#include "Utils/include/MathsUtils.h"

PlanTowards::PlanTowards(
  PlanningModule* planningModule,
  const boost::shared_ptr<PlanTowardsConfig>& config) :
  NavigationBehavior(planningModule, config, "PlanTowards")
{
}

PlanTowards::~PlanTowards() {}

boost::shared_ptr<PlanTowardsConfig> PlanTowards::getBehaviorCast()
{
  return SPC(PlanTowardsConfig, config);
}

bool PlanTowards::initiate()
{
  LOG_INFO("PlanToward.initiate() called...");
  velocityInput = unique_ptr<VelocityInput<float>>(new VelocityInput<float>());
  potentialField2D = unique_ptr<PotentialField2D<float>>(new PotentialField2D<float>(10.0));
  return true;
}

void PlanTowards::reinitiate(const boost::shared_ptr<BehaviorConfig>& cfg)
{
  this->config = SPC(PlanTowardsConfig, cfg);
}

void PlanTowards::update()
{
  const auto& robotPose2D = ROBOT_POSE_2D_IN(PlanningModule);
  auto goal = getBehaviorCast()->goal;
  const auto& tolerance = getBehaviorCast()->tolerance;
  if (robotPose2D.getX() == robotPose2D.getX()) {
    *velocityInput =
      potentialField2D->update(
        robotPose2D,
        goal,
        OBSTACLES_OBS_IN(PlanningModule).data,
        tolerance);
    if (velocityInput->norm() <= 1e-3) {
      if (getBehaviorCast()->keepMoving)
        velocityInput->x() = 0.001;
      else
        finish();
    }
    cout << "robotPose2D: " << robotPose2D.get().transpose() << endl;
    cout << "goal: " << goal.get().transpose() << endl;
    cout << "velocityInput: " << velocityInput->get().transpose() << endl;
    velocityInput->theta() = max(min(velocityInput->theta(), 0.25f), -0.25f);
    velocityInput->clip(-1, 1);
    velocityInput->clip(-getBehaviorCast()->maxLimit.getX(), getBehaviorCast()->maxLimit.getX());
    executeMotionAction();
  }
}

void PlanTowards::executeMotionAction()
{
  auto mConfig = boost::make_shared<NaoqiMoveTowardConfig>(*(this->velocityInput));
  mConfig->startPosture = getBehaviorCast()->startPosture;
  mConfig->endPosture = getBehaviorCast()->endPosture;
  setupMBRequest(0, mConfig);
  MOVE_TARGET_OUT(PlanningModule) = goal;
}

void PlanTowards::finish()
{
  LOG_INFO("PlanToward.finish() called...");
  this->killGeneralBehavior();
  this->killAllMotionBehaviors();
  inBehavior = false;
}
