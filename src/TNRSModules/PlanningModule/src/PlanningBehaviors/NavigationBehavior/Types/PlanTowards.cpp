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
  potentialField2D = unique_ptr<PotentialField2D<float>>(new PotentialField2D<float>());
  return true;
}

void PlanTowards::reinitiate(const boost::shared_ptr<BehaviorConfig>& cfg)
{
  LOG_INFO("PlanToward.reinitiate() called...");
  this->config = SPC(PlanTowardsConfig, cfg);
}

void PlanTowards::update()
{
  if (ROBOT_LOCALIZED_IN(PlanningModule)) {
    const auto& robotPose2D = ROBOT_POSE_2D_IN(PlanningModule);
    cout << "OBSTACLES_OBS_IN(PlanningModule).data: " << OBSTACLES_OBS_IN(PlanningModule).data.size() << endl;
    auto goal = getBehaviorCast()->goal;
    if (getBehaviorCast()->reachClosest) {
      goal.x() = fabsf(goal.getX() - robotPose2D.getX()) <= 0.25 ? robotPose2D.getX() : goal.getX();
      goal.y() = fabsf(goal.getY() - robotPose2D.getY()) <= 0.25 ? robotPose2D.getY() : goal.getY();
      goal.theta() = fabsf(goal.getTheta() - robotPose2D.getTheta()) <= Angle::DEG_15 ? robotPose2D.getTheta() : goal.getTheta();
    }
    if (robotPose2D.getX() == robotPose2D.getX()) {
      *velocityInput =
        potentialField2D->update(
          robotPose2D,
          goal,
          OBSTACLES_OBS_IN(PlanningModule).data);
      executeMotionAction();
    }
  } else {
    this->killAllMotionBehaviors();
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
  inBehavior = false;
}
