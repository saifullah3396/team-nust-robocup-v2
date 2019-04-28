/**
 * @file PlanningBehaviors/NavigationBehavior/Types/GoToTarget.cpp
 *
 * This file implements the class GoToTarget
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/PBConfigs/PBNavigationConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/NavigationBehavior.h"
#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/Types/GoToTarget.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "UserCommModule/include/UserCommRequest.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "Utils/include/DataHolders/PositionInput.h"
#include "Utils/include/PathPlanner/PathPlanner.h"
#include "Utils/include/VisionUtils.h"

using namespace PathPlannerSpace;

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

bool GoToTarget::initiate()
{
  LOG_INFO("NavigationBehavior.initiate() called...")
  planFailCount = 0;
  replanFailCount = 0;
  pathPlanner = this->getPathPlanner();
  pathPlanner->updateMap();
  goal = getBehaviorCast()->goal;
  if (!setGoal(goal)) {
    if (getBehaviorCast()->reachClosest) {
      if (findPossibleGoal(goal)) {
        //LOG_INFO("NavigationBehavior.setGoal() successful. Continuing...")
      } else {
        LOG_ERROR("NavigationBehavior.setGoal() failed. Exiting...")
        return false;
      }
    } else {
      return false;
      LOG_ERROR("NavigationBehavior.setGoal() failed. Exiting...")
    }
  }
  if (!setStart()) {
    LOG_ERROR("NavigationBehavior.setStart() failed. Exiting...")
    return false;
  }
  pathPlanned = false;
  behaviorState = planPath;
  return true;
}

void GoToTarget::reinitiate(const BehaviorConfigPtr& cfg)
{
  LOG_INFO("NavigationBehavior reinitiation...")
  int currentStep = N_FOOTSTEPS_IN(PlanningModule) - startStep;
  if (currentStep < 6 || plannedPath[currentStep].getLeg() != RIGHT) {
    ///< Don't reinitiate unless the current step is right foot since the planner
    ///< always returns the solution with first step as right foot
    ///< Also gives reasonable time between reinitiation calls
    return;
  }

  this->config = cfg;
  if (!pathPlanned || currentStep >= plannedPath.size() - 1) {
    ///< If the path is not already planned, we just call initiate
    initiate();
  } else {
    ///< Update the map
    pathPlanner->updateMap();

    ///< Set new goal position
    bool goalSet = false;
    goal = getBehaviorCast()->goal;
    if (setGoal(goal)) {
      //LOG_INFO("NavigationBehavior.setGoal() successful. Continuing...")
      goalSet = true;
    } else {
      if (getBehaviorCast()->reachClosest) {
        pathPlanned = false;
        goalSet = findPossibleGoal(goal);
      }
    }

    ///< If a goal position is set successfully
    if (goalSet) {
      auto right = plannedPath[currentStep];
      auto left = plannedPath[currentStep - 1];
      auto step =
        State(
          right.getX() - left.getX(),
          right.getY() - left.getY(),
          right.getTheta() - left.getTheta(),
          right.getLeg()
        );
      Matrix<float, 4, 4> lFootT;
      if (getFootTransform(lFootT, LEFT)) {
        auto lTheta =
          MathsUtils::matToEuler(
            (Matrix<float, 3, 3>) lFootT.block(0, 0, 3, 3)
          )[2];
        State leftActual(lFootT(0, 3), lFootT(1, 3), lTheta, LEFT);
        State rightActual =
          State(
            leftActual.getX() + step.getX(),
            leftActual.getY() + step.getY(),
            leftActual.getTheta() + step.getTheta(),
            step.getLeg()
          );
        if (setStart(leftActual, rightActual) && replan()) {
          if (GET_DVAR(int, drawFootsteps))
            drawFootstepsData();
          if (GET_DVAR(int, sendFootsteps))
            sendFootstepsData();
          pathPlanned = true;
          planFailCount = 0;
          behaviorState = executeMotion;
        } else {
          pathPlanned = false;
          ++planFailCount;
        }
      }
    }
  }
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
