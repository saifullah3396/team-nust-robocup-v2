/**
 * @file PlanningModule/PlanningBehaviors/Attacker.h
 *
 * This file declares the class Attacker.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "LocalizationModule/include/LocalizationRequest.h"
#include "LocalizationModule/include/LandmarkDefinitions.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/Attacker.h"
#include "Utils/include/Behaviors/MBConfigs/MBKickConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBDiveConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBPostureConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBHeadControlConfig.h"
#include "Utils/include/Behaviors/SBConfigs/SBStiffnessConfig.h"
#include "Utils/include/TeamPositions.h"
#include "Utils/include/VisionUtils.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/RobocupRole.h"
#include "Utils/include/DataHolders/TeamRobot.h"
#include "Utils/include/DataHolders/WorldBallInfo.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "VisionModule/include/VisionRequest.h"

#define ATTACKER_PTR static_cast<Attacker*>(bPtr)

boost::shared_ptr<AttackerConfig> Attacker::getBehaviorCast()
{
  return boost::static_pointer_cast<AttackerConfig>(config);
}

void Attacker::initiate()
{
  LOG_INFO("Attacker.initiated() called...")
  BaseModule::publishModuleRequest(boost::make_shared<SwitchBallObstacle>(true));
  Soccer::initiate();
  ROBOCUP_ROLE_OUT(PlanningModule) = (int)RobocupRole::ATTACKER;
  inBehavior = true;
}

void Attacker::ReactAttacker::onRun()
{
  if (ATTACKER_PTR->robotIsFalling()) {
    nextState = ATTACKER_PTR->fallRecovery.get();
  } else if (ATTACKER_PTR->robotIsPenalised()) {
    nextState = ATTACKER_PTR->waitForPenalty.get();
  } else {
    /*
    if (!ATTACKER_PTR->mbInProgress()) {
      auto hsConfig =
        boost::make_shared<HeadScanConfig>();
      *hsConfig =
      *boost::static_pointer_cast<HeadScanConfig>(
        ATTACKER_PTR->getBehaviorCast()->hsConfig);
      ATTACKER_PTR->setupMBRequest(MOTION_1, hsConfig);
    }
    */
    if (!ATTACKER_PTR->mbInProgress()) {
      auto httConfig =
        boost::make_shared<HeadTargetTrackConfig>();
      *httConfig =
      *boost::static_pointer_cast<HeadTargetTrackConfig>(
        ATTACKER_PTR->getBehaviorCast()->httConfig);
      ATTACKER_PTR->setupMBRequest(MOTION_1, httConfig);
    }

    if (!ATTACKER_PTR->isLocalized()) {
      nextState = ATTACKER_PTR->localize.get();
      return;
    }

    if (ATTACKER_PTR->shouldPlayBall()) {
      nextState = ATTACKER_PTR->playBall.get();
    } else {
      nextState = ATTACKER_PTR->goToPosition.get();
    }
  }
}

bool Attacker::ballInRange() {
  static auto ballReactionLimitX = Matrix<float, 2, 1>(fieldMidX - 1.f, fieldMaxX);
  static auto ballReactionLimitY = Matrix<float, 2, 1>(fieldMinY, fieldMaxY);
  auto& ballWorld = WORLD_BALL_INFO_OUT(PlanningModule).posWorld;
  if (ballWorld.x < ballReactionLimitX[0] || ballWorld.x > ballReactionLimitX[1] ||
      ballWorld.y < ballReactionLimitY[0] || ballWorld.y > ballReactionLimitY[1]
     )
  {
    return false;
  }
  return true;
}
