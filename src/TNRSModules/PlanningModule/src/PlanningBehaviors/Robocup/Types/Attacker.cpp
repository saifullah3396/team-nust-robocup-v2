/**
 * @file PlanningBehaviors/Robocup/Types/Attacker.cpp
 *
 * This file declares the class Attacker.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "LocalizationModule/include/LocalizationRequest.h"
#include "LocalizationModule/include/LandmarkDefinitions.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/Attacker.h"
#include "BehaviorConfigs/include/PBConfigs/PBRobocupConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBKickConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBDiveConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "BehaviorConfigs/include/GBConfigs/GBStiffnessConfig.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/TeamPositions.h"
#include "Utils/include/VisionUtils.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/RobocupRole.h"
#include "Utils/include/DataHolders/TeamRobot.h"
#include "Utils/include/DataHolders/WorldBallInfo.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "VisionModule/include/VisionRequest.h"

#define ATTACKER_PTR static_cast<Attacker*>(bPtr)

Attacker::Attacker(
  PlanningModule* planningModule,
  const boost::shared_ptr<AttackerConfig>& config) :
  Soccer(planningModule, config, "Attacker")
{
  DEFINE_FSM_STATE(Soccer, ReactAttacker, react)
}

boost::shared_ptr<AttackerConfig> Attacker::getBehaviorCast()
{
  return boost::static_pointer_cast<AttackerConfig>(config);
}

bool Attacker::initiate()
{
  LOG_INFO("Attacker.initiated() called...")
  BaseModule::publishModuleRequest(boost::make_shared<SwitchBallObstacle>(false));
  ROBOCUP_ROLE_OUT(PlanningModule) = (int)RobocupRole::attacker;
  return Soccer::initiate();
}

void Attacker::ReactAttacker::onRun()
{
  //cout << "In react..." << endl;
  if (ATTACKER_PTR->robotIsFalling()) {
    nextState = ATTACKER_PTR->fallRecovery.get();
    //cout << "robot is falling..." << endl;
  } else if (ATTACKER_PTR->robotIsPenalised()) {
    nextState = ATTACKER_PTR->waitForPenalty.get();
    //cout << "robot is penalized..." << endl;
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
      //cout << "robot is not localized..." << endl;
      nextState = ATTACKER_PTR->localize.get();
      return;
    }

    if (ATTACKER_PTR->shouldPlayBall()) {
      nextState = ATTACKER_PTR->playBall.get();
      //cout << "should play ball" << endl;
    } else {
      nextState = ATTACKER_PTR->goToPosition.get();
      //cout << "should go to position" << endl;
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
