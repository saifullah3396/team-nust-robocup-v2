/**
 * @file PlanningBehaviors/Robocup/Types/Defender.cpp
 *
 * This file declares the class Defender.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "BehaviorConfigs/include/PBConfigs/PBRobocupConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBKickConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBDiveConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "BehaviorConfigs/include/GBConfigs/GBStiffnessConfig.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "LocalizationModule/include/LandmarkDefinitions.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/Defender.h"
#include "TNRSBase/include/DebugBase.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/TeamPositions.h"
#include "Utils/include/VisionUtils.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/RobocupRole.h"
#include "Utils/include/DataHolders/TeamRobot.h"
#include "Utils/include/DataHolders/WorldBallInfo.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "VisionModule/include/VisionRequest.h"

#define DEFENDER_PTR static_cast<Defender*>(bPtr)

Defender::Defender(
  PlanningModule* planningModule,
  const boost::shared_ptr<DefenderConfig>& config) :
  Soccer(planningModule, config, "Defender")
{
  DEFINE_FSM_STATE(Soccer, ReactDefender, react)
}

boost::shared_ptr<DefenderConfig> Defender::getBehaviorCast()
{
  return boost::static_pointer_cast<DefenderConfig>(config);
}

bool Defender::initiate()
{
  LOG_INFO("Defender.initiated() called...")
  ROBOCUP_ROLE_OUT(PlanningModule) = (int)RobocupRole::defender;
  return Soccer::initiate();
}

void Defender::ReactDefender::onRun()
{
  cout << "ReactDefender" << endl;
  if (DEFENDER_PTR->robotIsFalling()) {
    nextState = DEFENDER_PTR->fallRecovery.get();
  } else if (DEFENDER_PTR->robotIsPenalised()) {
    nextState = DEFENDER_PTR->waitForPenalty.get();
  } else {
    /*
    if (!DEFENDER_PTR->mbInProgress()) {
      auto hsConfig =
        boost::make_shared<HeadScanConfig>();
      *hsConfig =
      *boost::static_pointer_cast<HeadScanConfig>(
        DEFENDER_PTR->getBehaviorCast()->hsConfig);
      DEFENDER_PTR->setupMBRequest(MOTION_1, hsConfig);
    }
    */
    if (!DEFENDER_PTR->mbInProgress()) {
      cout << "Setting head config" << endl;
      auto httConfig =
        boost::make_shared<HeadTargetTrackConfig>();
      *httConfig =
      *boost::static_pointer_cast<HeadTargetTrackConfig>(
        DEFENDER_PTR->getBehaviorCast()->httConfig);
      DEFENDER_PTR->setupMBRequest(MOTION_1, httConfig);
    }

    if (!DEFENDER_PTR->isLocalized()) {
      nextState = DEFENDER_PTR->localize.get();
      return;
    }

    if (DEFENDER_PTR->shouldPlayBall()) {
      nextState = DEFENDER_PTR->playBall.get();
    } else {
      nextState = DEFENDER_PTR->goToPosition.get();
    }
  }
}

bool Defender::ballInRange()
{
  static auto ballReactionLimitX = Matrix<float, 2, 1>(fieldMinX, fieldMaxX);
  static auto ballReactionLimitY = Matrix<float, 2, 1>(fieldMinY, fieldMaxY);
  auto ballWorld = WORLD_BALL_INFO_OUT(PlanningModule).posWorld;
  if (ballWorld.x < ballReactionLimitX[0] || ballWorld.x > ballReactionLimitX[1] ||
      ballWorld.y < ballReactionLimitY[0] || ballWorld.y > ballReactionLimitY[1]
     )
  {
    return false;
  }
  return true;
}

cv::Point_<float> Defender::findBallKickTarget()
{
  LOG_INFO("Finding ball kick target...")
  bool teammateFound = false;
  cv::Point_<float> kickTarget;
  const auto& pose = ROBOT_POSE_2D_IN(PlanningModule);
  kickTarget.x = pose.getX();
  kickTarget.y = pose.getY();
  ///< Find a teammate to pass the ball
  for (const auto& tr : TEAM_ROBOTS_IN(PlanningModule)) {
    ///< Find robots
    if (tr.positionConfidence > 60) {
      ///< Team robot is ahead of this robot. We don't want to send the ball behind
      if (tr.pose.getX() > kickTarget.x) {
        if (!behindObstacle(kickTarget)) {
          kickTarget.x = tr.pose.getX();
          kickTarget.y = tr.pose.getY();
          teammateFound = true;
        }
      }
    }
  }

  if (teammateFound) {
    LOG_INFO("Passing to teammate at:" << kickTarget)
    return kickTarget;
  } else {
    ///< Assign opponent goal as the target of the robot. Opponent goal is in positive X
    kickTarget = cv::Point_<float>(goalPostX, MathsUtils::sign(pose.getY()) * goalPostY / 2);
    if (behindObstacle(kickTarget)) { ///< Whether the target is behind an obstacle
      auto lTarget = kickTarget;
      auto rTarget = kickTarget;
      lTarget.y += 0.2;
      rTarget.y -= 0.2;
      while (true) {
        if (behindObstacle(lTarget)) {
          lTarget.y += 0.2;
          if (behindObstacle(rTarget)) {
            rTarget.y -= 0.2;
          } else {
            return rTarget;
          }
        } else {
          return lTarget;
        }
      }
    } else {
      LOG_INFO("Kicking to:" << kickTarget)
      return kickTarget;
    }
  }
  return cv::Point_<float>(0.0, 0.0);
}
