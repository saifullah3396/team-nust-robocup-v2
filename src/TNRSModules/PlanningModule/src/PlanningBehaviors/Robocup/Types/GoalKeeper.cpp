/**
 * @file PlanningModule/PlanningBehaviors/GoalKeeper.h
 *
 * This file declares the class GoalKeeper.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "LocalizationModule/include/LocalizationRequest.h"
#include "LocalizationModule/include/LandmarkDefinitions.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/GoalKeeper.h"
#include "TNRSBase/include/DebugBase.h"
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
#include "Utils/include/Solvers/MotionEquationSolver.h"
#include "VisionModule/include/VisionRequest.h"

#define GOAL_KEEPER_PTR static_cast<GoalKeeper*>(bPtr)

boost::shared_ptr<GoalKeeperConfig> GoalKeeper::getBehaviorCast()
{
  return boost::static_pointer_cast<GoalKeeperConfig>(config);
}

void GoalKeeper::initiate()
{
  LOG_INFO("GoalKeeper.initiated() called...")
  Soccer::initiate();
  ROBOCUP_ROLE_OUT(PlanningModule) = (int)RobocupRole::GOALKEEPER;
  inBehavior = true;
}

bool GoalKeeper::ballInRange()
{
  static auto ballReactionLimitX = Matrix<float, 2, 1>(fieldMinX, fieldMinX + 1.5);
  static auto ballReactionLimitY = Matrix<float, 2, 1>(penaltyBoxMinY - 0.5, penaltyBoxMaxY + 0.5);
  auto& ballWorld = WORLD_BALL_INFO_OUT(PlanningModule).posWorld;
  if (ballWorld.x < ballReactionLimitX[0] || ballWorld.x > ballReactionLimitX[1] ||
      ballWorld.y < ballReactionLimitY[0] || ballWorld.y > ballReactionLimitY[1]
     )
  {
    return false;
  }
  return true;
}

void GoalKeeper::ReactGoalKeeper::onRun()
{
  cout << "GoalKeeper::ReactGoalKeeper::onRun()" << endl;
  if (GOAL_KEEPER_PTR->robotIsFalling()) {
    nextState = GOAL_KEEPER_PTR->fallRecovery.get();
  } else if (GOAL_KEEPER_PTR->robotIsPenalised()) {
    nextState = GOAL_KEEPER_PTR->waitForPenalty.get();
  } else {
    /*
    if (!GOAL_KEEPER_PTR->mbInProgress()) {
      auto hsConfig =
        boost::make_shared<HeadScanConfig>();
      *hsConfig =
      *boost::static_pointer_cast<HeadScanConfig>(
        GOAL_KEEPER_PTR->getBehaviorCast()->hsConfig);
      GOAL_KEEPER_PTR->setupMBRequest(MOTION_1, hsConfig);
    }
    */
    if (!GOAL_KEEPER_PTR->mbInProgress()) {
      auto httConfig =
        boost::make_shared<HeadTargetTrackConfig>();
      *httConfig =
      *boost::static_pointer_cast<HeadTargetTrackConfig>(
        GOAL_KEEPER_PTR->getBehaviorCast()->httConfig);
      GOAL_KEEPER_PTR->setupMBRequest(MOTION_1, httConfig);
    }

    if (!GOAL_KEEPER_PTR->isLocalized()) {
      nextState = GOAL_KEEPER_PTR->localize.get();
      return;
    }

    if (GOAL_KEEPER_PTR->shouldPlayBall()) {
      nextState = GOAL_KEEPER_PTR->playBall.get();
    } else if (GOAL_KEEPER_PTR->ballFound()) {
      Vector2f endPosition;
      float timeToReach;
      if (GOAL_KEEPER_PTR->ballIncoming(endPosition, timeToReach)) {
        //cout << "time to reach:" << timeToReach << endl;
        //cout << "endPosition:" << endPosition << endl;
        //! If ball will reach within 'diveReactionTime' seconds, try to dive.
        if (timeToReach <= GOAL_KEEPER_PTR->diveReactionTime) {
          GOAL_KEEPER_PTR->interceptTarget.x() = -penaltyBoxMidX;
          GOAL_KEEPER_PTR->interceptTarget.y() = endPosition[1];
          GOAL_KEEPER_PTR->interceptTarget.theta() = 0.f;
          if (GOAL_KEEPER_PTR->divePossible()) {
            nextState = GOAL_KEEPER_PTR->dive.get();
          } else {
            //! if ball is too far for dive, move to intercept.
            nextState = GOAL_KEEPER_PTR->interceptBall.get();
          }
        } else {
          //! if ball is slow and does not need fast reaction
          nextState = GOAL_KEEPER_PTR->interceptBall.get();
        }
      } else {
        //! if ball is not cross the goal
        GOAL_KEEPER_PTR->interceptTarget.x() = -penaltyBoxMidX;
        GOAL_KEEPER_PTR->interceptTarget.theta() = 0.f;
        static constexpr float maxYPosition = 0.5f;
        if (fabsf(endPosition[1]) > maxYPosition)
          GOAL_KEEPER_PTR->interceptTarget.y() =
            MathsUtils::sign(endPosition[1]) * maxYPosition;
        else
          GOAL_KEEPER_PTR->interceptTarget.y() = endPosition[1];
        nextState = GOAL_KEEPER_PTR->interceptBall.get();
      }
    } else {
      //! Go to start position if ball is not found
      nextState = GOAL_KEEPER_PTR->goToPosition.get();
    }
  }
}

bool GoalKeeper::ballIncoming(Vector2f& endPosition, float& timeToReach)
{
  auto& ballWorld = WORLD_BALL_INFO_OUT(PlanningModule).posWorld;
  auto& ballWorldVel = WORLD_BALL_INFO_OUT(PlanningModule).velWorld;
  MotionEquationSolver* meSolver;
  if (ballMotionModel == BallMotionModel::DAMPED) {
    meSolver = new DampedMESolver(
      Vector2f (-penaltyBoxMaxX, 0.0),
      Vector2f (ballWorld.x, ballWorld.y),
      Vector2f (ballWorldVel.x, ballWorldVel.y),
      coeffDamping
    );
  } else {
    meSolver = new FrictionMESolver(
      Vector2f (-penaltyBoxMaxX, 0.0),
      Vector2f (ballWorld.x, ballWorld.y),
      Vector2f (ballWorldVel.x, ballWorldVel.y),
      coeffFriction
    );
  }
  meSolver->optDef();
  endPosition = meSolver->getEndPosition();
  if (endPosition[0] <= -penaltyBoxMinX &&
      endPosition[1] >= -goalPostY &&
      endPosition[1] <= goalPostY)
  {
    timeToReach = meSolver->getTimeToReach();
    delete meSolver;
    return true;
  } else {
    timeToReach = meSolver->getTimeToReach();
    delete meSolver;
    return false;
  }
}

void GoalKeeper::InterceptBall::onStart()
{
  cout << "Soccer::InterceptBall::onStart()" << endl;
  if (GOAL_KEEPER_PTR->goingToTarget != GoingToTarget::NONE &&
      GOAL_KEEPER_PTR->goingToTarget != GoingToTarget::INTERCEPT_BALL)
    GOAL_KEEPER_PTR->killChild();
}

void GoalKeeper::InterceptBall::onRun()
{
  cout << "Soccer::InterceptBall::onRun()" << endl;
  if (GOAL_KEEPER_PTR->robotIsFalling()) {
    nextState = GOAL_KEEPER_PTR->fallRecovery.get();
  } else if (GOAL_KEEPER_PTR->robotIsPenalised()) {
    nextState = GOAL_KEEPER_PTR->waitForPenalty.get();
  } else {
    if (GOAL_KEEPER_PTR->goingToTarget != GoingToTarget::INTERCEPT_BALL) {
      if (
          (ROBOT_POSE_2D_IN_REL(PlanningModule, GOAL_KEEPER_PTR) - GOAL_KEEPER_PTR->interceptTarget).get().norm() >
          GOAL_KEEPER_PTR->interceptBallPosTol)
      {
        cout << "Setting navigation config..." << endl;
        GOAL_KEEPER_PTR->setNavigationConfig(GOAL_KEEPER_PTR->interceptTarget);
        GOAL_KEEPER_PTR->goingToTarget = GoingToTarget::INTERCEPT_BALL;
        ROBOT_INTENTION_OUT_REL(PlanningModule, GOAL_KEEPER_PTR) = 1;
      }
    } else {
      if (
          (GOAL_KEEPER_PTR->moveTarget.get() - GOAL_KEEPER_PTR->interceptTarget.get()).norm() >
          GOAL_KEEPER_PTR->interceptBallPosTol || !GOAL_KEEPER_PTR->getChild())
      {
        GOAL_KEEPER_PTR->goingToTarget = GoingToTarget::NONE;
      }
    }
    nextState = GOAL_KEEPER_PTR->react.get();
  }
}

bool GoalKeeper::divePossible()
{
  auto pose = ROBOT_POSE_2D_IN(PlanningModule);
  Point2f wInRobot;
  //wInRobot.x = -pose.getX() * pose.cosT - pose.getY() * pose.sinT;
  wInRobot.y = pose.getX() * pose.st - pose.getY() * pose.ct;
  Point2f ssInRobot;
  //ssInRobot.x = wInRobot.x + saveSpot.x * ct + saveSpot.y * st;
  //cout << "robot sint: " << pose.st << endl;
  //cout << "robot ciost: " << pose.ct << endl;
  ssInRobot.y =
    wInRobot.y - interceptTarget.getX() * pose.st + interceptTarget.getY() * pose.ct;
  //cout << "winrobot: " << wInRobot.y <
  //cout << "ssInRobot: " << ssInRobot << endl;
  //cout << "interceptTarget: " << interceptTarget.get() << endl;
  if (ssInRobot.y > 0) {
    //if (ssInRobot.y <= 0.1) {
    //  lastDiveTargetType = toUType(KeyFrameDiveTypes::IN_PLACE);
    //  return true;
    //} else if (ssInRobot.y > 0.1 && ssInRobot.y <= 0.2) {
    lastDiveTargetType = toUType(KeyFrameDiveTypes::LEFT);
    return true;
    //}
  } else {
    //if (ssInRobot.y > -0.1) {
     // lastDiveTargetType = toUType(KeyFrameDiveTypes::IN_PLACE);
    //  return true;
    //} else if (ssInRobot.y < -0.1 && ssInRobot.y <= -0.2) {
    lastDiveTargetType = toUType(KeyFrameDiveTypes::RIGHT);
    return true;
    //}
  }
  return false;
}

void GoalKeeper::Dive::onStart()
{
  GOAL_KEEPER_PTR->killMotionBehavior(MOTION_1);
  GOAL_KEEPER_PTR->killChild();
}

void GoalKeeper::Dive::onRun()
{
  LOG_INFO("Exeucting GoalKeeper.diveAction()")
  if (GOAL_KEEPER_PTR->robotIsFalling()) {
    nextState = GOAL_KEEPER_PTR->fallRecovery.get();
  } else if (GOAL_KEEPER_PTR->robotIsPenalised()) {
    nextState = GOAL_KEEPER_PTR->waitForPenalty.get();
  } else {
    cout << "lastDiveTargetType: " << GOAL_KEEPER_PTR->lastDiveTargetType << endl;
    if (GOAL_KEEPER_PTR->lastDiveTargetType != -1) {
      if (!GOAL_KEEPER_PTR->mbInProgress()) {
        auto dConfig =
          boost::make_shared <KFMDiveConfig> (
            (KeyFrameDiveTypes) GOAL_KEEPER_PTR->lastDiveTargetType);
        GOAL_KEEPER_PTR->setupMBRequest(MOTION_1, dConfig);
        GOAL_KEEPER_PTR->lastDiveTargetType = -1;
        nextState = GOAL_KEEPER_PTR->postDive.get();
      }
    }
  }
}

void GoalKeeper::PostDive::onRun()
{
  LOG_INFO("PostDive::onRun()...")
  if (GOAL_KEEPER_PTR->robotIsFalling()) {
    nextState = GOAL_KEEPER_PTR->fallRecovery.get();
  } else if (GOAL_KEEPER_PTR->robotIsPenalised()) {
    nextState = GOAL_KEEPER_PTR->waitForPenalty.get();
  } else {
    if (!GOAL_KEEPER_PTR->mbInProgress()) {
      auto pConfig =
        boost::make_shared<MBPostureConfig>(
          PostureState::STAND_HANDS_BEHIND, 2.f);
      GOAL_KEEPER_PTR->setupMBRequest(MOTION_1, pConfig);
      nextState = GOAL_KEEPER_PTR->react.get();
    }
  }
}

/*
Point_<float> GoalKeeper::findBallKickTarget()
{
  LOG_INFO("Finding ball kick target...")
  bool teammateFound = false;
  Point_<float> kickTarget;
  kickTarget.x = ROBOT_POSE_2D.getX();
  kickTarget.y = ROBOT_POSE_2D.getY();
  //! Find a teammate to pass the ball
  for (const auto& robot : TEAM_ROBOTS) {
    //! Find robots
    if (robot.positionConfidence > 60) {
      //! Team robot is ahead of this robot. We don't want to send the ball behind
      if (robot.pose.getX() > kickTarget.x) {
        if (!behindObstacle(kickTarget)) {
          kickTarget.x = robot.pose.getX();
          kickTarget.y = robot.pose.getY();
          teammateFound = true;
        }
      }
    }
  }

  if (teammateFound) {
    LOG_INFO("Passing to teammate at:" << kickTarget)
    return kickTarget;
  } else {
    //! Assign opponent goal as the target of the robot. Opponent goal is in positive X
    kickTarget = Point_<float>(goalPostX, MathsUtils::sign(ROBOT_POSE_2D.getY()) * goalPostY / 2);
    if (behindObstacle(kickTarget)) { //! Whether the target is behind an obstacle
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
  return Point_<float>(0.0, 0.0);
}*/
