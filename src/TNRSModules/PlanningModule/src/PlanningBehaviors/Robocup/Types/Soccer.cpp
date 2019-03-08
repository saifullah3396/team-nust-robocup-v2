/**
 * @file PlanningModule/src/PlanningBehaviors/Robocup/Types/Soccer.cpp
 *
 * This file implements the class Soccer
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "LocalizationModule/include/LandmarkDefinitions.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/Soccer.h"
#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/Types/GoToTarget.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/Behaviors/MBConfigs/MBKickConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBDiveConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBPostureConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBHeadControlConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBGetupConfig.h"
#include "Utils/include/Behaviors/SBConfigs/SBStiffnessConfig.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/RobocupRole.h"
#include "Utils/include/DataHolders/TeamRobot.h"
#include "Utils/include/DataHolders/WorldBallInfo.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/TNRSLine.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/TeamPositions.h"
#include "VisionModule/include/VisionRequest.h"

#define BOOST_SPC(x) boost::static_pointer_cast<x>

float Soccer::coeffDamping;
float Soccer::coeffFriction;

boost::shared_ptr<PBRobocupConfig> Soccer::getBehaviorCast()
{
  return boost::static_pointer_cast <PBRobocupConfig> (config);
}

void Soccer::loadExternalConfig() {
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG("EnvProperties",
      (float, coeffDamping, coeffDamping),
      (float, coeffRF, coeffFriction),
    )
    loaded = true;
  }
}

void Soccer::initiate()
{
  LOG_INFO("Soccer.initiated() called...")
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(true, FeatureExtractionIds::segmentation));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(true, FeatureExtractionIds::field));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(true, FeatureExtractionIds::robot));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(true, FeatureExtractionIds::lines));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(true, FeatureExtractionIds::goal));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(true, FeatureExtractionIds::BALL));
  if (getBehaviorCast()->startPoseKnown) {
    BaseModule::publishModuleRequest(
      boost::make_shared<InitiateLocalizer>(
        getBehaviorCast()->startPose)
    );
  }
  LOCALIZE_LAST_KNOWN_OUT(PlanningModule) = getBehaviorCast()->startPoseKnown;
  ON_SIDE_LINE_OUT(PlanningModule) =
    getBehaviorCast()->onSideAtStart;
  BaseModule::publishModuleRequest(boost::make_shared<SwitchVision>(true));
  BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(true));
}

void Soccer::update()
{
  if (this->getChild())
    boost::static_pointer_cast<PlanningBehavior>(this->getChild())->setMBIdOffset(MOTION_1+1);

  //! Update required input data
  updateRobotData(); /// Tested

  //! Stop if a motion or static behavior request is in progress
  if (requestInProgress()) return; /// Tested

  //! Print game state
  //! printGameData();

  fsm->update();
}

void Soccer::finish()
{
  this->killChild();
  this->killAllMotionBehaviors();
  inBehavior = false;
}

void Soccer::SetPosture::onRun()
{
  cout << "SetPosture" << endl;
  if (bPtr->robotIsFalling()) {
    nextState = bPtr->fallRecovery.get();
  } else if (bPtr->robotIsPenalised()) {
    nextState = bPtr->waitForPenalty.get();
  } else {
    if (bPtr->setPostureAndStiffness(
        PostureState::STAND_HANDS_BEHIND,
        StiffnessState::robocup, MOTION_1))
    {
      nextState = bPtr->localize.get();
    }
  }
}

void Soccer::Localize::onStart()
{
  //cout << "Soccer::Localize::onStart()" << endl;
  bPtr->killChild();
  bPtr->killMotionBehavior(MOTION_1);
}

void Soccer::Localize::onRun()
{
  cout << "Localize" << endl;
  //cout << "Soccer::Localize::onRun()" << endl;
  if (bPtr->robotIsFalling()) {
    nextState = bPtr->fallRecovery.get();
  } else if (bPtr->robotIsPenalised()) {
    nextState = bPtr->waitForPenalty.get();
  } else {
    if (!bPtr->isLocalized()) {
      if (!bPtr->mbInProgress()) {
        bPtr->setupMBRequest(MOTION_1, bPtr->getBehaviorCast()->hsConfig);
      }
    } else {
      nextState = bPtr->react.get();
    }
  }
}

void Soccer::Localize::onStop()
{
  //cout << "Soccer::Localize::onStop()" << endl;
  bPtr->killMotionBehavior(MOTION_1);
}

void Soccer::GoToPosition::onStart()
{
  //cout << "Soccer::GoToPosition::onRun()" << endl;
  if (bPtr->goingToTarget != GoingToTarget::NONE &&
      bPtr->goingToTarget != GoingToTarget::INIT_POSITION)
    bPtr->killChild();
}

void Soccer::GoToPosition::onRun()
{
  cout << "GoToPosition" << endl;
  if (bPtr->robotIsFalling()) {
    nextState = bPtr->fallRecovery.get();
  } else if (bPtr->robotIsPenalised()) {
    nextState = bPtr->waitForPenalty.get();
  } else {
    RobotPose2D<float> target;
    target = positionsInGame[ROBOCUP_ROLE_OUT_REL(PlanningModule, bPtr)];
    if (bPtr->goingToTarget != GoingToTarget::INIT_POSITION) {
      if ((ROBOT_POSE_2D_IN_REL(PlanningModule, bPtr) - target).get().norm() > bPtr->goalPosTol) {
        bPtr->setNavigationConfig(target);
        ROBOT_INTENTION_OUT_REL(PlanningModule, bPtr) = 1;
        bPtr->goingToTarget = GoingToTarget::INIT_POSITION;
      } else if (fabsf(ROBOT_POSE_2D_IN_REL(PlanningModule, bPtr).getTheta() - target.getTheta()) > bPtr->goalAngleTol) {
        target.x() = ROBOT_POSE_2D_IN_REL(PlanningModule, bPtr).getX();
        target.y() = ROBOT_POSE_2D_IN_REL(PlanningModule, bPtr).getY();
        bPtr->setNavigationConfig(target);
        ROBOT_INTENTION_OUT_REL(PlanningModule, bPtr) = 1;
        bPtr->goingToTarget = GoingToTarget::INIT_POSITION;
      }
    } else {
      if ((bPtr->moveTarget.get() - target.get()).norm() > bPtr->goalPosTol || !bPtr->getChild()) {
        bPtr->goingToTarget = GoingToTarget::NONE;
      }
    }
    nextState = bPtr->react.get();
  }
}

void Soccer::PlayBall::onStart()
{
  cout << "PlayBall" << endl;
  //cout << "Soccer::PlayBall::onStart()" << endl;
  if (bPtr->goingToTarget != GoingToTarget::NONE &&
      bPtr->goingToTarget != GoingToTarget::BALL) {
    bPtr->killChild();
  }
  /*if (bPtr->matchLastMotionRequest(
        (int)MBIds::headControl, (int)MBHeadControlTypes::headTargetTrack))
  {
    if (BOOST_SPC(HeadTargetTrackConfig)(bPtr->lastMBConfig)->headTargetType !=
        HeadTargetTypes::ball)
    {
      bPtr->killMotionBehavior(MOTION_1);
    }
  } else {
    bPtr->killMotionBehavior(MOTION_1);
  }*/
}

void Soccer::PlayBall::onRun()
{
  cout << "PlayBall" << endl;
  if (bPtr->robotIsFalling()) {
    nextState = bPtr->fallRecovery.get();
  } else if (bPtr->robotIsPenalised()) {
    nextState = bPtr->waitForPenalty.get();
  } else {
    //! Start tracking ball
    if (!bPtr->mbInProgress()) {
      auto httConfig =
        boost::make_shared<HeadTargetTrackConfig>();
      *httConfig =
      *boost::static_pointer_cast<HeadTargetTrackConfig>(
        bPtr->getBehaviorCast()->httConfig);
      bPtr->setupMBRequest(MOTION_1, httConfig);
    }
    //cout << "Soccer::PlayBall::onRun()" << endl;
    ROBOT_INTENTION_OUT_REL(PlanningModule, bPtr) = 3;
    // Send robot at distance 'ballKickDist'm from the ball and angle 'ballKickAngle'm.
    //if (bPtr->goingToTarget != GoingToTarget::BALL) {
    float dist = norm(BALL_INFO_IN_REL(PlanningModule, bPtr).posRel);
    //cout << "dist: " << dist << endl;
    if (dist > bPtr->ballKickDist + 0.05) { // If ball is greater than 'ballKickDist'm far
      bPtr->goToBall(bPtr->ballKickDist);
      nextState = bPtr->react.get();
    } else {
      bPtr->lastKickTarget = bPtr->findBallKickTarget();
      nextState = bPtr->alignToKick.get();
    }
    //} else {
     // auto& velWorld = WORLD_BALL_INFO_IN_REL(PlanningModule, bPtr).velWorld;
     // cout << "norm(velWorld): " << norm(velWorld) << endl;
     // if (norm(velWorld) > bPtr->ballMovedVelMin || !bPtr->getChild()) {
     //   bPtr->goingToTarget = GoingToTarget::NONE;
    //  }
      //nextState = bPtr->react.get();
    //}
  }
}

void Soccer::AlignToKick::onStart()
{
  cout << "AlignToKick" << endl;
  //cout << "Soccer::AlignToKick::onStart()" << endl;
  if (bPtr->goingToTarget != GoingToTarget::NONE &&
      bPtr->goingToTarget != GoingToTarget::KICK_ALIGNMENT)
    bPtr->killChild();
}

void Soccer::AlignToKick::onRun()
{
  cout << "AlignToKick" << endl;
  if (bPtr->robotIsFalling()) {
    nextState = bPtr->fallRecovery.get();
  } else if (bPtr->robotIsPenalised()) {
    nextState = bPtr->waitForPenalty.get();
  } else {
    //cout << "Soccer::AlignToKick::onRun()" << endl;
    //! Align to pass the ball to best target
    if (bPtr->goingToTarget != GoingToTarget::KICK_ALIGNMENT) {
      RobotPose2D<float> target;
      bPtr->findBestBallAlignment(target);
      bPtr->setNavigationConfig(target);
      bPtr->goingToTarget = GoingToTarget::KICK_ALIGNMENT;
    } else {
      if (!bPtr->getChild()) {
        //cout << "BALL_INFO.posRel: " << BALL_INFO_IN_REL(PlanningModule, bPtr).posRel << endl;
        //cout << "WORLD_BALL_INFO.posWorld: " << WORLD_BALL_INFO_IN_REL(PlanningModule, bPtr).posWorld << endl;
        bPtr->goingToTarget = GoingToTarget::NONE;
        // Add this to check whether robot is in correct position after alignment
        if (bPtr->alignedToKick()) {
          //cout << "Aligned ..." << endl;
          nextState = bPtr->kickBall.get();
        } else {
          nextState = bPtr->react.get();
        }
      }
    }
  }
}

void Soccer::KickBall::onStart()
{
  //cout << "Soccer::KickBall::onStart()" << endl;
  bPtr->killMotionBehavior(MOTION_1);
}

void Soccer::KickBall::onRun()
{
  static bool kickSet = false;
  if (bPtr->robotIsFalling()) {
    nextState = bPtr->fallRecovery.get();
  } else if (bPtr->robotIsPenalised()) {
    nextState = bPtr->waitForPenalty.get();
  } else {
    cout << "Soccer::KickBall::onRun()" << endl;
    if (kickSet) {
      if (!bPtr->mbInProgress()) {
        nextState = bPtr->setPosture.get();
        kickSet = false;
        return;
      }
    }

    if (BALL_INFO_IN_REL(PlanningModule, bPtr).found) {
      if (!bPtr->mbInProgress()) {
        auto kConfig =
          boost::make_shared<JSOImpKickConfig>(BALL_INFO_IN_REL(PlanningModule, bPtr).posRel);
        kConfig->reqVel = Point2f(0.75, 0.0);
        bPtr->setupMBRequest(MOTION_1, kConfig);
        kickSet = true;
      }
    } else {
      nextState = bPtr->react.get();
    }
  }
}

void Soccer::FallRecovery::onStart()
{
  cout << "Soccer::fallRecovery::onStart()..." << endl;
  bPtr->killChild();
  bPtr->killAllMotionBehaviors();
  bPtr->killStaticBehavior();
  // Turn off perception modules and reset when robot is stable again
  BaseModule::publishModuleRequest(boost::make_shared<SwitchVision>(false));
  BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(false));
}

void Soccer::FallRecovery::onRun()
{
  cout << "Soccer::fallRecovery::onRun()..." << endl;
  if (ROBOT_FALLEN_IN_REL(PlanningModule, bPtr)) {
    nextState = bPtr->getup.get();
  } else {
    auto posture = POSTURE_STATE_IN_REL(PlanningModule, bPtr);
    float postureTime = 2.f;
    if (posture == PostureState::FALLING_FRONT) {
      postureTime = 0.5f;
    } else if (posture == PostureState::FALLING_BACK) {
      postureTime = 0.5f;
    }
    if (posture != PostureState::STAND) {
      if (!bPtr->mbInProgress()) {
        auto pConfig =
          boost::make_shared<MBPostureConfig>(
            PostureState::STAND, postureTime);
        bPtr->setupMBRequest(MOTION_1, pConfig);
      }
    } else {
      BaseModule::publishModuleRequest(boost::make_shared<SwitchVision>(true));
      BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(true));
      LOCALIZE_LAST_KNOWN_OUT_REL(PlanningModule, bPtr) = true;
      nextState = bPtr->setPosture.get();
    }
  }
}

void Soccer::Getup::onStart()
{
  cout << "Soccer::Getup::onStart()..." << endl;
  bPtr->killChild();
  bPtr->killAllMotionBehaviors();
  bPtr->killStaticBehavior();
  // Turn off perception modules and reset when robot is stable again
  BaseModule::publishModuleRequest(boost::make_shared<SwitchVision>(false));
  BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(false));
}

void Soccer::Getup::onRun()
{
  static bool getupCmdSent = false;
  if (getupCmdSent) {
    if (!bPtr->mbInProgress()) { //! Finished
      nextState = bPtr->fallRecovery.get();
      getupCmdSent = false;
    }
  } else {
    if (!bPtr->mbInProgress()) {
      auto posture = POSTURE_STATE_IN_REL(PlanningModule, bPtr);
      if (posture == PostureState::FALL_FRONT) {
        if (bPtr->getupFromGround(KeyFrameGetupTypes::FRONT, StiffnessState::GETUP, MOTION_1))
          getupCmdSent= true;
      } else if (posture == PostureState::FALL_BACK) {
        if (bPtr->getupFromGround(KeyFrameGetupTypes::BACK, StiffnessState::GETUP, MOTION_1))
          getupCmdSent= true;
      } else if (posture == PostureState::FALL_SIT) {
        if (bPtr->getupFromGround(KeyFrameGetupTypes::SIT, StiffnessState::GETUP, MOTION_1))
          getupCmdSent= true;
      }
    }
  }
}

void Soccer::WaitForPenalty::onStart()
{
  BaseModule::publishModuleRequest(boost::make_shared<SwitchVision>(false));
  BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(false));
  //! Localization has to be reset as well
  LOCALIZE_LAST_KNOWN_OUT_REL(PlanningModule, bPtr) = false;
}

void Soccer::WaitForPenalty::onRun()
{
  static bool penaliseMotionSet = false;
  if (!penaliseMotionSet) {
    if (bPtr->setPostureAndStiffness(
          PostureState::STAND, StiffnessState::robocup, MOTION_1))
    {
      penaliseMotionSet = true;
    }
  }
  if (penaliseMotionSet && !bPtr->robotIsPenalised()) {
    penaliseMotionSet = false;
    nextState = bPtr->setPosture.get();
  }
}

bool Soccer::shouldPlayBall()
{
  static unsigned ballLostCount = 0;
  if (goingToTarget == GoingToTarget::BALL && !ballFound()) {
      ++ballLostCount;
    return ballLostCount > 20 ? false : true;
  }
  if (!ballFound()) {
    cout << "ball not found..." << endl;
    return false;
  }
  if (!ballInRange())
    return false;
  if (otherRobotOnBall()) {
    return false;
  }
  return true;
}

void Soccer::goToBall(const float& distFromBall)
{
  //cout << "Soccer::goToBall: " << endl;
  RobotPose2D<float> target;
  //auto angle =
  //    atan2(
  //      WORLD_BALL_INFO_OUT(PlanningModule).posWorld.y - ROBOT_POSE_2D.getY(),
  //      WORLD_BALL_INFO.posWorld.x - ROBOT_POSE_2D.getX());
  //cout << "angle: " << angle * 180 / M_PI << endl;
  auto ballWorld = WORLD_BALL_INFO_OUT(PlanningModule).posWorld;
  target.x() = ballWorld.x - distFromBall * cos(0.0);
  target.y() = ballWorld.y - distFromBall * sin(0.0);
  target.theta() = 0.0;
  setNavigationConfig(target);
  goingToTarget = GoingToTarget::BALL;
}

Point2f Soccer::findBallKickTarget()
{
  //LOG_INFO("Finding ball kick target...")
  bool teammateFound = false;
  cv::Point_<float> kickTarget;
  kickTarget.x = ROBOT_POSE_2D_IN(PlanningModule).getX();
  kickTarget.y = ROBOT_POSE_2D_IN(PlanningModule).getY();
  //! Find a teammate to pass the ball
  for (const auto& robot : TEAM_ROBOTS_IN(PlanningModule)) {
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
    kickTarget = cv::Point_<float>(goalPostX, MathsUtils::sign(ROBOT_POSE_2D_IN(PlanningModule).getY()) * goalPostY / 2);
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
  return cv::Point_<float>(0.0, 0.0);
}

void Soccer::findBestBallAlignment(RobotPose2D<float>& alignPosition)
{
  auto ballWorld = WORLD_BALL_INFO_OUT(PlanningModule).posWorld;
  auto diff = lastKickTarget - ballWorld;
  auto angle = 0;//atan2(diff.y, diff.x);
  auto ca = cos(angle);
  auto sa = sin(angle);
  float xOffset = -0.17;
  float yOffset = 0.05; // Right foot alignment
  //if (BALL_INFO.posRel.y > 0) {
  //  yOffset *= -1.0; // Left foot alignment
  //}
  alignPosition.x() = ballWorld.x + xOffset * ca - yOffset * sa; // [cos -sin 0 x] * [-xo yo 0 1]^T
  alignPosition.y() = ballWorld.y + xOffset * sa + yOffset * ca; // [sin  cos 0 y]
  alignPosition.theta() = angle;                                                // [0      0 1 0]
}

bool Soccer::alignedToKick()
{
  auto ballRel = BALL_INFO_IN(PlanningModule).posRel;
  static float midToFootDist = 0.0475;
  static constexpr float kickAlignmentTol = 0.02;
  if (ballRel.x > 0.18 ||
      fabsf(ballRel.y - midToFootDist) > kickAlignmentTol && //! Left foot
      fabsf(ballRel.y + midToFootDist) > kickAlignmentTol) //! Right foot
    return false;
  return true;
}

bool Soccer::behindObstacle(const Point2f& target)
{
  auto pose = ROBOT_POSE_2D_IN(PlanningModule);
  TNRSLine<float> targetLine;
  targetLine.p1 = Point2f(pose.getX(), pose.getY());
  for (auto& obs : OBSTACLES_OBS_IN(PlanningModule).data) {
    targetLine.p2 = target;
    Point2f inter;
    if (obs.frontT.findIntersection(targetLine, inter)) {
      Point2f diffInter = inter - obs.frontT.p1;
      auto magInter = norm(diffInter);
      auto unitInter = Point2f(diffInter.x / magInter, diffInter.y / magInter);
      if (unitInter.x / obs.frontT.unit.x < 0) {
        continue;
      } else {
        float r = magInter / obs.frontT.d;
        if (r > 0.0 && r < 1.0) {
          return true;
        }
      }
    }
  }
  return false;
}

