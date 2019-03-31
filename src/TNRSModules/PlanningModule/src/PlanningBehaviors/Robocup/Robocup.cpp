/**
 * @file PlanningModule/PlanningBehaviors/Robocup.h
 *
 * This file implements the class Robocup.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include <boost/make_shared.hpp>
#include "BehaviorConfigs/include/PBConfigs/PBRobocupConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBGetupConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBKickConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBNavigationConfig.h"
#include "BehaviorConfigs/include/GBConfigs/GBStiffnessConfig.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "PlanningModule/include/PlanningModule.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Robocup.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/RobocupSetup.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/GoalKeeper.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/Defender.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/Attacker.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/RobocupRole.h"
#include "Utils/include/DataHolders/RoboCupGameControlData.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/TeamRobot.h"
#include "Utils/include/DataHolders/WorldBallInfo.h"
#include "Utils/include/TeamPositions.h"
#include "Utils/include/VisionUtils.h"
#include "VisionModule/include/VisionRequest.h"

Robocup::Robocup(
  PlanningModule* planningModule,
  const boost::shared_ptr<PBRobocupConfig>& config,
  const string& name) :
  PlanningBehavior(planningModule, config, name),
  inFallRecovery(false),
  readyToGetup(false),
  waitForUnpenalise(false),
  penaliseMotion(false),
  moveTarget(RobotPose2D<float>(100, 100, 100)),
  ballMotionModel(BallMotionModel::damped)
{
}

boost::shared_ptr<Robocup> Robocup::getType(
  PlanningModule* planningModule, const BehaviorConfigPtr& cfg)
{
  Robocup* r;
  cout << "Config type:" << cfg->type << endl;
  switch (cfg->type) {
    case toUType(PBRobocupTypes::robocupSetup):
      r = new RobocupSetup(planningModule, SPC(RobocupSetupConfig, cfg)); break;
    case toUType(PBRobocupTypes::goalKeeper):
      r = new GoalKeeper(planningModule, SPC(GoalKeeperConfig, cfg)); break;
    case toUType(PBRobocupTypes::defender):
      r = new Defender(planningModule, SPC(DefenderConfig, cfg)); break;
    case toUType(PBRobocupTypes::attacker):
      r = new Attacker(planningModule, SPC(AttackerConfig, cfg)); break;
  }
  return boost::shared_ptr<Robocup>(r);
}

boost::shared_ptr<PBRobocupConfig> Robocup::getBehaviorCast()
{
  return SPC(PBRobocupConfig, config);
}

bool Robocup::isLocalized()
{
  return ROBOT_LOCALIZED_IN(PlanningModule);
}

bool Robocup::ballFound()
{
  return
    WORLD_BALL_INFO_OUT(PlanningModule).found &&
    BALL_INFO_IN(PlanningModule).found;
}

bool Robocup::otherRobotOnBall()
{
  // Find if any other teammate is following ball right now and
  // is also closer.
  auto ballToRobot = norm(BALL_INFO_IN(PlanningModule).posRel);
  for (const auto& tr : TEAM_ROBOTS_IN(PlanningModule)) {
    // If any robot is intending to play the ball
    if (tr.intention == 3) {
      float dist = MathsUtils::dist(
        WORLD_BALL_INFO_OUT(PlanningModule).posWorld.x,
        WORLD_BALL_INFO_OUT(PlanningModule).posWorld.y,
        tr.pose.getX(),
        tr.pose.getY());
      // If that robot is closer to ball than this robot
      if (dist < ballToRobot) {
        return true;
      }
    }
  }
  return false;
}

void Robocup::updateRobotData()
{
  setRobotIntention();
}

void Robocup::setRobotSuggestions()
{
  /*auto& teamRobots = IVAR(vector<TeamRobot>, PlanningModule::teamRobots);
   for (int i = 0; i < teamRobots.size(); ++i) {
   // Wait for team to get localized
   if (teamRobots[i].intention == ) {
   ++teamRobotsLost;
   continue;
   }
   float dist =
   MathsUtils::dist(
   robotState.x, robotState.y,
   teamRobots[i].pose.x, teamRobots[i].pose.y
   );
   // Check if another team member has overlapping position
   if (dist < 0.2) {
   resetLocalizer();
   teamRobotOverlap = true;
   }
   }*/
}

bool Robocup::robotIsPenalised()
{
  auto& gameData = GAME_DATA_OUT(PlanningModule);
  TeamInfo& team =
    gameData.teams[(int)(gameData.teams[0].teamNumber == TEAM_NUMBER_IN(PlanningModule) ? 0 : 1)];
  return (bool) team.players[PLAYER_NUMBER_IN(PlanningModule)-1].penalty;
}

bool Robocup::waitForPenalty()
{
  if (!waitForUnpenalise) {
    if (!penaliseMotion) {
      if (robotIsPenalised()) {
        // Start penalty motion
        penaliseMotion = true;
        // Kill all motion and static behaviors
        killAllMotionBehaviors();
        killGeneralBehavior();
        return true;
      } else {
        return false;
      }
    } else {
      // Set pose to stand due to penalty
      if (this->setPostureAndStiffness(PostureState::stand, StiffnessState::robocup, MOTION_1)) {
        penaliseMotion = false;
        waitForUnpenalise = true;
      }
      return true;
    }
  } else {
    // Wait until penalty has finished
    if (!robotIsPenalised()) {
      waitForUnpenalise = false;
      return false;
    } else {
      return true;
    }
  }
}

bool Robocup::robotIsFalling()
{
  return
    ROBOT_FALLEN_IN(PlanningModule) ||
    POSTURE_STATE_IN(PlanningModule) == PostureState::fallFront ||
    POSTURE_STATE_IN(PlanningModule) == PostureState::fallingBack;
}

void Robocup::fallenRobotAction()
{
  LOG_INFO("Executing Robocup.fallenRobotAction()...")
  this->killChild();
  this->killAllMotionBehaviors();
  this->killGeneralBehavior();
  // Turn off perception modules and reset when robot is stable again
  BaseModule::publishModuleRequest(boost::make_shared<SwitchVision>(false));
  BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(false));
  inFallRecovery = true;
}

void Robocup::fallRecoveryAction()
{
  cout << "Executing Robocup.inFallRecoveryAction()..." << endl;
  auto posture = POSTURE_STATE_IN(PlanningModule);
  if (ROBOT_FALLEN_IN(PlanningModule)) {
    if (posture == PostureState::fallFront) {
      getupFromGround(KeyFrameGetupTypes::front, StiffnessState::getup, MOTION_1);
    } else if (posture == PostureState::fallBack) {
      getupFromGround(KeyFrameGetupTypes::back, StiffnessState::getup, MOTION_1);
    } else if (posture == PostureState::fallSit) {
      getupFromGround(KeyFrameGetupTypes::sit, StiffnessState::getup, MOTION_1);
    }
  } else {
    float postureTime = 2.f;
    if (posture == PostureState::fallingFront) {
      postureTime = 0.5f;
    } else if (posture == PostureState::fallingBack) {
      postureTime = 0.5f;
    }
    if (posture != PostureState::stand) {
      cout << "posture != stand" << endl;
      if (!mbInProgress()) {
        cout << "Setting posture to stand at to help robot not fall." << endl;
        auto pConfig =
          boost::make_shared<InterpToPostureConfig>();
        pConfig->targetPosture = PostureState::stand;
        pConfig->timeToReachP = postureTime;
        setupMBRequest(MOTION_1, pConfig);
      }
    } else {
      BaseModule::publishModuleRequest(boost::make_shared<SwitchVision>(true));
      BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(true));
      LOCALIZE_LAST_KNOWN_OUT(PlanningModule) = true;
      inFallRecovery = false;
    }
  }
}

void Robocup::setRobotIntention()
{
  auto role = ROBOCUP_ROLE_OUT(PlanningModule);
  if (ROBOT_LOCALIZED_IN(PlanningModule) || role == -1) {
    if (role == (int)RobocupRole::goalKeeper)
    ROBOT_INTENTION_OUT(PlanningModule) = 1;
    else if (role == (int)RobocupRole::defender || role == (int)RobocupRole::defenseSupport)
    ROBOT_INTENTION_OUT(PlanningModule) = 2;
    else if (role == (int)RobocupRole::offenseSupport || role == (int)RobocupRole::attacker)
    ROBOT_INTENTION_OUT(PlanningModule) = 3;
  } else {
    ROBOT_INTENTION_OUT(PlanningModule) = 4; // robot lost
  }
}

bool Robocup::getupFromGround(
  const KeyFrameGetupTypes& getupType,
  const StiffnessState& desStiffness,
  const unsigned& mbManagerId)
{
  if (STIFFNESS_STATE_IN(PlanningModule) != desStiffness) {
    if (!gbInProgress()) {
      auto sConfig =
        boost::make_shared <GBStiffnessConfig> (desStiffness);
      setupGBRequest(sConfig);
    }
    return false;
  } else {
    if (!mbInProgress()) {
      auto getupConfig =
        boost::make_shared<KFMGetupConfig>(getupType);
      setupMBRequest(mbManagerId, getupConfig);
      return true;
    }
    return false;
  }
}

void Robocup::setNavigationConfig(const RobotPose2D<float>& target,
  const boost::shared_ptr<InterpToPostureConfig>& startPosture,
  const boost::shared_ptr<InterpToPostureConfig>& endPosture)
{
  //if (this->getChild())
  //  return;
  auto config =
    boost::make_shared<GoToTargetConfig>();
  config->goal = target;
  config->reachClosest = true;
  if (startPosture)
    config->startPosture = startPosture;
  if (endPosture)
    config->endPosture = endPosture;
  this->moveTarget = target;
  this->setupChildRequest(config, true);
}

void Robocup::resetLocalizer()
{
  BaseModule::publishModuleRequest(boost::make_shared<ResetLocalizer>());
}

void Robocup::printGameData()
{
  auto& gameData = GAME_DATA_OUT(PlanningModule);
  cout << "gameData:" << endl;
  cout << "gameData.state: " << (int)gameData.state << endl;
  cout << "gameData.firstHalf: " << (int)gameData.firstHalf << endl;
  cout << "gameData.kickOffTeam: " << (int)gameData.kickOffTeam << endl;
  cout << "gameData.secsRemaining: " << (int)gameData.secsRemaining << endl;
  for (size_t i = 0; i < 2; ++i) {
    cout << "gameData.teams.teamNumber: " << (int)gameData.teams[i].teamNumber << endl;
    cout << "gameData.teams.teamColour: " << (int)gameData.teams[i].teamColour << endl;
    cout << "gameData.teams.score: " << (int)gameData.teams[i].score << endl;
    cout << "gameData.teams.penaltyShot: " << (int)gameData.teams[i].penaltyShot << endl;
    cout << "gameData.teams.singleShots: " << (int)gameData.teams[i].singleShots << endl;
    cout << "gameData.teams.coachSequence: " << (int)gameData.teams[i].coachSequence << endl;
    cout << "gameData.teams.coachMessage: " << gameData.teams[i].coachMessage << endl;
    cout << "gameData.teams.players[0].penalty: " << (int)gameData.teams[i].players[0].penalty << endl;
    cout << "gameData.teams.players[1].penalty: " << (int)gameData.teams[i].players[1].penalty << endl;
    cout << "gameData.teams.players[2].penalty: " << (int)gameData.teams[i].players[2].penalty << endl;
    cout << "gameData.teams.players[3].penalty: " << (int)gameData.teams[i].players[3].penalty << endl;
    cout << "gameData.teams.players[4].penalty: " << (int)gameData.teams[i].players[4].penalty << endl;
    cout << "gameData.teams.players[0].secsTillUnpenalised: " << (int)gameData.teams[i].players[0].secsTillUnpenalised << endl;
    cout << "gameData.teams.players[1].secsTillUnpenalised: " << (int)gameData.teams[i].players[1].secsTillUnpenalised << endl;
    cout << "gameData.teams.players[2].secsTillUnpenalised: " << (int)gameData.teams[i].players[2].secsTillUnpenalised << endl;
    cout << "gameData.teams.players[3].secsTillUnpenalised: " << (int)gameData.teams[i].players[3].secsTillUnpenalised << endl;
    cout << "gameData.teams.players[4].secsTillUnpenalised: " << (int)gameData.teams[i].players[4].secsTillUnpenalised << endl;
  }
}
