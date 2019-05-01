/**
 * @file PlanningBehaviors/Robocup/Types/RobocupSetup.cpp
 *
 * This file implements the class RobocupSetupSetup.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 27 Jul 2018
 */

#include "LocalizationModule/include/LocalizationRequest.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/RobocupSetup.h"
#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/Types/GoToTarget.h"
#include "BehaviorConfigs/include/PBConfigs/PBRobocupConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "BehaviorConfigs/include/GBConfigs/GBWDConfig.h"
#include "TNRSBase/include/DebugBase.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/DataHolders/RobocupRole.h"
#include "Utils/include/DataHolders/RoboCupGameControlData.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/TeamRobot.h"
#include "Utils/include/TeamPositions.h"
#include "VisionModule/include/VisionRequest.h"

RobocupSetup::RobocupSetup(
  PlanningModule* planningModule,
  const boost::shared_ptr<RobocupSetupConfig>& config) :
  Robocup(planningModule, config, "RobocupSetup"),
  behaviorState(startup),
  setSequenceFinished(false)
{
}

boost::shared_ptr<RobocupSetupConfig> RobocupSetup::getBehaviorCast()
{
  return boost::static_pointer_cast <RobocupSetupConfig> (config);
}

bool RobocupSetup::initiate()
{
  LOG_INFO("RobocupSetup.initiate() called...");
  ///< Enable all feature extraction modules
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
    boost::make_shared<SwitchFeatureExtModule>(true, FeatureExtractionIds::ball));
  return true;
}

void RobocupSetup::update()
{
  LOG_INFO("In RobocupSetup.update()...");
  // Update required input data
  updateRobotData(); /// Tested
  printGameData();
  if(waitForPenalty()) { /// Tested
    LOG_INFO("In waitForPenalty()...");
    // Robot is penalised so turn off vision and localization modules
    BaseModule::publishModuleRequest(boost::make_shared<SwitchVision>(false));
    BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(false));
    // Localization has to be resetted as well
    LOCALIZE_LAST_KNOWN_OUT(PlanningModule) = false;
    return;
  }

  // Check if the robot is fallen or about to fall
  if (!inFallRecovery && robotIsFalling()) { /// Tested
    LOG_INFO("In robotIsFalling()...");
    // Get ready to recover since the robot has fallen
    fallenRobotAction(); /// Tested
  }

  // Stop if a motion or static behavior request is in progress
  if (requestInProgress()) return; /// Tested
  if (shutdownCallBack()) return; /// Tested

  // Just for debugging individual behaviors bypassing this behavior
  //behaviorState = gameplaySequence;
  // If the robot is recovering from a fall
  if (inFallRecovery) { /// Tested
    LOG_INFO("In fallRecoveryAction()...");
    // Perform fall recovery action
    fallRecoveryAction(); /// Tested
  } else {
    // Robot is standing and able to perform actions
    if (behaviorState == startup) {
      LOG_INFO("BehaviorState: startup");
      startupAction(); /// Tested
    } else if (behaviorState == robocupCfg) {
      LOG_INFO("BehaviorState: robocupCfg");
      cfgHandlingAction();
    } else if (behaviorState == readySequence) {
      LOG_INFO("BehaviorState: readySequence");
      readySequenceAction();
    } else if (behaviorState == getInPosition) {
      LOG_INFO("BehaviorState: getInPosition");
      getInPositionAction();
    }else if (behaviorState == setSequence) {
      LOG_INFO("BehaviorState: setSequence");
      setSequenceAction();
    } else if (behaviorState == gameplaySequence) {
      LOG_INFO("BehaviorState: gameplaySequence");
      gameplaySequenceAction();
    }
  }
}

void RobocupSetup::finish()
{
  LOG_INFO("RobocupSetup::finish() called...")
  inBehavior = false;
}

void RobocupSetup::startupAction()
{
  if (setPostureAndStiffness(PostureState::stand, StiffnessState::robocup, MOTION_1))
    behaviorState = robocupCfg;
}

void RobocupSetup::cfgHandlingAction()
{
  auto& gameData = GAME_DATA_OUT(PlanningModule);
  ///Example states for checking penalty shootout behavior
  ///gameData.state = STATE_SET;
  ///gameData.secondaryState = STATE2_PENALTYSHOOT;
  ///gameData.kickOffTeam = 30;
  ///Example states for checking gameplay behavior
  //gameData.state = STATE_SET;
  //gameData.secondaryState = STATE2_NORMAL;
  //gameData.kickOffTeam = 30;
  if ((unsigned) gameData.state == STATE_INITIAL) {
    LOG_INFO("GameCtrlState: STATE_INITIAL")
  } else if ((unsigned) gameData.state == STATE_READY) {
    LOG_INFO("GameCtrlState: STATE_READY")
    ON_SIDE_LINE_OUT(PlanningModule) = true;
    BaseModule::publishModuleRequest(boost::make_shared<SwitchVision>(true));
    BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(true));
    BaseModule::publishModuleRequest(boost::make_shared<ResetLocalizer>());
    behaviorState = readySequence;
  } else if ((unsigned) gameData.state == STATE_SET) {
    LOG_INFO("GameCtrlState: STATE_SET")
    auto secState = (unsigned) gameData.secondaryState;
    auto kickOffTeam = (unsigned) gameData.kickOffTeam;
    const auto& ourTeamNumber = TEAM_NUMBER_IN(PlanningModule);
    if (secState == STATE2_PENALTYSHOOT) {
      LOG_INFO("GameCtrlState: STATE2_PENALTYSHOOT")
      if (kickOffTeam == ourTeamNumber) {
        LOG_INFO("GameState: PENALTY_STRIKER")
         // auto planConfig =
         //   boost::make_shared < PBPenaltiesConfig > (PBPenaltiesTypes::PENALTY_STRIKER);
         // setupChildBehaviorRequest(planConfig);
      } else {
        LOG_INFO("GameState: PENALTY_GOALKEEPER")
         //auto planConfig =
         //   boost::make_shared < PBPenaltiesConfig > (PBPenaltiesTypes::PENALTY_GOALKEEPER);
         // setupChildBehaviorRequest(planConfig);
      }
    } else if (secState == STATE2_NORMAL) {
      behaviorState = setSequence;
    }
  }
}

void RobocupSetup::readySequenceAction()
{
  if (!ROBOT_LOCALIZED_IN(PlanningModule)) {
    LOG_INFO("Trying to localize...")
    if (!mbInProgress()) {
      LOG_INFO("Setting HeadTargetSearch motion config to find HeadTargetTypes::goal...")
      auto mConfig =
        boost::make_shared <HeadTargetTrackConfig> ();
      mConfig->headTargetType = HeadTargetTypes::goal;
      ///< Robot is on sidelines with other robots so keep scan range minimum.
      mConfig->scanConfig->scanMaxYaw = 75.0 * M_PI / 180;
      setupMBRequest(MOTION_1, mConfig);
    }
  } else {
    LOG_INFO("Robot localized...")
    bool teamRobotOverlap = false;
    const auto& pose = ROBOT_POSE_2D_IN(PlanningModule);
    unsigned teamRobotsLost = 0;
    LOG_INFO("RobotPosition: " << pose.get());
    for (const auto& tr : TEAM_ROBOTS_IN(PlanningModule)) {
      if (tr.id == (int) PLAYER_NUMBER_IN(PlanningModule)) {
        ///< This robot itself
        continue;
      }
      ///< Wait for team to get localized
      if (tr.positionConfidence < 50) {
        ++teamRobotsLost;
        continue;
      }
      auto dist = (pose.get() - tr.pose.get()).norm();
      ///< Check if another team member has overlapping position
      if (dist < 0.2) {
        LOG_INFO("Position overlapping with team member. Relocalizing...")
        resetLocalizer();
        teamRobotOverlap = true;
      }
    }

    if (!teamRobotOverlap && teamRobotsLost >= 1) {
      auto& gameData = GAME_DATA_OUT(PlanningModule);
      auto kickOffTeam = (unsigned) gameData.kickOffTeam;
      const auto& ourTeamNumber = TEAM_NUMBER_IN(PlanningModule);
      int robocupRole = (int) RobocupRole::goalKeeper;
      auto smallestDist = 1e3;
      if (kickOffTeam == ourTeamNumber) {
        for (size_t i = (int) RobocupRole::goalKeeper;
          i < (int) RobocupRole::count; ++i) {
          auto dist = (pose.get() - sidePositionsAtt[i].get()).norm();
          if (dist < smallestDist) {
            smallestDist = dist;
            robocupRole = i;
          }
        }
      } else {
        for (size_t i = (int) RobocupRole::goalKeeper;
          i < (int) RobocupRole::count; ++i) {
          auto dist = (pose.get() - sidePositionsDef[i].get()).norm();
          if (dist < smallestDist) {
            smallestDist = dist;
            robocupRole = i;
          }
        }
      }
      // Chnage this varaible to unsigned or RobocupRole
      ROBOCUP_ROLE_OUT(PlanningModule) = robocupRole;
      LOG_INFO("Robocup role: " << robocupRole);
      setRobotIntention();
      ON_SIDE_LINE_OUT(PlanningModule) = false;
      LOCALIZE_LAST_KNOWN_OUT(PlanningModule) = true;
      this->killMotionBehavior(MOTION_1);
      behaviorState = getInPosition;
    }
  }
}

void RobocupSetup::getInPositionAction()
{
  auto& gameData = GAME_DATA_OUT(PlanningModule);
  // If the ready sequence time is up and game controller sent set
  // command
  if ((unsigned) gameData.state == STATE_SET) {
    killMotionBehavior(MOTION_1);
    behaviorState = setSequence;
  } else {
    // Else keep trying to get in correct position
    if (!ROBOT_LOCALIZED_IN(PlanningModule)) {
      cout << "Robot not localized. Trying to scan and get localized...\n";
      LOCALIZE_LAST_KNOWN_OUT(PlanningModule) = true;
      if (!mbInProgress()) {
        auto mConfig =
          boost::make_shared <HeadTargetTrackConfig> ();
        mConfig->headTargetType = HeadTargetTypes::goal;
        // If robot is on Sidelines alone. Better to find T or L corners too.
        if (ON_SIDE_LINE_OUT(PlanningModule))
          mConfig->scanConfig->scanMaxYaw = 75.0 * MathsUtils::DEG_TO_RAD;
        else // If robot is in the field.
          mConfig->scanConfig->scanMaxYaw = 75.0 * MathsUtils::DEG_TO_RAD;
        setupMBRequest(MOTION_1, mConfig);
      }
    } else {
      // After the walk finishes reset the localizer to find its
      // position estimate from landmarks
      if (!mbInProgress()) {
        RobotPose2D<float> target;
        if (setSequenceFinished) {
          const auto& pose = ROBOT_POSE_2D_IN(PlanningModule);
          const auto& role = ROBOCUP_ROLE_OUT(PlanningModule);
          if (role == (int)RobocupRole::goalKeeper) {
            target = positionsInGame[0];
          } else if (role == (int)RobocupRole::defender || role == (int)RobocupRole::defenseSupport) {
            const auto& pose = ROBOT_POSE_2D_IN(PlanningModule);
            auto smallestDist = 1e3;
            for (int i = 1; i < 3; ++i) {
              auto dist = (pose.get() - positionsInGame[i].get()).norm();
              if (dist < smallestDist) {
                smallestDist = dist;
                target = positionsInGame[i];
              }
            }
          } else if (role == (int)RobocupRole::offenseSupport || role == (int)RobocupRole::attacker) {
            auto smallestDist = 1e3;
            for (int i = 3; i < 5; ++i) {
              auto dist = (pose.get() - positionsInGame[i].get()).norm();
              if (dist < smallestDist) {
                smallestDist = dist;
                target = positionsInGame[i];
              }
            }
          }
          setPlanTowardsConfig(target, keepMovingWhileNav);
          behaviorState = goingToPosition;
          ON_SIDE_LINE_OUT(PlanningModule) = false;
        } else {
          auto kickOffTeam = (unsigned) gameData.kickOffTeam;
          const auto& ourTeamNumber = TEAM_NUMBER_IN(PlanningModule);
          if (kickOffTeam == ourTeamNumber) { // Attack team
            target = startPositionsAtt[ROBOCUP_ROLE_OUT(PlanningModule)];
          } else {
            target = startPositionsDef[ROBOCUP_ROLE_OUT(PlanningModule)];
          }
          cout << "Start position: " << target.get() << endl;
          //if (this->lastChildCfg &&
          //    this->lastChildCfg->id == (unsigned)PBIds::NAVIGATION)
          //{ // If last child running was navigation behavior
          setPlanTowardsConfig(target, keepMovingWhileNav);
          behaviorState = goingToPosition;
        }
      }
    }
  }
}

void RobocupSetup::goingToPositionAction()
{
  const auto& pose = ROBOT_POSE_2D_IN(PlanningModule);
  if ((moveTarget.get() - pose.get()).norm() < 5e-3) {
    if (setSequenceFinished)
      behaviorState = gameplaySequence;
    else
      behaviorState = setSequence;
  } else {
    if (!this->getChild()) {
      behaviorState = getInPosition;
    }
  }
}

void
RobocupSetup::setSequenceAction()
{
  cout << "Executing RobocupSetup.setSequenceAction()...\n";
  if (POSTURE_STATE_IN(PlanningModule) != PostureState::stand) {
    if (!mbInProgress()) {
      auto pConfig =
        boost::make_shared<InterpToPostureConfig>();
      pConfig->targetPosture = PostureState::stand;
      pConfig->timeToReachP = 2.f;
      setupMBRequest(MOTION_1, pConfig);
    }
    //OVAR(bool, PlanningModule::runVisionModule) = false;
    //OVAR(bool, PlanningModule::runLocalizationModule) = false;
  } else {
    if (!WHISTLE_DETECTED_IN(PlanningModule)) {
      if (!gbInProgress()) {
        auto sConfig = boost::make_shared<GBWDConfig>();
        setupGBRequest(sConfig);
      }
    } else {
      //OVAR(bool, PlanningModule::runVisionModule) = true;
      //OVAR(bool, PlanningModule::runLocalizationModule) = true;
      //LOCALIZE_LAST_KNOWN_OUT(PlanningModule) = true;
      behaviorState = gameplaySequence;
      setSequenceFinished = true;
    }
  }
}

void
RobocupSetup::gameplaySequenceAction()
{
  // Just to check individual behavior set manually.
  auto& role = ROBOCUP_ROLE_OUT(PlanningModule);
  role = (int)RobocupRole::goalKeeper;
  if (role == (int)RobocupRole::goalKeeper) {
    //auto planConfig = boost::make_shared<GoalKeeperConfig>();
    //setupChildBehaviorRequest(planConfig);
  } else if (role == (int)RobocupRole::defender) {
    //defenderAction();
  } else if (role == (int)RobocupRole::defenseSupport) {
    //defSupportAction();
  } else if (role == (int)RobocupRole::offenseSupport) {
    //offSupportAction();
  } else if (role == (int)RobocupRole::attacker) {
    //attackerAction();
  }
}
