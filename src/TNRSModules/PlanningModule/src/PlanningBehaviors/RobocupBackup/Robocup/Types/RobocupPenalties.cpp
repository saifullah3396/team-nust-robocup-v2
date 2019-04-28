/**
 * @file PlanningBehaviors/Robocup/Types/RobocupPenalties.cpp
 *
 * This file declares the class RobocupPenalties.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/GBConfigs/GBStiffnessConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBRobocupConfig.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/RobocupPenalties.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/DataHolders/SPLStandardMessage.h"

RobocupPenalties::RobocupPenalties(
  PlanningModule* planningModule,
  const boost::shared_ptr<PenaltiesConfig>& config) :
  Robocup(planningModule, config, "RobocupPenalties")
{
}

boost::shared_ptr<PenaltiesConfig> RobocupPenalties::getBehaviorCast()
{
  return boost::static_pointer_cast<PenaltiesConfig>(config);
}

bool RobocupPenalties::initiate()
{
  /*LOG_INFO("RobocupPenalties.initiate()")
  auto gameData = GAME_DATA_OUT(PlanningModule);
  gameData.state = STATE_PLAYING;
  if ((unsigned) gameData.state == STATE_PLAYING) {
    LOG_INFO("GameCtrlState... STATE_PLAYING")
    auto secState = (unsigned) gameData.secondaryState;
    auto kickOffTeam = (unsigned) gameData.kickOffTeam;
    auto& ourTeamNumber = IVAR(int, PlanningModule::teamNumber);
    if (secState == STATE2_PENALTYSHOOT) {
      if (kickOffTeam == ourTeamNumber) {
        striker = true; // striker
      } else {
        striker = false; // goalkeeper
      }
    }
  }*/
  return true;
}

void RobocupPenalties::update()
{
  LOG_INFO("RobocupPenalties.update()")
}

void RobocupPenalties::finish()
{
  inBehavior = false;
}

void RobocupPenalties::penaltyCfgAction()
{
  /*LOG_INFO("Setting Penalty Configuration...")
  auto gameData = IVAR(RoboCupGameControlData, PlanningModule::gameData);
  gameData.state = STATE_PLAYING;
  if ((unsigned) gameData.state == STATE_PLAYING) {
    LOG_INFO("GameCtrlState... STATE_PLAYING")
    auto secState = (unsigned) gameData.secondaryState;
    auto kickOffTeam = (unsigned) gameData.kickOffTeam;
    auto& ourTeamNumber = IVAR(int, PlanningModule::teamNumber);
    if (secState == STATE2_PENALTYSHOOT) {
      if (kickOffTeam == ourTeamNumber) {
        striker = true; // striker
      } else {
        striker = false; // goalkeeper
      }
    }
  }*/
}
