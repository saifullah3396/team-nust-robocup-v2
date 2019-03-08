/**
 * @file GameCommModule/GameCommModule.h
 *
 * This file implements the class GameCommModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "GameCommModule/include/GameCommModule.h"
#include "GameCommModule/include/TeamComm.h"
#include "TNRSBase/include/MemoryConnector.h"
#include "TNRSBase/include/SharedMemory.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/TeamRobot.h"
#include "Utils/include/ThreadSafeQueue.h"
#include "Utils/include/DebugUtils.h"
#include <opencv2/opencv.hpp>

DEFINE_INPUT_CONNECTOR(GameCommModule,
  (int, gameCommThreadPeriod),
  (int, playerNumber),
  (int, teamNumber),
  (int, teamPort),
  (bool, robotFallen),
  (RobotPose2D<float>, robotPose2D),
  (RobotPose2D<float>, moveTarget),
  (cv::Point_<float>, kickTarget),
  (BallInfo<float>, ballInfo),
  (int, robotIntention),
  (bool, robotLocalized),
  (int, positionConfidence),
  (int, sideConfidence),
)

DEFINE_OUTPUT_CONNECTOR(GameCommModule,
  (vector<TeamRobot<float> >, teamRobots),
)

GameCommModule::GameCommModule(void* teamNUSTSPL) :
  BaseModule(
    teamNUSTSPL,
    static_cast<unsigned>(TNSPLModules::gameComm),
    "GameCommModule"
  )
{
}

void GameCommModule::setThreadPeriod()
{
  setPeriodMinMS(GAME_COMM_PERIOD_IN(GameCommModule));
}

void GameCommModule::initMemoryConn()
{
  inputConnector =
    new InputConnector(this, getModuleName() + "InputConnector");
  outputConnector =
    new OutputConnector(this, getModuleName() + "OutputConnector");
  inputConnector->initConnector();
  outputConnector->initConnector();
}

void GameCommModule::init()
{
  LOG_INFO("Initializing GameCommModule...")
  string bcastAddr = UdpComm::getWifiBroadcastAddress();
  LOG_INFO("Setting up TeamCommunication...")
  teamComm =
    boost::shared_ptr<TeamComm>(
      new TeamComm(this, TEAM_PORT_IN(GameCommModule), bcastAddr.c_str())
    );
}

void GameCommModule::handleRequests()
{
}

void GameCommModule::mainRoutine()
{
  teamComm->send();
  teamComm->receive();
  teamComm->processReceivedMsgs();
}
