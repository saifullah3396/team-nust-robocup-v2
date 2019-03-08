/**
 * @file TeamComm/TeamComm.h
 *
 * This file implements the class TeamComm
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Jan 2017
 */

#include "TNRSBase/include/MemoryIOMacros.h"
#include "GameCommModule/include/GameCommModule.h"
#include "GameCommModule/include/TeamComm.h"
#include "MotionModule/include/MotionBehaviorIds.h"
#include "Utils/include/DataHolders/TeamRobot.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/PrintUtils.h"

TeamComm::TeamComm(
  GameCommModule* gameCommModule,
  const unsigned short& port,
  const char* subnet) :
  MemoryBase(gameCommModule),
  gameCommModule(gameCommModule),
  port(port)
{
  LOG_INFO("Setting up UDP connection on port: " << port)
  LOG_INFO("Setting UDP target subnet: " << subnet)
  udpComm.setBlocking(false);
  udpComm.setBroadcast(true);
  udpComm.bind("0.0.0.0", port);
  udpComm.setTarget(subnet, port);
  udpComm.setLoopback(false);
}


void TeamComm::send()
{
  if (!port) return;
  SPLStandardMessage outMsg;
  updateMessage(outMsg);
  /*cout << "x: " << ROBOT_POSE_2D.getX() << endl;
   cout << "y: " <<  ROBOT_POSE_2D.getY() << endl;
   cout << "theta: " <<  ROBOT_POSE_2D.getTheta() << endl;
   cout << "outMsg: " << endl;
   cout << "pose1: " << outMsg.pose[0] << endl;
   cout << "pose2: " << outMsg.pose[1] << endl;
   cout << "pose3: " << outMsg.pose[2] << endl;*/
  udpComm.write(
    (const char*) &outMsg,
    offsetof(SPLStandardMessage, data) + outMsg.numOfDataBytes);
}

unsigned TeamComm::receive()
{
  if (!port) return 0;
  SPLStandardMessage inMsg;
  int size;
  unsigned receivedSize = 0;
  do {
    size = udpComm.read((char*) &inMsg, sizeof(SPLStandardMessage));
    if (size >= static_cast<int>(offsetof(SPLStandardMessage, data)) &&
        size <= static_cast<int>(sizeof(SPLStandardMessage))) {
      receivedSize = static_cast<unsigned>(size);
      receiveQueue.pushToQueue(inMsg);
    }
  } while (size > 0);
  return receivedSize;
}

void TeamComm::updateTeamData(const SPLStandardMessage& msg)
{
  if (int(msg.playerNum) == PLAYER_NUMBER_IN(GameCommModule)) return;
  TeamRobot<float> teamRobot;
  teamRobot.id = (int) msg.playerNum;
  teamRobot.fallen = msg.fallen;
  teamRobot.ballPos = cv::Point_<float>(msg.ball[0] / 1e3, msg.ball[1] / 1e3);
  teamRobot.ballVel = cv::Point_<float>(msg.ballVel[0] / 1e3, msg.ballVel[1] / 1e3);
  teamRobot.pose = RobotPose2D<float>(msg.pose[0], msg.pose[1], msg.pose[2]);
  teamRobot.walkingTo = cv::Point_<float>(msg.walkingTo[0], msg.walkingTo[1]);
  teamRobot.shootingTo = cv::Point_<float>(msg.shootingTo[0], msg.shootingTo[1]);
  teamRobot.intention = msg.intention;
  teamRobot.suggestionToMe =
    msg.suggestion[PLAYER_NUMBER_IN(GameCommModule) - 1];
  teamRobot.dataReceived = true;
  teamRobot.positionConfidence = msg.currentPositionConfidence;
  teamRobot.sideConfidence = msg.currentSideConfidence;
  TEAM_ROBOTS_OUT(GameCommModule)[(int) msg.playerNum - 1] = teamRobot;
}

void TeamComm::processReceivedMsgs()
{
  for (auto& tr : TEAM_ROBOTS_OUT(GameCommModule))
    tr.dataReceived = false;
  while (!receiveQueue.isEmpty()) {
    updateTeamData(receiveQueue.queueFront());
    receiveQueue.popQueue();
  }
}

void TeamComm::updateMessage(SPLStandardMessage& msg)
{
  msg.playerNum = PLAYER_NUMBER_IN(GameCommModule);
  msg.teamNum = TEAM_NUMBER_IN(GameCommModule);
  msg.fallen = ROBOT_FALLEN_IN(GameCommModule);
  RobotPose2D<float> rState = ROBOT_POSE_2D_IN(GameCommModule);
  msg.pose[0] = rState.getX() * 1e3;
  msg.pose[1] = rState.getY() * 1e3;
  msg.pose[2] = rState.getTheta();
  auto targetState = MOVE_TARGET_IN(GameCommModule);
  msg.walkingTo[0] = targetState.getX() * 1e3;
  msg.walkingTo[1] = targetState.getY() * 1e3;
  auto target = KICK_TARGET_IN(GameCommModule);
  msg.shootingTo[0] = target.x * 1e3;
  msg.shootingTo[1] = target.y * 1e3;
  if (BALL_INFO_IN(GameCommModule).found) {
    msg.ballAge = BALL_INFO_IN(GameCommModule).ballAge;
  } else {
    msg.ballAge = -1;
  }
  msg.ball[0] = BALL_INFO_IN(GameCommModule).posRel.x* 1e3;
  msg.ball[1] = BALL_INFO_IN(GameCommModule).posRel.y * 1e3;
  msg.ballVel[0] = BALL_INFO_IN(GameCommModule).velRel.x * 1e3;
  msg.ballVel[1] = BALL_INFO_IN(GameCommModule).velRel.y * 1e3;
  for (size_t i = 0; i < SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i) {
    msg.suggestion[i] = -1;
  }
  msg.intention = ROBOT_INTENTION_IN(GameCommModule);
  msg.averageWalkSpeed = 120.0;
  msg.maxKickDistance = 6e3;
  msg.currentPositionConfidence = POSITION_CONFIDENCE_IN(GameCommModule);
  msg.currentSideConfidence = SIDE_CONFIDENCE_IN(GameCommModule);
  msg.numOfDataBytes = 0;
}
