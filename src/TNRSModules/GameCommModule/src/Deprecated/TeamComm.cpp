/**
 * @file TeamComm/TeamComm.h
 *
 * This file implements the class TeamComm
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Jan 2017
 */

#include "CommModule/include/TeamComm.h"
#include "MotionModule/include/MotionBehaviorIds.h"

#define TEAM_ROBOTS OVAR(vector<TeamRobot<float> >, CommModule::teamRobots)
#define BALL_INFO IVAR(BallInfo<float>, CommModule::ballInfo)

void TeamComm::startUp(const int& port, const char* subnet)
{
  this->port = port;
  LOG_INFO("port: " << port)
  LOG_INFO("subnet: " << subnet)
  udpComm.setBlocking(false);
  udpComm.setBroadcast(true);
  udpComm.bind("0.0.0.0", port);
  udpComm.setTarget(subnet, port);
  udpComm.setLoopback(false);
}

void
TeamComm::send()
{
  if (!port) return;
  SPLStandardMessage outMsg;
  updateMessage(outMsg);
  /*cout << "x: " << IVAR(RobotPose2D<float>, CommModule::robotPose2D).x << endl;
   cout << "y: " <<  IVAR(RobotPose2D<float>, CommModule::robotPose2D).y << endl;
   cout << "theta: " <<  IVAR(RobotPose2D<float>, CommModule::robotPose2D).theta << endl;
   cout << "outMsg: " << endl;
   cout << "pose1: " << outMsg.pose[0] << endl;
   cout << "pose2: " << outMsg.pose[1] << endl;
   cout << "pose3: " << outMsg.pose[2] << endl;*/
  udpComm.write(
    (const char*) &outMsg,
    offsetof(SPLStandardMessage, data) + outMsg.numOfDataBytes);
}

unsigned
TeamComm::receive()
{
  if (!port) return 0;
  SPLStandardMessage inMsg;
  int size;
  unsigned remoteIp = 0;
  unsigned receivedSize = 0;
  do {
    size = udpComm.read((char*) &inMsg, sizeof(SPLStandardMessage));
    if (size >= static_cast<int>(offsetof(SPLStandardMessage, data)) && size <= static_cast<int>(sizeof(SPLStandardMessage))) {
      receivedSize = static_cast<unsigned>(size);
      receiveQueue.pushToQueue(inMsg);
    }
  } while (size > 0);
  return receivedSize;
}

void TeamComm::updateTeamData(const SPLStandardMessage& msg)
{
  if (int(msg.playerNum) == IVAR(int, CommModule::playerNumber)) return;
  TeamRobot<float> teamRobot;
  teamRobot.fallen = msg.fallen;
  teamRobot.ballPos = TNRSPoint2<float>(msg.ball[0] / 1e3, msg.ball[1] / 1e3);
  teamRobot.ballVel = TNRSPoint2<float>(msg.ballVel[0] / 1e3, msg.ballVel[1] / 1e3);
  teamRobot.pose = RobotPose2D<float>(msg.pose[0], msg.pose[1], msg.pose[2]);
  teamRobot.walkingTo = TNRSPoint2<float>(msg.walkingTo[0], msg.walkingTo[1]);
  teamRobot.shootingTo = TNRSPoint2<float>(msg.shootingTo[0], msg.shootingTo[1]);
  teamRobot.intention = msg.intention;
  teamRobot.suggestionToMe =
    msg.suggestion[IVAR(int, CommModule::playerNumber) - 1];
  teamRobot.dataReceived = true;
  teamRobot.positionConfidence = msg.currentPositionConfidence;
  teamRobot.sideConfidence = msg.currentSideConfidence;
  //teamRobot.print();
  //cout << "Player number: " << (int)msg.playerNum << endl;
  TEAM_ROBOTS[(int) msg.playerNum - 1] =
    teamRobot;
}

void
TeamComm::processReceivedMsgs()
{
  for (auto& tr : TEAM_ROBOTS)
    tr.dataReceived = false;
  while (!receiveQueue.isEmpty()) {
    updateTeamData(receiveQueue.queueFront());
    receiveQueue.popQueue();
  }
}

void
TeamComm::updateMessage(SPLStandardMessage& msg)
{
  msg.playerNum = IVAR(int, CommModule::playerNumber);
  msg.teamNum = IVAR(int, CommModule::teamNumber);
  msg.fallen = IVAR(bool, CommModule::robotFallen);
  RobotPose2D<float> rState = IVAR(RobotPose2D<float>, CommModule::robotPose2D);
  msg.pose[0] = rState.getX() * 1e3;
  msg.pose[1] = rState.getY() * 1e3;
  msg.pose[2] = rState.getTheta();
  /*auto lastMB = IVAR(BehaviorAccepted, CommModule::lastMBehaviorAccepted);
  if (lastMB.state->id == (unsigned) MBIds::MOVEMENT && lastMB.state->started && !lastMB.state->finished) {
    auto targetState = IVAR(RobotPose2D<float>, CommModule::moveTarget);
    msg.walkingTo[0] = targetState.x * 1e3;
    msg.walkingTo[1] = targetState.y * 1e3;
  } else {
    msg.walkingTo[0] = rState.x * 1e3;
    msg.walkingTo[1] = rState.y * 1e3;
  }

  if (lastMB.state->id == (unsigned) MBIds::KICK && lastMB.state->started && !lastMB.state->finished) {
    auto target = IVAR(Vector2f, CommModule::kickTarget);
    msg.shootingTo[0] = target[0] * 1e3;
    msg.shootingTo[1] = target[1] * 1e3;
  } else {
    msg.shootingTo[0] = rState.x * 1e3;
    msg.shootingTo[1] = rState.y * 1e3;
  }*/
  //msg.ballAge TO BE FIXED
  if (BALL_INFO.found) {
    msg.ballAge = BALL_INFO.ballAge;
  } else {
    msg.ballAge = -1;
  }
  msg.ball[0] = BALL_INFO.posRel.x() * 1e3;
  msg.ball[1] = BALL_INFO.posRel.y() * 1e3;
  msg.ballVel[0] = BALL_INFO.velRel.x() * 1e3;
  msg.ballVel[1] = BALL_INFO.velRel.y() * 1e3;
  for (size_t i = 0; i < SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i) {
    msg.suggestion[i] = -1;
  }
  msg.intention = IVAR(int, CommModule::robotIntention);
  msg.averageWalkSpeed = 120.0;
  msg.maxKickDistance = 6e3;
  msg.currentPositionConfidence = IVAR(int, CommModule::positionConfidence);
  msg.currentSideConfidence = IVAR(int, CommModule::sideConfidence);
  msg.numOfDataBytes = 0;
}
