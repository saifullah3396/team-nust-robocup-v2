#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/BehaviorInfo.h"
#include "Utils/include/DataHolders/Camera.h"
#include "Utils/include/DataHolders/GoalInfo.h"
#include "Utils/include/DataHolders/Landmark.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/DataHolders/OccupancyMap.h"
#include "Utils/include/DataHolders/PositionInput.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/TeamRobot.h"
#include "Utils/include/DataHolders/VelocityInput.h"
#include "Utils/include/DataHolders/WorldBallInfo.h"

int main()
{
  BallInfo<float> ball;
  ball.print();
  cout << ball.getJson() << endl;

  BehaviorInfo behaviorInfo;
  behaviorInfo.print();
  cout << behaviorInfo.getJson() << endl;

  WorldBallInfo<float> wBall;
  wBall.print();
  cout << wBall.getJson() << endl;

  Camera<float> cam("name");
  cam.print();
  cout << cam.getJson() << endl;

  GoalInfo<float> gi;
  gi.print();
  cout << gi.getJson() << endl;

  Landmark<float> lm(0, cv::Point_<float>(0.0, 0.0));
  lm.print();
  cout << lm.getJson() << endl;

  UnknownLandmark<float> uklm(cv::Point_<float>(0.0, 0.0));
  uklm.print();
  cout << uklm.getJson() << endl;

  KnownLandmark<float> klm (0, cv::Point_<float>(), RobotPose2D<float>());
  klm.print();
  cout << klm.getJson() << endl;

  Obstacle<float> obs(ObstacleType::goalPost);
  obs.print();
  cout << obs.getJson() << endl;

  OccupancyMap<float> om;
  om.print();
  cout << om.getJson() << endl;

  RobotPose2D<float> rb;
  rb.print();
  cout << rb.getJson() << endl;

  PositionInput<float> pi(0, 0, 0);
  pi.print();
  cout << pi.getJson() << endl;

  VelocityInput<float> vi(0, 0, 0);
  vi.print();
  cout << vi.getJson() << endl;

  TeamRobot<float> tr;
  tr.print();
  cout << tr.getJson() << endl;
  return 0;
}
