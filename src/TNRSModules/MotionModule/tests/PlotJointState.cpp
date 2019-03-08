#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "Utils/include/HardwareIds.h"
#include "Utils/include/ConfigManager.h"
#include "Utils/include/Constants.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/PlotEnv.h"
#include "Utils/include/JsonUtils.h"

using namespace std;
using namespace Eigen;
using namespace GnuPlotEnv;

int main(int argc, char **argv)
{
  ConfigManager::setDirPaths("Robots/Sim/");
  ConfigManager::createDirs();
  string path = string("/home/sensei/team-nust-robocup/logs/Robots/Sim/InterpToPosture/log5/InterpToPosture.json");
  if (argc > 1)
    path = argv[1];
  auto jointIndex = Joints::headYaw;
  if (argc > 2) {
    int index;
    DataUtils::stringToVar(argv[2], index);
    jointIndex = static_cast<Joints>(index);
  }
  using namespace std;
  Json::Value json;
  ifstream data(path, ifstream::binary);
  data >> json;
  auto state = json[Constants::jointNames[toUType(jointIndex)]]["state"];

  Matrix<double, Dynamic, 1> sensedPosition(state.size());
  Matrix<double, Dynamic, 1> cmdPosition(state.size());
  Matrix<double, Dynamic, 1> estPosition(state.size());
  Matrix<double, Dynamic, 1> estVelocity(state.size());
  Matrix<double, Dynamic, 1> estAcceleration(state.size());
  Matrix<double, Dynamic, 1> time(state.size());
  for (size_t i = 0; i < state.size(); ++i) {
    cmdPosition[i] = state[static_cast<int>(i)]["cmd"].asFloat();
    estPosition[i] = state[static_cast<int>(i)]["position"].asFloat();
    estVelocity[i] = state[static_cast<int>(i)]["velocity"].asFloat();
    estAcceleration[i] = state[static_cast<int>(i)]["accel"].asFloat();
    sensedPosition[i] = state[static_cast<int>(i)]["sensed"].asFloat();
    time[i] = state[static_cast<int>(i)]["time"].asFloat();
  }

  Vector2d range;
  range[0] = -3.14;
  range[1] = 3.14;
  PlotEnv<double>::set_terminal_std("qt");
  PlotEnv<double> pe = PlotEnv<double>(
    "JointState",
    "Time (seconds)",
    "State",
    "z-Axis",
    range,
    range,
    range);
  pe.plot2D("Commanded Position", time, cmdPosition);
  pe.plot2D("Sensed Position", time, sensedPosition);
  pe.plot2D("Estimated Position", time, estPosition);
  //pe.plot2D("Estimated Velocity", time, estVelocity);
  //pe.plot2D("Estimated Acceleration", time, estAcceleration);
  pe.showonscreen();
  while(true);
  return 0;
}
