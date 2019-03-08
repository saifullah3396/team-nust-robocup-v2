#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "MotionModule/include/KinematicsModule/JointModelCalibrator.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/ConfigManager.h"
#include "Utils/include/DataUtils.h"

using namespace std;
using namespace Eigen;

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
  JointModelCalibrator calibrator(jointIndex, path);
  calibrator.optDef();
  calibrator.printResults();
  while(true);
  return 0;
}
