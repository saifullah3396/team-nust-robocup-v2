#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "Utils/include/Solvers/ImuCalibrator.h"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
  string path = string("/home/sensei/team-nust-robocup/logs/Robots/Sim/KinematicsModule/ImuData.json");
  if (argc > 1)
    path = argv[1];
  ImuCalibrator<3> calibrator(path);
  calibrator.optDef();
  calibrator.printResults();
  return 0;
}
