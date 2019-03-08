#include <iostream>
#include "Utils/include/HardwareIds.h"

using namespace std;

int main()
{
  cout << "Joints::count: " << toUType(Joints::count) << endl;
  for (const auto& j : Joints()) {
    cout << "Joints: " << toUType(j) << endl;
  }
  cout << "HardwareIds::headStart: " << toUType(HardwareIds::headStart) << endl;
  cout << "HardwareIds::nHead: " << toUType(HardwareIds::nHead) << endl;
  cout << "HardwareIds::lArmStart: " << toUType(HardwareIds::lArmStart) << endl;
  cout << "HardwareIds::nLArm: " << toUType(HardwareIds::nLArm) << endl;
  cout << "HardwareIds::rArmStart: " << toUType(HardwareIds::rArmStart) << endl;
  cout << "HardwareIds::nRArm: " << toUType(HardwareIds::nRArm) << endl;
  cout << "HardwareIds::lLegStart: " << toUType(HardwareIds::lLegStart) << endl;
  cout << "HardwareIds::nLLeg: " << toUType(HardwareIds::nLLeg) << endl;
  cout << "HardwareIds::rLegStart: " << toUType(HardwareIds::rLegStart) << endl;
  cout << "HardwareIds::nRLeg: " << toUType(HardwareIds::nRLeg) << endl;

  cout << "Links::count: " << toUType(Links::count) << endl;
  for (const auto& j : Links()) {
    cout << "Links: " << toUType(j) << endl;
  }

  cout << "TouchSensors::count: " << toUType(TouchSensors::count) << endl;
  for (const auto& j : TouchSensors()) {
    cout << "TouchSensors: " << toUType(j) << endl;
  }

  cout << "SwitchSensors::count: " << toUType(SwitchSensors::count) << endl;
  for (const auto& j : SwitchSensors()) {
    cout << "SwitchSensors: " << toUType(j) << endl;
  }

  cout << "BatterySensors::count: " << toUType(BatterySensors::count) << endl;
  for (const auto& j : BatterySensors()) {
    cout << "BatterySensors: " << toUType(j) << endl;
  }

  cout << "InertialSensors::count: " << toUType(InertialSensors::count) << endl;
  for (const auto& j : InertialSensors()) {
    cout << "InertialSensors: " << toUType(j) << endl;
  }

  cout << "FsrSensors::count: " << toUType(FsrSensors::count) << endl;
  for (const auto& j : FsrSensors()) {
    cout << "FsrSensors: " << toUType(j) << endl;
  }

  cout << "SonarSensors::count: " << toUType(SonarSensors::count) << endl;
  for (const auto& j : SonarSensors()) {
    cout << "SonarSensors: " << toUType(j) << endl;
  }

  cout << "InertialSensors::count: " << toUType(InertialSensors::count) << endl;
  for (const auto& j : InertialSensors()) {
    cout << "InertialSensors: " << toUType(j) << endl;
  }

  return 0;
}
