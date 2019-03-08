#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include "ControlModule/include/HardwareLayer.h"

typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
typedef boost::shared_ptr<AL::DCMProxy> ALDCMProxyPtr;

int
main(int argc, char* argv[])
{
  if (argc != 2) {
    std::cerr << "Wrong number of arguments!" << std::endl;
    std::cerr << "Usage: say NAO_IP" << std::endl;
    exit(2);
  }
  ConfigManager::setDirPaths("Robots/Sim/");
  ALMemoryProxyPtr memoryProxy;
  ALDCMProxyPtr dcmProxy;
  JointSensorsLayer js1(memoryProxy, JointSensorTypes::position);
  JointSensorsLayer js2(memoryProxy, JointSensorTypes::hardness);
  JointSensorsLayer js3(memoryProxy, JointSensorTypes::current);
  JointSensorsLayer js4(memoryProxy, JointSensorTypes::temp);
  TouchSensorsLayer ts(memoryProxy);
  SwitchSensorsLayer ss(memoryProxy);
  FsrSensorsLayer fs(memoryProxy);
  InertialSensorsLayer is(memoryProxy);
  BatterySensorsLayer bs(memoryProxy);
  SonarSensorsLayer sos(memoryProxy);
  LedSensorsLayer ls(memoryProxy);

  JointActuatorsLayer jpa(dcmProxy, JointActuatorTypes::angles);
  JointActuatorsLayer jsa(dcmProxy, JointActuatorTypes::hardness);
  LedActuatorsLayer la(dcmProxy);
  return 1;
}
