#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "../include/SharedMemory.h"
#include "Utils/include/ConfigManager.h"

using namespace std;

int
main()
{
  ConfigManager::setDirPaths("Robots/Sim/");
  SharedMemoryPtr sharedMemory;
  cout << "Initiating memory...1" << endl;
  sharedMemory = boost::make_shared<SharedMemory>();
  cout << "Initiating memory...2" << endl;
  sharedMemory->init();
  cout << "Memory initiated successfully." << endl;
  cout << sharedMemory->getJson() << endl;
  return 1;
}
