#include <iostream>
#include "BehaviorConfigs/include/PBConfigs/PBExternalInterfaceConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBKickSequenceConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBNavigationConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBRobocupConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBStartupConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBConfig.h"
#include "BehaviorConfigs/include/PBConfigs/TestSuiteConfig.h"

int main()
{
  PBRobocupConfigPtr config
    = boost::make_shared<PBRobocupConfig>();
  cout << "config: " << config->getJson() << endl;
  RobocupSetupConfigPtr config2
    = boost::make_shared<RobocupSetupConfig>();
  cout << "config2: " << config2->getJson() << endl;
  AttackerConfigPtr config3
    = boost::make_shared<AttackerConfig>();
  cout << "config3: " << config3->getJson() << endl;
  DefenderConfigPtr config4
    = boost::make_shared<DefenderConfig>();
  cout << "config4: " << config4->getJson() << endl;
  GoalKeeperConfigPtr config5
    = boost::make_shared<GoalKeeperConfig>();
  cout << "config5: " << config5->getJson() << endl;
  //PBExternalInterfaceConfigPtr config1
  //  = boost::make_shared<PBExternalInterfaceConfig>();
  //PBKickSequenceConfigPtr config2
  //  = boost::make_shared<PBKickSequenceConfig>();
  //PBNavigationConfigPtr config3
  //  = boost::make_shared<PBNavigationConfig>();
  //PBRobocupConfigPtr config4
  //  = boost::make_shared<PBRobocupConfig>();
  //PBStartupConfigPtr config5
  //  = boost::make_shared<PBStartupConfig>();
  //TestSuiteConfigPtr config6
  //  = boost::make_shared<TestSuiteConfig>();
}
