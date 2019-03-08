/**
 * @file PlanningModule/src/PBConfigs/PBConfig.cpp
 *
 * This file implements the struct PBConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "Utils/include/Behaviors/PBConfigs/PBExternalInterfaceConfig.h"
#include "Utils/include/Behaviors/PBConfigs/PBKickSequenceConfig.h"
#include "Utils/include/Behaviors/PBConfigs/PBNavigationConfig.h"
#include "Utils/include/Behaviors/PBConfigs/PBRobocupConfig.h"
#include "Utils/include/Behaviors/PBConfigs/PBStartupConfig.h"
#include "Utils/include/Behaviors/PBConfigs/PBConfig.h"
#include "Utils/include/Behaviors/PBConfigs/TestSuiteConfig.h"

boost::shared_ptr<PBConfig>
PBConfig::makeFromJson(const Json::Value& obj)
{
  //LOG_INFO("PBConfig::makeFromJson() called...")
  boost::shared_ptr<PBConfig> config;
  try {
    if (!obj.isNull()) {
      switch (obj["id"].asUInt()) {
        case toUType(PBIds::externalInterface):
          config = PBExternalInterfaceConfig::makeFromJson(obj); break;
        case toUType(PBIds::kickSequence):
          config = PBKickSequenceConfig::makeFromJson(obj); break;
        case toUType(PBIds::navigation):
          config = PBNavigationConfig::makeFromJson(obj); break;
        case toUType(PBIds::robocup):
          config = PBRobocupConfig::makeFromJson(obj); break;
        case toUType(PBIds::startup):
          config = PBStartupConfig::makeFromJson(obj); break;
        case toUType(PBIds::testSuite):
          config = TestSuiteConfig::makeFromJson(obj); break;
      }
    }
  } catch (Json::Exception& e) {
    cout
      << "Exception caught making a PBConfig from json object;\t";
    LOG_EXCEPTION(e.what());
    config.reset();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    config.reset();
  }
  return config;
}
