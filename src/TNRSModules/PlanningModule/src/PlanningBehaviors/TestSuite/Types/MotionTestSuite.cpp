/**
 * @file PlanningBehaviors/TestSuite/Types/MotionTestSuite.cpp
 *
 * This file implements the class MotionTestSuite
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include <fstream>
#include "LocalizationModule/include/LocalizationRequest.h"
#include "MotionModule/include/MotionRequest.h"
#include "PlanningModule/include/PlanningRequest.h"
#include "PlanningModule/include/PlanningBehaviors/TestSuite/Types/MotionTestSuite.h"
#include "TNRSBase/include/DebugBase.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"
#include "BehaviorConfigs/include/PBConfigs/TestSuiteConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBNavigationConfig.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/JsonUtils.h"
#include "VisionModule/include/VisionRequest.h"

MotionTestSuite::MotionTestSuite(
  PlanningModule* planningModule,
  const boost::shared_ptr<MotionTestSuiteConfig>& config) :
  TestSuite(planningModule, config, "MotionTestSuite")
{
}

bool MotionTestSuite::initiate()
{
  LOG_INFO("MotionTestSuite.initiate() called...");
  try {
    using namespace std;
    string jsonConfigPath;
    jsonConfigPath =
      ConfigManager::getMBConfigsPath() + getBehaviorCast()->requestedBehavior;
    Json::Value json;
    ifstream config(jsonConfigPath, ifstream::binary);
    config >> json;
    if (!mbInProgress()) {
      using namespace std;
      motionConfig =
        boost::static_pointer_cast<MBConfig>(BehaviorConfig::makeFromJson(json));
      if (motionConfig) {
        setupMBRequest(MOTION_1, boost::static_pointer_cast<MBConfig>(motionConfig));
        return true;
      } else {
        return false;
      }
    }
  } catch (Json::Exception& e) {
    LOG_EXCEPTION(e.what());
  } catch (BehaviorException& e) {
    LOG_EXCEPTION(e.what());
  }
}

void MotionTestSuite::update()
{
  if (requestInProgress()) return;
  if (shutdownCallBack()) return;
  if (!mbInProgress()) {
    ///< Repeat on finish based on chest button press
    #ifndef V6_CROSS_BUILD
    auto& switchSensors = SWITCH_SENSORS_OUT(PlanningModule);
    #else
    const auto& switchSensors = SWITCH_SENSORS_IN(PlanningModule);
    #endif
    if (switchSensors[toUType(SwitchSensors::chestBoardButton)] > 0.1) {
      if (setPostureAndStiffness(PostureState::stand, StiffnessState::max, MOTION_1)) {
        if (motionConfig) {
          setupMBRequest(MOTION_1, boost::static_pointer_cast<MBConfig>(motionConfig));
        } else {
          finish();
        }
      }
    }
  }
}

void MotionTestSuite::finish()
{
  LOG_INFO("MotionTestSuite.finish() called...")
  inBehavior = false;
}

MotionTestSuiteConfigPtr MotionTestSuite::getBehaviorCast()
{
  return boost::static_pointer_cast <MotionTestSuiteConfig> (config);
}
