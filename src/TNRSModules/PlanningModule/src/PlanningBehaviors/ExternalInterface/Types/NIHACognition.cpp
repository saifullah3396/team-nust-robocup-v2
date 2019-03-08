/**
 * @file PlanningBehaviors/CognitionModule/Types/NIHACognition.cpp
 *
 * This file implements the class NIHACognition
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "UserCommModule/include/UserCommRequest.h"
#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/Types/NIHACognition.h"
#include "VisionModule/include/VisionRequest.h"
#include "Utils/include/VisionUtils.h"

NIHACognitionConfigPtr NIHACognition::getBehaviorCast()
{
  return boost::static_pointer_cast <NIHACognitionConfig> (config);
}

void
NIHACognition::initiate()
{
  LOG_INFO("NIHACognition.initiate()...")
  auto vRequest = boost::make_shared<SwitchVision>(true);
  BaseModule::publishModuleRequest(vRequest);
  inBehavior = true;
}

void
NIHACognition::update()
{
}

void NIHACognition::finish()
{
  inBehavior = false;
}

void NIHACognition::waitForConnAction()
{
}

void NIHACognition::sendHeaderAction()
{
}

void NIHACognition::sendDataAction()
{
}

void NIHACognition::dataToString(string& data)
{
}
