/**
 * @file PlanningBehaviors/ExternalInterface/Types/UserRequestsHandler.cpp
 *
 * This file implements the class UserRequestsHandler
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "BehaviorConfigs/include/MBConfigs/MBTeleopConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBExternalInterfaceConfig.h"
#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/Types/UserRequestsHandler.h"
#include "UserCommModule/include/UserCommRequest.h"
#include "Utils/include/VisionUtils.h"
#include "VisionModule/include/VisionRequest.h"

boost::shared_ptr<UserRequestsHandlerConfig> UserRequestsHandler::getBehaviorCast()
{
  return SPC(UserRequestsHandlerConfig, config);
}

bool UserRequestsHandler::initiate()
{
  LOG_INFO("UserRequestsHandler.initiate()...")
  inBehavior = true;
}

void UserRequestsHandler::update()
{
  //PRINT("UserRequestsHandler.update()...")
  if (requestInProgress()) return;
  static auto prevJoints = GET_DVAR(vector<float>, jointCommands);
  auto currJoints = GET_DVAR(vector<float>, jointCommands);
  if (currJoints != prevJoints) {
    setupMBRequest(0, boost::make_shared<MBTeleopConfig>(currJoints));
  }
  prevJoints = currJoints;
}

void UserRequestsHandler::finish()
{
  inBehavior = false;
}
