/**
 * @file TNRSModules/BehaviorConfigs/src/PBConfigs/PBExternalInterfaceConfig.cpp
 *
 * This file defines the struct PBExternalInterfaceConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "BehaviorConfigs/include/PBConfigs/PBExternalInterfaceConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  PBExternalInterfaceConfig, PBConfig, PBExternalInterfaceConfigPtr,
  (PBExternalInterfaceTypes, nihaCognition, NIHACognitionConfig),
  (PBExternalInterfaceTypes, userReqHandler, UserRequestsHandlerConfig),
);

void NIHACognitionConfig::init() {}
void NIHACognitionConfig::validate()
{

}

void UserRequestsHandlerConfig::init() {}
void UserRequestsHandlerConfig::validate()
{

}
