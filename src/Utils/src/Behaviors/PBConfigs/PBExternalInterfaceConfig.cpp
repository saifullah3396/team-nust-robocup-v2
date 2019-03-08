/**
 * @file Utils/include/Behaviors/PBConfigs/PBExternalInterfaceConfig.h
 *
 * This file defines the struct PBExternalInterfaceConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "Utils/include/Behaviors/PBConfigs/PBExternalInterfaceConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  PBExternalInterfaceConfig, PBConfig, PBExternalInterfaceConfigPtr,
  (PBExternalInterfaceTypes, nihaCognition, NIHACognitionConfig),
  (PBExternalInterfaceTypes, userReqHandler, UserRequestsHandlerConfig),
)

void NIHACognitionConfig::validate()
{
  
}

void UserRequestsHandlerConfig::validate()
{
  
}
