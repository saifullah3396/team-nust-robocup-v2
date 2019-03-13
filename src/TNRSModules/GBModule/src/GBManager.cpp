/**
 * @file GBModule/src/GBManager.cpp
 *
 * This file implements the class GBManager
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 02 Aug 2018
 */

#include "GBModule/include/GBModule.h"
#include "GBModule/include/GBManager.h"
#include "GBModule/include/StiffnessModule/StiffnessModule.h"
#include "GBModule/include/LedsModule/LedsModule.h"
#include "GBModule/include/WhistleDetector/WhistleDetector.h"
#include "GBModule/include/GeneralBehaviorIds.h"

GBManager::GBManager(GBModule* gbModule) :
  BehaviorManager("GBManager"), gbModule(gbModule)
{
}

bool GBManager::makeBehavior(
  BehaviorPtr& behavior, const BehaviorConfigPtr& cfg)
{
  if (cfg->baseType != BaseBehaviorType::general)
    return false;
  if (cfg->id == toUType(GBIds::stiffnessModule)) {
    behavior = BehaviorPtr(StiffnessModule::getType(gbModule, cfg));
  } else if (cfg->id == toUType(GBIds::ledsModule)) {
    behavior = BehaviorPtr(LedsModule::getType(gbModule, cfg));
  } else if (cfg->id == toUType(GBIds::whistleDetector)) {
    behavior = BehaviorPtr(WhistleDetector::getType(gbModule, cfg));
  } else {
    return false;
  }
  return true;
}
