/**
 * @file PlanningModule/src/PBManager.cpp
 *
 * This file implements the class PBManager
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 02 Aug 2018
 */

#include "PlanningModule/include/PlanningModule.h"
#include "PlanningModule/include/PBManager.h"
#include "PlanningModule/include/PlanningBehaviors/RobotStartup/RobotStartup.h"
#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/NavigationBehavior.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Robocup.h"
//#include "PlanningModule/include/PlanningBehaviors/KickSequence/KickSequence.h"
//#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/ExternalInterface.h"
#include "PlanningModule/include/PlanningBehaviors/TestSuite/TestSuite.h"

PBManager::PBManager(PlanningModule* planningModule) :
  BehaviorManager("PBManager"),
  planningModule(planningModule)
{
}

bool PBManager::makeBehavior(
  BehaviorPtr& behavior, const BehaviorConfigPtr& cfg)
{
  if (cfg->baseType != BaseBehaviorType::planning)
    return false;
  if (cfg->id == toUType(PBIds::startup)) {
    behavior =
      BehaviorPtr(RobotStartup::getType(planningModule, cfg));
  } else if (cfg->id == toUType(PBIds::navigation)) {
    behavior =
      BehaviorPtr(NavigationBehavior::getType(planningModule, cfg));
  } else if (cfg->id == toUType(PBIds::robocup)) {
    behavior =
      BehaviorPtr(Robocup::getType(planningModule, cfg));
//  } else if (cfg->id == toUType(PBIds::kickSequence)) {
//    behavior =
//      BehaviorPtr(KickSequence::getType(planningModule, cfg));
//  } else if (cfg->id == toUType(PBIds::externalInterface)) {
//    behavior =
//      BehaviorPtr(ExternalInterface::getType(planningModule, cfg));
  } else if (cfg->id == toUType(PBIds::testSuite)) {
    behavior =
      BehaviorPtr(TestSuite::getType(planningModule, cfg));
  } else {
    return false;
  }
  return true;
}
