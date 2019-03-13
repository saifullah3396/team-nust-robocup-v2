/**
 * @file GBModule/src/GeneralBehavior.cpp
 *
 * This file implements the class GeneralBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "GBModule/include/GBModule.h"
#include "GBModule/include/GeneralBehavior.h"
#include "Utils/include/Constants.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/DebugUtils.h"

GeneralBehavior::GeneralBehavior(
  GBModule* gbModule,
  const BehaviorConfigPtr& config,
  const string& name) :
  Behavior(config, name),
  MemoryBase(gbModule),
  gbModule(gbModule)
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  motionProxy = gbModule->getSharedMotionProxy();
  #endif
  this->cycleTime = gbModule->getPeriodMinMS() / 1000.f;
}

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
void GeneralBehavior::naoqiStiffnessInterpolation(
  const vector<unsigned>& ids,
  const AL::ALValue& timeLists, const AL::ALValue& stiffnessLists,
  const bool& postCommand)
{
  ASSERT(
    ids.size() == timeLists.getSize() && timeLists.getSize() == stiffnessLists.getSize());
  AL::ALValue names;
  names.clear();
  names.arraySetSize(ids.size());
  for (int i = 0; i < ids.size(); ++i) {
    names[i] = Constants::jointNames[ids[i]];
  }
  try {
    if (postCommand) {
      motionProxy->post.stiffnessInterpolation(
        names,
        stiffnessLists,
        timeLists);
    } else {
      motionProxy->stiffnessInterpolation(names, stiffnessLists, timeLists);
    }
  } catch (exception &e) {
    LOG_EXCEPTION(e.what())
  }
}
#endif
