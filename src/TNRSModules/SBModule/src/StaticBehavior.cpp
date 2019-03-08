/**
 * @file SBModule/src/StaticBehavior.cpp
 *
 * This file implements the class StaticBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "SBModule/include/SBModule.h"
#include "SBModule/include/StaticBehavior.h"
#include "Utils/include/Constants.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/DebugUtils.h"

StaticBehavior::StaticBehavior(
  SBModule* sbModule,
  const BehaviorConfigPtr& config,
  const string& name) :
  Behavior(config, name),
  MemoryBase(sbModule),
  sbModule(sbModule)
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  motionProxy = sbModule->getSharedMotionProxy();
  #endif
  this->cycleTime = sbModule->getPeriodMinMS() / 1000.f;
}

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
void StaticBehavior::naoqiStiffnessInterpolation(
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
