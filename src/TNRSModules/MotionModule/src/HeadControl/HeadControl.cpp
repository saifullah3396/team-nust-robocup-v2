/**
 * @file MotionModule/src/HeadControl/HeadControl.cpp
 *
 * This file implements the class HeadControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 20 Nov 2017
 */

#include "MotionModule/include/HeadControl/HeadControl.h"
#include "MotionModule/include/HeadControl/Types/HeadTargetTrack.h"
#include "MotionModule/include/HeadControl/Types/HeadTargetSearch.h"
#include "MotionModule/include/HeadControl/Types/HeadScan.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/GoalInfo.h"

template <typename Scalar>
boost::shared_ptr<HeadControl<Scalar> > HeadControl<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  HeadControl<Scalar>* hc;
  switch (cfg->type) {
      case toUType(MBHeadControlTypes::headTargetTrack):
        hc = new HeadTargetTrack<Scalar>(motionModule, cfg); break;
      case toUType(MBHeadControlTypes::headTargetSearch):
        hc = new HeadTargetSearch<Scalar>(motionModule, cfg); break;
      case toUType(MBHeadControlTypes::headScan):
        hc = new HeadScan<Scalar>(motionModule, cfg); break;
  }
  return boost::shared_ptr<HeadControl<Scalar> >(hc);
}

template <typename Scalar>
bool HeadControl<Scalar>::findTarget(
  const HeadTargetTypes& targetType, 
  cv::Point_<Scalar>& targetXY,
  Scalar& targetZ)
{
  try {
    if (targetType == HeadTargetTypes::ball) {
      auto ballInfo = BALL_INFO_IN(MotionModule);
      if (ballInfo.found) {
        targetXY = ballInfo.posRel;
        targetZ = 0.05f;
        return true;
      } else {
        return false;
      }
    } else if (targetType == HeadTargetTypes::GOAL) {
      auto goalInfo = GOAL_INFO_IN(MotionModule);
      if (
        goalInfo.found && 
        goalInfo.leftPost.x > -50 && 
        goalInfo.rightPost.x > -50) 
      {
        targetXY = goalInfo.mid;
        targetZ = 0.f;
        return true;
      } else {
        return false;
      }
    } else if (targetType == HeadTargetTypes::LANDMARKS) {
      if (LANDMARKS_FOUND_IN(MotionModule)) {
        targetZ = -1.f;
        return true;
      } else {
        return false;
      }
    } else {
      throw BehaviorException(
        this, "Undefined target type for HeadControl", false, EXC_INVALID_BEHAVIOR_SETUP);
    }
  } catch (BehaviorException& e) {
    LOG_EXCEPTION(e.what());
    this->inBehavior = false;
    return false;
  }
}

template class HeadControl<MType>;
