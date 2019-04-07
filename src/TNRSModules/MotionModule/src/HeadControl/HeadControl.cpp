/**
 * @file MotionModule/src/HeadControl/HeadControl.cpp
 *
 * This file implements the class HeadControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 20 Nov 2017
 */

#include "MotionModule/include/MotionModule.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "MotionModule/include/HeadControl/HeadControl.h"
#include "MotionModule/include/HeadControl/HeadTargetTypes.h"
#include "MotionModule/include/HeadControl/Types/HeadTargetTrack.h"
#include "MotionModule/include/HeadControl/Types/HeadScan.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/GoalInfo.h"

template <typename Scalar>
HeadControl<Scalar>::HeadControl(
  MotionModule* motionModule,
  const boost::shared_ptr<MBHeadControlConfig>& config,
  const string& name) :
  MotionBehavior<Scalar>(motionModule, config, name)
{
}

template <typename Scalar>
boost::shared_ptr<HeadControl<Scalar> > HeadControl<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg)
{
  HeadControl<Scalar>* hc;
  switch (cfg->type) {
      case toUType(MBHeadControlTypes::headScan):
        hc = new HeadScan<Scalar>(motionModule, SPC(HeadScanConfig, cfg)); break;
      case toUType(MBHeadControlTypes::headTargetTrack):
        hc = new HeadTargetTrack<Scalar>(motionModule, SPC(HeadTargetTrackConfig, cfg)); break;
  }
  return boost::shared_ptr<HeadControl<Scalar> >(hc);
}

template <typename Scalar>
bool HeadControl<Scalar>::findTarget(
  const HeadTargetTypes& targetType,
  Matrix<Scalar, 4, 1>& targetPos,
  bool& trackable)
{
  try {
    if (targetType == HeadTargetTypes::ball) {
      auto ballInfo = BALL_INFO_IN(MotionModule);
      if (ballInfo.found) {
        targetPos[0] = ballInfo.posRel.x;
        targetPos[1] = ballInfo.posRel.y;
        targetPos[2] = ballInfo.radius;
        targetPos[3] = 1.0;
        trackable = true;
        return true;
      } else {
        return false;
      }
    } else if (targetType == HeadTargetTypes::goal) {
      auto goalInfo = GOAL_INFO_IN(MotionModule);
      if (goalInfo.found)
      {
        if (goalInfo.mid.x == goalInfo.mid.x) {
          targetPos[0] = goalInfo.mid.x;
          targetPos[1] = goalInfo.mid.y;
        } else if (goalInfo.leftPost.x == goalInfo.leftPost.x) {
          targetPos[0] = goalInfo.leftPost.x;
          targetPos[1] = goalInfo.leftPost.y;
        } else if (goalInfo.rightPost.x == goalInfo.rightPost.x) {
          targetPos[0] = goalInfo.rightPost.x;
          targetPos[1] = goalInfo.rightPost.y;
        }
        targetPos[2] = 0.0;
        targetPos[3] = 1.0;
        trackable = true;
        return true;
      } else {
        return false;
      }
    } else if (targetType == HeadTargetTypes::landmarks) {
      if (LANDMARKS_FOUND_IN(MotionModule)) {
        targetPos[0] = NAN;
        targetPos[1] = NAN;
        targetPos[2] = 0.0;
        targetPos[3] = 1.0;
        trackable = false;
        return true;
      } else {
        return false;
      }
    } else {
      throw BehaviorException(
        this, "Undefined target type for HeadControl", false);
    }
  } catch (BehaviorException& e) {
    LOG_EXCEPTION(e.what());
    this->inBehavior = false;
    return false;
  }
}

template class HeadControl<MType>;
