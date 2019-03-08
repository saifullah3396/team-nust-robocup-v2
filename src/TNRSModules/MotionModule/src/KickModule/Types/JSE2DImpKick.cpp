/**
 * @file MotionModule/src/KickModule/JSE2DImpKick.cpp
 *
 * This file implements the class JSE2DImpKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#include "BehaviorManager/include/StateMachine.h"
#include "MotionModule/include/MotionLogger.h"
#include "MotionModule/include/KickModule/Types/JSE2DImpKick.h"
#include "MotionModule/include/KickModule/Types/KickImpact2DSolver.h"
#include "Utils/include/Behaviors/MBConfigs/MBKickConfig.h"
#include "Utils/include/Constants.h"

#define JSE2D_KICK_PTR static_cast<JSE2DImpKick<Scalar>*>(this->bPtr)

template <typename Scalar>
JSE2DImpKick<Scalar>::JSE2DImpKick(
  MotionModule* motionModule,
  const boost::shared_ptr<JSE2DImpKickConfig>& config) :
  JointSpaceKick<Scalar>(motionModule, config, "JSE2DImpKick")
{
  DEFINE_FSM_STATE(JointSpaceKick<Scalar>, JSE2DPlanKick, this->planKick)
  DEFINE_FSM_STATE(JointSpaceKick<Scalar>, WaitForExecution, waitForExecution)
  kickImpact2DSolver = boost::make_shared<KickImpact2DSolver<Scalar>>(this);
}

template <typename Scalar>
bool JSE2DImpKick<Scalar>::initiate()
{
  if (this->config->logData)
    MOTION_LOGGER->setRefTime(this->getBehaviorCast()->timeAtEstimation);
  return JointSpaceKick<Scalar>::initiate();
}

template <typename Scalar>
void JSE2DImpKick<Scalar>::JSE2DPlanKick::onRun()
{
  JointSpaceKick<Scalar>::PlanKick::onRun();
  if (!JSE2D_KICK_PTR->kickFailed) {
    this->nextState = JSE2D_KICK_PTR->waitForExecution.get();
  }
}

template <typename Scalar>
void JSE2DImpKick<Scalar>::WaitForExecution::onRun()
{
  auto timeFromStart =
    duration<double>(
      high_resolution_clock::now() -
      JSE2D_KICK_PTR->getBehaviorCast()->timeAtEstimation
    ).count();
  auto timeToKick =
    JSE2D_KICK_PTR->getBehaviorCast()->timeUntilImpact - JSE2D_KICK_PTR->kickTimeToImpact;
  if (MathsUtils::almostEqual((Scalar)timeFromStart, (Scalar)timeToKick, (Scalar)JSE2D_KICK_PTR->cycleTime)) {
    this->nextState = JSE2D_KICK_PTR->executeKick.get();
  }
}

template <typename Scalar>
void JSE2DImpKick<Scalar>::setupKickBase()
{
  try {
    //LOG_INFO("JSE2DImpKick.setupKickBase()")
    auto& ball = this->getBehaviorCast()->ball;
    auto& target = this->getBehaviorCast()->target;
    auto& reqVel = this->getBehaviorCast()->reqVel;
    //cout << "ball: " << ball << endl;
    //cout << "target: " << target << endl;
    //cout << "reqVel: " << reqVel << endl;
    if (target.x != -1.f) { // if target is defined
      this->ballPosition = Matrix<Scalar, 3, 1>(ball.x, ball.y, -Constants::footHeight + this->ballRadius);
      this->ballVelocity =
        Matrix<Scalar, 3, 1>(this->getBehaviorCast()->ballVel.x, this->getBehaviorCast()->ballVel.y, 0.f);
      this->targetPosition = Matrix<Scalar, 3, 1>(target.x, target.y, -Constants::footHeight + this->ballRadius);
      auto ballToTarget = this->targetPosition - this->ballPosition;
      this->targetDistance = ballToTarget.norm();
      this->ballToTargetUnit = ballToTarget / this->targetDistance;
      this->targetAngle = atan2(this->ballToTargetUnit[1], this->ballToTargetUnit[0]);
      if (!this->setKickSupportLegs()) {
        throw BehaviorException(
          this,
          "Unable to decide this->kick and support legs for the given ball position.",
          false
        );
      }
      if (!this->setTransformFrames(JointStateType::actual)) {
        throw BehaviorException(
          this,
          "Unable to set initial transformation frames for the kick.",
          false
        );
      }
      float footSpacing = this->supportToKick(1, 3) / 2;
      // Sending ball from feet center frame to base support leg frame
      this->ballPosition[1] += footSpacing;
      this->targetPosition[1] += footSpacing;
    } else {
      throw BehaviorException(
        this,
        "Required this->kick parameters 'ball' or 'target are not well-defined",
        false
      );
    }
  } catch (BehaviorException& e) {
    LOG_EXCEPTION(e.what());
    this->inBehavior = false;
  }
}

template <typename Scalar>
void JSE2DImpKick<Scalar>::solveForImpact()
{
  this->kickImpact2DSolver->optDef();
}

template <typename Scalar>
boost::shared_ptr<JSE2DImpKickConfig> JSE2DImpKick<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <JSE2DImpKickConfig> (this->config);
}

template class JSE2DImpKick<MType>;
