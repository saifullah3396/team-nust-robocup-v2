/**
 * @file MotionModule/src/KickModule/JSOImpKick.cpp
 *
 * This file implements the class JSOImpKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#include "MotionModule/include/KickModule/MaxMomentumEEOpt.h"
#include "MotionModule/include/KickModule/Types/JSOImpKick.h"
#include "MotionModule/include/BalanceModule/BalanceDefinitions.h"
#include "MotionModule/include/JointRequest.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/MotionLogger.h"
#include "MotionModule/include/TrajectoryPlanner/ZmpConstraint.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBKickConfig.h"
#include "Utils/include/DataHolders/PostureState.h"

using namespace Utils;

template <typename Scalar>
JSOImpKick<Scalar>::JSOImpKick(
  MotionModule* motionModule,
  const boost::shared_ptr<JSOImpKickConfig>& config) :
  JointSpaceKick<Scalar>(motionModule, config, "JSOImpKick")
{
  maxMomentumEEOpt = boost::make_shared<MaxMomentumEEOpt<Scalar> >(this);
}

template <typename Scalar>
boost::shared_ptr<JSOImpKickConfig> JSOImpKick<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <JSOImpKickConfig> (this->config);
}

template <typename Scalar>
void JSOImpKick<Scalar>::setupKickBase()
{
  LOG_INFO("JSOImpKick.setupKickBase()")
  if (this->config->logData) {
    JSON_ASSIGN(
      this->dataLogger->getRoot(),
      "config",
      this->getBehaviorCast()->getJson()
    );
  }
  auto& ball = this->getBehaviorCast()->ball;
  auto& target = this->getBehaviorCast()->target;
  auto& reqVel = this->getBehaviorCast()->reqVel;
  auto& targetDistAngle = this->getBehaviorCast()->targetDistAngle;
  if (reqVel.x != -1.f || target.x != -1.f || targetDistAngle[0] != -1.f) {
    this->ballPosition = Matrix<Scalar, 3, 1>(ball.x, ball.y, this->ballRadius);
    if (target.x != -1.f) { // if target is defined use this
      this->targetPosition = Matrix<Scalar, 3, 1>(target.x, target.y, this->ballRadius);
      auto ballToTarget = this->targetPosition - this->ballPosition;
      this->targetDistance = ballToTarget.norm();
      this->ballToTargetUnit = ballToTarget / this->targetDistance;
      this->targetAngle = atan2(this->ballToTargetUnit[1], this->ballToTargetUnit[0]);
      this->desImpactVelKnown = false;
    } else if (reqVel.x != -1) { // if des velocity is defined do this
      this->targetAngle = reqVel.y * M_PI / 180.0;
      this->ballToTargetUnit =
        Matrix<Scalar, 3, 1>(cos(this->targetAngle), sin(this->targetAngle), 0.0);
      this->desImpactVel = reqVel.x * this->ballToTargetUnit;
      this->desImpactVelKnown = true;
    } else if (targetDistAngle[0] != -1.f) { // if target is defined use this
      targetDistAngle[1] *= M_PI / 180.0;
      this->targetPosition =
        Matrix<Scalar, 3, 1>(
          this->ballPosition[0] + targetDistAngle[0] * cos(targetDistAngle[1]),
          this->ballPosition[1] + targetDistAngle[0] * sin(targetDistAngle[1]),
          this->ballRadius);
      this->targetDistance = targetDistAngle[0];
      this->ballToTargetUnit =
        Matrix<Scalar, 3, 1>(cos(targetDistAngle[1]), sin(targetDistAngle[1]), 0.0);
      this->targetAngle = targetDistAngle[1];
      this->desImpactVelKnown = false;
    }
    if (!this->setKickSupportLegs()) {
      throw BehaviorException(
        this,
        "Unable to decide kick and support legs for the given ball position.",
        false
      );
    }
    Scalar footSpacing = this->kickLeg == LinkChains::rLeg ? this->kM->getFootSpacing() : -this->kM->getFootSpacing();
    // Sending ball from feet center frame to base support leg frame
    this->ballPosition[1] -= footSpacing / 2;
    if (target.x != -1.f)
      this->targetPosition[1] -= footSpacing / 2;
    if (!this->setEndEffectorXY(this->targetAngle)) {
      throw BehaviorException(
        this,
        "Unable to set end effector for kick.",
        false
      );
    }
    this->kickTaskJoints = vector<bool>(toUType(Joints::count), false);
    auto kickChain = this->kM->getLinkChain(this->kickLeg);
    for (size_t i = 0; i < kickChain->size; ++i)
      this->kickTaskJoints[kickChain->start + i] = true;

    this->armsTaskJoints = vector<bool>(toUType(Joints::count), false);
    auto lArmChain = this->kM->getLinkChain(LinkChains::lArm);
    for (size_t i = 0; i < lArmChain->size; ++i)
      this->armsTaskJoints[lArmChain->start + i] = true;
    auto rArmChain = this->kM->getLinkChain(LinkChains::rArm);
    for (size_t i = 0; i < rArmChain->size; ++i)
      this->armsTaskJoints[rArmChain->start + i] = true;
    this->zmpControlCfgInKick = boost::make_shared <ZmpControlConfig> ();
    this->zmpControlCfgInKick->supportLeg = this->supportLeg;
    this->zmpControlCfgInKick->keepOtherLegContact = false;
    this->zmpControlCfgInKick->regularizePosture = true;
    this->zmpControlCfgInKick->keepTorsoUpright = false;
    auto activeJoints = vector<bool>(toUType(Joints::count), false);
    activeJoints[toUType(Joints::lShoulderPitch)] = true;
    activeJoints[toUType(Joints::lShoulderRoll)] = true;
    activeJoints[toUType(Joints::rShoulderPitch)] = true;
    activeJoints[toUType(Joints::rShoulderRoll)] = true;
    this->zmpControlCfgInKick->activeJoints = vector<unsigned>(activeJoints.begin(), activeJoints.end());
    this->zmpControlCfgInKick->target[0] = -0.02;
    this->zmpControlCfgInKick->target[1] = 0.0;

    /*Matrix<Scalar, Dynamic, 1> postureTarget(toUType(Joints::count));
    if (this->supportLeg == LinkChains::lLeg) {
      postureTarget = Matrix<Scalar, Dynamic, 1>::Map(
        &balanceDefs[0][0],
        sizeof(balanceDefs[0]) / sizeof(balanceDefs[0][0]));
    } else {
      postureTarget = Matrix<Scalar, Dynamic, 1>::Map(
        &balanceDefs[1][0],
        sizeof(balanceDefs[1]) / sizeof(balanceDefs[1][0]));
    }*/
    //this->kM->setJointPositions(0, postureTarget, JointStateType::sim);
    this->setTransformFrames(JointStateType::actual);
    //this->kM->setStateFromTo(JointStateType::actual, JointStateType::sim);
    //this->setTransformFrames(JointStateType::sim);
    //findBestEEAndImpactPose();
    //this->defineTrajectory();
    if (this->config->logData) {
      Json::Value jsonSetup;
      JSON_ASSIGN(jsonSetup, "targetPosition", JsonUtils::matrixToJson(this->targetPosition));
      JSON_ASSIGN(jsonSetup, "actualPosition", Json::nullValue);
      JSON_ASSIGN(jsonSetup, "targetDistance", this->targetDistance);
      JSON_ASSIGN(jsonSetup, "targetAngle", this->targetAngle * 180.0 / M_PI);
      JSON_ASSIGN(jsonSetup, "kickLeg", toUType(this->kickLeg));
      JSON_ASSIGN(jsonSetup, "supportLeg", toUType(this->supportLeg));
      JSON_ASSIGN(jsonSetup, "ballToTargetUnit", JsonUtils::matrixToJson(this->ballToTargetUnit));
      JSON_ASSIGN(jsonSetup, "footSpacingInit", this->footSpacing);
      JSON_ASSIGN(jsonSetup, "supportToTorsoInit", JsonUtils::matrixToJson(this->supportToTorso));
      JSON_ASSIGN(jsonSetup, "supportToKickInit", JsonUtils::matrixToJson(this->supportToKick));
      JSON_ASSIGN(jsonSetup, "endEffectorInit", JsonUtils::matrixToJson(this->endEffector));
      JSON_ASSIGN(this->dataLogger->getRoot(), "setup", jsonSetup);
    }
  } else {
    throw BehaviorException(
      this,
      "Required kick parameters 'ball', 'reqVel' or 'target are not well-defined",
      false
    );
  }
}

template <typename Scalar>
void JSOImpKick<Scalar>::solveForImpact()
{
  this->impactPose.setIdentity();
  this->impactPose(0, 3) = this->ballPosition[0] - this->ballToTargetUnit[0] * (this->ballRadius);
  this->impactPose(1, 3) = this->ballPosition[1] - this->ballToTargetUnit[1] * (this->ballRadius);
  this->impactPose(2, 3) = this->ballPosition[2];
  //maxMomentumEEOpt->optDef();
}

template class JSOImpKick<MType>;
