/**
 * @file MotionModule/include/Kicthis->kModule/JointSpaceKick.cpp
 *
 * This file implements the class JointSpaceKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#include "BehaviorManager/include/StateMachine.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/KinematicsModule/ComState.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/KinematicsModule/TaskIkSolver.h"
#include "MotionModule/include/BalanceModule/BalanceDefinitions.h"
#include "MotionModule/include/KickModule/Types/JointSpaceKick.h"
#include "MotionModule/include/TrajectoryPlanner/ZmpConstraint.h"
#include "MotionModule/include/TrajectoryPlanner/TorqueConstraint.h"
#include "MotionModule/include/TrajectoryPlanner/CbOptimizer.h"
#include "Utils/include/Behaviors/MBConfigs/MBBalanceConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBKickConfig.h"
#include "Utils/include/Splines/CubicSpline.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/JsonLogger.h"
#include "Utils/include/PlotEnv.h"
#include "Utils/include/VisionUtils.h"

using namespace GnuPlotEnv;
using namespace Utils;

template <typename Scalar>
Scalar JointSpaceKick<Scalar>::preImpactBeta = 0.5;
template <typename Scalar>
Scalar JointSpaceKick<Scalar>::preImpactAlpha = 0.02;
template <typename Scalar>
Scalar JointSpaceKick<Scalar>::postImpactGroundDist = 0.01;
template <typename Scalar>
unsigned JointSpaceKick<Scalar>::numIkIterations = 100;
template <typename Scalar>
Scalar JointSpaceKick<Scalar>::knotsInitialValue = 0.2;
template <typename Scalar>
bool JointSpaceKick<Scalar>::solveForImpactOmega = false;
template <typename Scalar>
bool JointSpaceKick<Scalar>::useHipYawPitchJoint = false;
template <typename Scalar>
bool JointSpaceKick<Scalar>::addTorqueConstraint = true;
template <typename Scalar>
bool JointSpaceKick<Scalar>::addZmpConstraint = true;
template <typename Scalar>
unsigned JointSpaceKick<Scalar>::maxConstantVelIterations = 10;
template <typename Scalar>
Scalar JointSpaceKick<Scalar>::constantVelCutoffDist = 0.5;
template <typename Scalar>
Scalar JointSpaceKick<Scalar>::afterKickWait = 0.1;
template <typename Scalar>
Scalar JointSpaceKick<Scalar>::afterBalanceWait = 0.25;

template <typename Scalar>
JointSpaceKick<Scalar>::JointSpaceKick(
  MotionModule* motionModule,
  const boost::shared_ptr<JSKickConfig>& config,
  const string& name) :
  KickModule<Scalar>(motionModule, config, name)
{
  DEFINE_FSM_STATE(JointSpaceKick<Scalar>, SetStartPosture, setStartPosture);
  DEFINE_FSM_STATE(JointSpaceKick<Scalar>, GoToBalance, goToBalance);
  DEFINE_FSM_STATE(JointSpaceKick<Scalar>, PlanKick, planKick);
  DEFINE_FSM_STATE(JointSpaceKick<Scalar>, ExecuteKick, executeKick);
  DEFINE_FSM_STATE(JointSpaceKick<Scalar>, SetPostKickPosture, setPostKickPosture);
  DEFINE_FSM(fsm, JointSpaceKick<Scalar>, setStartPosture);
}

template <typename Scalar>
bool JointSpaceKick<Scalar>::initiate()
{
  LOG_INFO("JointSpaceKick.initiate() called...")
  try {
    setupKickBase();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    return false;
  }

  if (this->getBehaviorCast()->postureConfig)
    this->fsm->state = setStartPosture.get();
  else if (this->getBehaviorCast()->balanceConfig)
    this->fsm->state = goToBalance.get();
  else
    this->fsm->state = executeKick.get();
  return true;
}

template <typename Scalar>
void JointSpaceKick<Scalar>::update()
{
  LOG_INFO("FsmState:" << fsm->state->name)
  if (fsm->update())
    finish();
}

template <typename Scalar>
void JointSpaceKick<Scalar>::finish()
{
  LOG_INFO("JointSpaceKick.finish() called...")
  this->inBehavior = false;
}

template <typename Scalar>
void JointSpaceKick<Scalar>::SetStartPosture::onStart() {
  //! Sets the posture as a blocking child
  this->bPtr->setupPosture();
}

template <typename Scalar>
void JointSpaceKick<Scalar>::SetStartPosture::onRun()
{
  this->nextState = this->bPtr->goToBalance.get();
}

template <typename Scalar>
void JointSpaceKick<Scalar>::GoToBalance::onStart() {
  //! Sets the balancer in parallel
  this->bPtr->setupBalance();
}

template <typename Scalar>
void JointSpaceKick<Scalar>::GoToBalance::onRun()
{
  static bool balanceUpdated = false;
  static Scalar wait = 0.0;
  if (
    wait >
    this->bPtr->getBehaviorCast()->balanceConfig->timeToReachB +
    this->bPtr->afterBalanceWait)
  {
    //! If balance is reached change the state to kicking
    this->nextState = this->bPtr->planKick.get();
    balanceUpdated = false;
    wait = 0.0;
  } else if (
    this->bPtr->getBehaviorCast()->balanceConfig->type == (int)MBBalanceTypes::zmpControl &&
    wait > this->bPtr->getBehaviorCast()->balanceConfig->timeToReachB &&
    !balanceUpdated)
  {
    boost::static_pointer_cast<ZmpControlConfig>(this->bPtr->getBehaviorCast()->balanceConfig)->keepOtherLegContact = false;
    this->bPtr->setupBalance();
    balanceUpdated = true;
    wait += this->bPtr->cycleTime;
  } else {
    wait += this->bPtr->cycleTime;
  }
}

template <typename Scalar>
void JointSpaceKick<Scalar>::PlanKick::onRun() {
  //! Get the current balancing pose
  this->bPtr->kM->setStateFromTo(JointStateType::actual, JointStateType::sim);
  //! Set the transformation frames
  this->bPtr->setTransformFrames(JointStateType::sim);
  //! Solve the impact conditions
  this->bPtr->solveForImpact();
  //! Plan kicking trajectory
  this->bPtr->defineTrajectory();
  //! Plot the kicking trajectory
  //this->bPtr->plotKick();
  if (!this->bPtr->kickFailed) {
    this->nextState = this->bPtr->executeKick.get();
  } else {
    this->nextState = this->bPtr->setStartPosture.get();
  }
}

template <typename Scalar>
void JointSpaceKick<Scalar>::ExecuteKick::onStart()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  if (this->bPtr->getBehaviorCast()->inKickBalance) {
    this->bPtr->requestExecution(true);
  } else {
    this->bPtr->requestExecution();
  }
  #else
  //! If Naoqi motion proxy is not used
  //! Reset the real-time balancer to start using hands for balancing
  if (this->bPtr->getBehaviorCast()->inKickBalance) {
    this->bPtr->getBehaviorCast()->balanceConfig = this->bPtr->zmpControlCfgInKick;
    this->bPtr->setupBalance();
  } else {
    this->bPtr->killChild();
  }
  #endif
}

template <typename Scalar>
void JointSpaceKick<Scalar>::ExecuteKick::onRun()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  if (this->bPtr->execTime > this->bPtr->totalTimeToKick) {
    this->nextState = this->bPtr->setPostKickPosture.get();
  } else {
    this->bPtr->logEndEffectorActual();
    this->bPtr->execTime +=  this->bPtr->cycleTime;
  }
  #else
  static unsigned step = 0;
  Matrix<Scalar, Dynamic, 1> desJoints(toUType(Joints::count));
  desJoints.setZero();
  auto kickChain = this->bPtr->kM->getLinkChain(this->bPtr->kickLeg);
  for (size_t i = 0; i < toUType(Joints::count); ++i) {
    if (i >= kickChain->start && i < kickChain->start + kickChain->size)
      desJoints[i] = this->bPtr->jointTrajectories[i - kickChain->start][step];;
  }
  if (this->bPtr->getBehaviorCast()->balanceConfig->type == (int)MBBalanceTypes::mpComControl) {
    this->bPtr->setJointCmds(desJoints);
  } else if (this->bPtr->getBehaviorCast()->balanceConfig->type == (int)MBBalanceTypes::zmpControl) {
    cout << "Adding posture task..." << endl;
    auto kickTask = this->bPtr->kM->makePostureTask(desJoints, this->bPtr->kickTaskJoints, 10, 1.0);
    this->bPtr->addMotionTask(kickTask);
  }
  step++;
  if (step >= this->bPtr->jointTrajectories[0].size()) {
    step = 0;
    this->bPtr->killChild();
    this->nextState = this->bPtr->setPostKickPosture.get();
  }
  #endif
}

template <typename Scalar>
void JointSpaceKick<Scalar>::SetPostKickPosture::onStart() {
  //! Sets the posture as a blocking child after required wait time
  static float wait = 0.f;
  if (wait < this->bPtr->afterKickWait) {
    wait += this->bPtr->cycleTime;
  } else {
    wait = 0.0;
    this->bPtr->setupPosture();
  }
}

template <typename Scalar>
void JointSpaceKick<Scalar>::SetPostKickPosture::onRun()
{
  this->nextState = nullptr; //! State machine finish
}

template <typename Scalar>
JSKickConfigPtr JointSpaceKick<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <JSKickConfig> (this->config);
}

template <typename Scalar>
void JointSpaceKick<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    KickModule<Scalar>::loadExternalConfig();
    GET_CONFIG("MotionBehaviors",
      (Scalar, JointSpaceKick.preImpactBeta, preImpactBeta),
      (Scalar, JointSpaceKick.preImpactAlpha, preImpactAlpha),
      (Scalar, JointSpaceKick.postImpactGroundDist, postImpactGroundDist),
      (unsigned, JointSpaceKick.numIkIterations, numIkIterations),
      (Scalar, JointSpaceKick.knotsInitialValue, knotsInitialValue),
      (bool, JointSpaceKick.solveForImpactOmega, solveForImpactOmega),
      (bool, JointSpaceKick.useHipYawPitchJoint, useHipYawPitchJoint),
      (bool, JointSpaceKick.addTorqueConstraint, addTorqueConstraint),
      (bool, JointSpaceKick.addZmpConstraint, addZmpConstraint),
      (unsigned, JointSpaceKick.maxConstantVelIterations, maxConstantVelIterations),
      (Scalar, JointSpaceKick.constantVelCutoffDist, constantVelCutoffDist),
      (Scalar, JointSpaceKick.afterKickWait, afterKickWait),
      (Scalar, JointSpaceKick.afterBalanceWait, afterBalanceWait),
    )
  }
}

template <typename Scalar>
void JointSpaceKick<Scalar>::defineTrajectory()
{
  //! Get end-effector transformation in support leg frame
  Matrix<Scalar, 4, 4> eeTrans = this->supportToKick * this->endEffector;
  //! Define via points for pre-impact phase.
  const Scalar balldx = this->ballRadius * this->ballToTargetUnit[0];
  const Scalar balldy = this->ballRadius * this->ballToTargetUnit[1];
  Matrix<Scalar, 4, 4> preImpactPose2, preImpactPose1, preImpactPose0;
  //preImpactPose3.setIdentity();
  //preImpactPose3(0, 3) = this->impactPose(0, 3);
  //preImpactPose3(1, 3) = this->impactPose(1, 3);
  //preImpactPose3(2, 3) = this->impactPose(2, 3);
  //preImpactPose3(0, 3) =
  //  preImpactPose3(0, 3) - balldx * preImpactBeta;
  //preImpactPose3(1, 3) =
  //  preImpactPose3(1, 3) - balldy * preImpactBeta;
  preImpactPose0 = eeTrans;
  preImpactPose1 = eeTrans;
  preImpactPose1(2, 3) += preImpactAlpha;
  preImpactPose2.setIdentity();
  preImpactPose2(0, 3) = (this->impactPose(0, 3) + preImpactPose1(0, 3)) / 2;
  preImpactPose2(1, 3) = (this->impactPose(1, 3) + preImpactPose1(1, 3)) / 2;
  preImpactPose2(2, 3) = (this->impactPose(2, 3) + preImpactPose1(2, 3)) / 2;
  preImpactPose2(0, 3) =
    preImpactPose2(0, 3) - balldx * preImpactBeta;
  preImpactPose2(1, 3) =
    preImpactPose2(1, 3) - balldy * preImpactBeta;
  //! Define via points for post-impact phase
  Matrix<Scalar, 4, 4> postImpactPose2, postImpactPose1;
  postImpactPose1 = preImpactPose1;
  postImpactPose1(2, 3) += postImpactGroundDist;
  postImpactPose2 = preImpactPose0;
  postImpactPose2(2, 3) += postImpactGroundDist;
  //! Update all cartesian poses
  cPosesPre.push_back(preImpactPose0);
  cPosesPre.push_back(preImpactPose1);
  cPosesPre.push_back(preImpactPose2);
  //cPosesPre.push_back(preImpactPose3);
  cPosesPre.push_back(this->impactPose);
  cPosesPost.push_back(this->impactPose);
  cPosesPost.push_back(postImpactPose1);
  cPosesPost.push_back(postImpactPose2);
  //! Setup for inverse kinematics
  auto kickChain = this->kM->getLinkChain(this->kickLeg);
  unsigned chainStart = kickChain->start;
  unsigned chainSize = kickChain->size;
  Matrix<Scalar, Dynamic, Dynamic> jointPosPre;
  jointPosPre.resize(cPosesPre.size(), chainSize);
  jointPosPre.setZero();
  vector<bool> activeResidual = vector<bool>(6, true);

  //! We do not care about orientation while solving for inverse kinematics
  activeResidual[3] = false;
  activeResidual[4] = false;
  activeResidual[5] = false;
  //! Remove HipYawPitch joint from calclation of inverse kinematics
  vector<bool> activeJoints = vector<bool>(chainSize, true);
  activeJoints[0] = false;
  jointPosPre.row(0) =
    this->kM->getJointPositions(
      static_cast<Joints>(chainStart), chainSize).transpose();
  //! Solve inverse kinematics for poses 1-4. This process is really expensive
  for (int i = 1; i < jointPosPre.rows(); ++i) {
    //if (i >= 5) {
    //  activeResidual[3] = true;
    //  activeResidual[4] = true;
    //  activeResidual[5] = true;
    //}
    this->kM->setStateFromTo(JointStateType::actual, JointStateType::sim);
    //this->kM->setChainPositions(this->kickLeg, jointPosPre.row(i-1).transpose(), JointStateType::sim);
    Matrix<Scalar, Dynamic, 1> angles;
    //if (this->kickLeg == LinkChains::lLeg)
    //  angles = this->kM->inverseLeftLeg(this->endEffector, cPosesPre[i])[0];
    //else
    // angles = this->kM->inverseRightLeg(this->endEffector, cPosesPre[i])[0];
    angles = this->kM->solveJacobianIK(this->kickLeg, this->endEffector, cPosesPre[i], numIkIterations, activeJoints, activeResidual);
    if (angles[0] != angles[0]) {
        LOG_ERROR("Requested kick cannot be performed.")
        this->kickFailed = true;
        this->finish();
        return;
    }
    /*int collisionCheckIterations = 10;
    if (this->checkFootCollision(angles)) {
      for (int j = 0; j < collisionCheckIterations; ++j) {
        cPoses[i](0, 3) += this->ballToTargetUnit[0] * 0.02; // Move towards the ball 2cm
        cPoses[i](1, 3) += this->ballToTargetUnit[1] * 0.02; // Move towards the ball 2cm
        cPosesT[i] = this->torsoToSupport * cPoses[i];
        Matrix<Scalar, Dynamic, 1> angles;
        angles = this->kM->solveJacobianIK(this->kickLeg, this->endEffector, cPosesT[i], 50, JointStateType::sim, true, 5e-4, 0.5, activeJoints);
        if (!this->checkFootCollision(angles))
          break;
      }
    }*/
    jointPosPre.row(i) = angles.transpose();
  }
  //! Setting up pre-impact trajectory.
  //! Required cartesian velocities at initial and final poses
  Matrix<Scalar, Dynamic, 1> desCartesianVel;
  desCartesianVel.resize(6); //x-y-z, r-p-y
  Matrix<Scalar, Dynamic, 1> impactJoints = jointPosPre.row(jointPosPre.rows()-1);
  if (!this->desImpactVelKnown) {
    this->computeDesImpactVel(impactJoints);
  } else {
    Scalar coeffRest = 1.f; // Full elastic collision
    Matrix<Scalar, 3, 1> direction =
    this->torsoToSupport.block(0, 0, 3, 3) * this->ballToTargetUnit;
    this->kM->setChainPositions(this->kickLeg, impactJoints, JointStateType::sim);
    Scalar vm;
    this->kM->computeVirtualMass(this->kickLeg, direction, this->endEffector, vm, JointStateType::sim);
    auto eeVelMag = this->desImpactVel.norm();
    this->desBallVel = eeVelMag / ((vm + this->ballMass) / ((1 + coeffRest) * vm));
    if (this->config->logData) {
      Json::Value jsonImpact;
      JSON_ASSIGN(jsonImpact, "desBallVel", this->desBallVel);
      JSON_ASSIGN(jsonImpact, "ballMass", this->ballMass);
      JSON_ASSIGN(jsonImpact, "coeffRest", coeffRest);
      JSON_ASSIGN(jsonImpact, "virtualMass", vm);
      JSON_ASSIGN(jsonImpact, "desImpactVel", JsonUtils::MatrixToJson(this->desImpactVel));
      JSON_ASSIGN(jsonImpact, "impJoints", JsonUtils::MatrixToJson(impactJoints * 180 / M_PI));
      JSON_ASSIGN(jsonImpact, "endEffector", JsonUtils::MatrixToJson(this->endEffector));
      JSON_ASSIGN(this->dataLogger->getRoot(), "impact", jsonImpact);
    }
  }
  this->desImpactVel[0] = std::min((Scalar)this->desImpactVel[0], (Scalar)1.0);
  this->desImpactVel[1] = std::min((Scalar)this->desImpactVel[1], (Scalar)1.0);
  desCartesianVel.block(0, 0, 3, 1) = this->desImpactVel; // x-y-z velocity
  //! Cartesian velocities in torso frame.
  desCartesianVel.block(0, 0, 3, 1) =
    this->torsoToSupport.block(0, 0, 3, 3) * desCartesianVel.segment(0, 3);
  //cout << "desCartesianVel: " << desCartesianVel << endl;
  //! Find last impact velocity in joints space using hitPose.
  Matrix<Scalar, Dynamic, 1> preImpJVel;
  this->kM->setChainPositions(this->kickLeg, impactJoints, JointStateType::sim); // impact pose joints
  Matrix<Scalar, Dynamic, Dynamic> jacobian;
  if (!solveForImpactOmega) {
    jacobian =
      this->kM->computeLimbJ(this->kickLeg, this->endEffector, JointStateType::sim);
    if (!useHipYawPitchJoint)
      jacobian.col(0).setZero();
    preImpJVel = MathsUtils::pseudoInverseSolve(
      jacobian,
      Matrix<Scalar, Dynamic, 1>(desCartesianVel));
  } else {
    jacobian =
      this->kM->computeLimbJ(this->kickLeg, this->endEffector, JointStateType::sim).block(0, 0, 3, 6);
    if (!useHipYawPitchJoint)
      jacobian.col(0).setZero();
    preImpJVel = MathsUtils::pseudoInverseSolve(
      jacobian,
      Matrix<Scalar, Dynamic, 1>(desCartesianVel.block(0, 0, 3, 1)));
  }
  Matrix<Scalar, Dynamic, Dynamic> jointBoundVels;
  jointBoundVels.resize(2, chainSize);
  jointBoundVels.setZero();
  jointBoundVels.row(1) = preImpJVel.transpose(); // Second row
  //! Pre-impact trajectory optimization
  Matrix<Scalar, Dynamic, 1> knots;
  knots.resize(cPosesPre.size()-1); // 5 poses for pre-impact/post-impact trajectory
  for (size_t i = 0; i < knots.rows(); ++i)
    knots[i] = knotsInitialValue;
  auto cb1 = CubicSpline<Scalar>(
    chainSize,
    jointPosPre,
    knots,
    this->cycleTime,
    jointBoundVels);
  cb1.setup();
  auto cbopt = CbOptimizer<Scalar>(this->motionModule, this->kickLeg, this->supportLeg, &cb1);
  if (addZmpConstraint) {
    vector<bool> activeChains(toUType(LinkChains::count), false);
    activeChains[toUType(this->kickLeg)] = true;
    auto zmpConstraint =
      boost::shared_ptr<ZmpConstraint<Scalar> >(
        new ZmpConstraint<Scalar>(
          this->motionModule, this->supportLeg, JointStateType::sim, activeChains));
    cbopt.addConstraint(zmpConstraint);
  }
  if (addTorqueConstraint) {
    vector<bool> tActiveJoints(toUType(Joints::count), false);
    for (size_t i = 0; i < kickChain->size; ++i)
      activeJoints[kickChain->start+i] = true;
    auto torqueConstraint =
      boost::shared_ptr<TorqueConstraint<Scalar> >(
        new TorqueConstraint<Scalar>(
          this->motionModule, this->supportLeg, JointStateType::sim, tActiveJoints)
      );
    cbopt.addConstraint(torqueConstraint);
  }
  cbopt.optDef();
  if (this->config->logData)
    cbopt.logConstraints(10, 0.0, this->logsDirPath);
  vector<Scalar> trajTime;
  cb1.evaluateSpline(jointTrajectories, trajTime, 0);
  kickTimeToImpact = trajTime.back();
  //! Constant velocity phase
  Matrix<Scalar, Dynamic, 1> prevJoints = impactJoints;
  Matrix<Scalar, Dynamic, 1> prevJointsVel = preImpJVel;
  desCartesianVel.block(0, 0, 3, 1) = this->desImpactVel;
  Matrix<Scalar, Dynamic, 1> velLimits;
  velLimits.resize(kickChain->size);
  auto kickJoints =
    this->kM->getJoints(static_cast<Joints>(kickChain->start), kickChain->size);
  for (size_t i = 0; i < kickChain->size; ++i) {
    velLimits[i] = kickJoints[i]->maxVelocity * 0.95;
  }
  for (size_t i = 0; i < maxConstantVelIterations; ++i) {
    this->kM->setStateFromTo(JointStateType::actual, JointStateType::sim);
    this->kM->setChainPositions(this->kickLeg, prevJoints, JointStateType::sim); // impact pose joints
    Matrix<Scalar, 4, 4> eeTrans =
      this->supportToTorso *
      this->kM->getForwardEffector(this->kickLeg, this->endEffector, JointStateType::sim);
    eeTrans.block(0 , 3, 3, 1) = eeTrans.block(0, 3, 3, 1) + desCartesianVel.block(0, 0, 3, 1) * this->cycleTime;
    Matrix<Scalar, Dynamic, 1> angles;
    angles = this->kM->solveJacobianIK(this->kickLeg, this->endEffector, eeTrans, numIkIterations, activeJoints, activeResidual);
    if (angles[0] != angles[0]) {
        LOG_ERROR("Requested kick cannot be performed.")
        this->kickFailed = true;
        this->finish();
        return;
    }
    if ((angles - prevJoints).norm() < 1e-4) // No change
      continue;
    for (size_t i = 0; i < jointTrajectories.size(); ++i)
      jointTrajectories[i].push_back(angles[i]);
    auto distFromBall = (eeTrans.block(0, 3, 3, 1) - this->ballPosition).norm();
    //LOG_INFO("distFromBall: " << distFromBall)
    if (distFromBall < this->ballRadius * constantVelCutoffDist) {
      break;
    }
    prevJointsVel = (angles - prevJoints) / this->cycleTime;
    if (((prevJointsVel.array().abs() - velLimits.array()) > 0.0).any())
      break;
    prevJoints = angles;
  }
  //! Post impact phase
  //! Second trajectory optimization
  vector < vector<Scalar> > jointTrajectories2;
  jointBoundVels.setZero();
  //! If constant velocity phase is not used use the preImpJVel here
  //jointBoundVels.row(0) = preImpJVel.transpose();
  //! Else use prevJointsVel
  jointBoundVels.row(0) = prevJointsVel.transpose(); // First row
  knots.resize(cPosesPost.size()-1); // 3 poses for post-impact trajectory
  Matrix<Scalar, Dynamic, Dynamic> jointPosPost;
  jointPosPost.resize(cPosesPost.size(), chainSize);
  //! If constant velocity phase is not used use the impact pose as first pose
  //requiredJoints.row(0) = impactJoints.transpose();
  //! Else use the final pose of constant velocity phase
  jointPosPost.row(0) = prevJoints.transpose();
  activeResidual[3] = true;
  activeResidual[4] = true;
  activeResidual[5] = true;
  //! Solve inverse kinematics for post impact poses
  for (int i = 1; i < jointPosPost.rows(); ++i) { //! First pos is known
    this->kM->setStateFromTo(JointStateType::actual, JointStateType::sim);
    //this->kM->setChainPositions(this->kickLeg, jointPosPost.row(i-1).transpose(), JointStateType::sim);
    Matrix<Scalar, Dynamic, 1> angles;
    angles = this->kM->solveJacobianIK(this->kickLeg, this->endEffector, cPosesPost[i], numIkIterations, activeJoints, activeResidual);
    if (angles[0] != angles[0]) {
        LOG_ERROR("Requested kick cannot be performed.")
        this->kickFailed = true;
        this->finish();
        return;
    }
    /*int collisionCheckIterations = 10;
    if (this->checkFootCollision(angles)) {
      for (int j = 0; j < collisionCheckIterations; ++j) {
        cPoses[i](0, 3) += this->ballToTargetUnit[0] * 0.02; // Move towards the ball 2cm
        cPoses[i](1, 3) += this->ballToTargetUnit[1] * 0.02; // Move towards the ball 2cm
        cPosesT[i] = this->torsoToSupport * cPoses[i];
        Matrix<Scalar, Dynamic, 1> angles;
        angles = this->kM->solveJacobianIK(this->kickLeg, this->endEffector, cPosesT[i], 50, JointStateType::sim, true, 5e-4, 0.5, activeJoints);
        if (!this->checkFootCollision(angles))
          break;
      }
    }*/
    jointPosPost.row(i) = angles.transpose();
  }
  for (size_t i = 0; i < knots.rows(); ++i)
    knots[i] = knotsInitialValue;
  auto cb2 = CubicSpline<Scalar>(
    chainSize,
    jointPosPost,
    knots,
    this->cycleTime,
    jointBoundVels);
  cb2.setup();
  cbopt = CbOptimizer<Scalar>(this->motionModule, this->kickLeg, this->supportLeg, &cb2);
  if (addZmpConstraint) {
    vector<bool> activeChains(toUType(LinkChains::count), false);
    activeChains[toUType(this->kickLeg)] = true;
    auto zmpConstraint =
      boost::shared_ptr<ZmpConstraint<Scalar> >(
        new ZmpConstraint<Scalar>(
          this->motionModule, this->supportLeg, JointStateType::sim, activeChains));
    cbopt.addConstraint(zmpConstraint);
  }
  if (addTorqueConstraint) {
    vector<bool> tActiveJoints(toUType(Joints::count), false);
    for (size_t i = 0; i < kickChain->size; ++i)
      activeJoints[kickChain->start+i] = true;
    auto torqueConstraint =
      boost::shared_ptr<TorqueConstraint<Scalar> >(
        new TorqueConstraint<Scalar>(
          this->motionModule, this->supportLeg, JointStateType::sim, tActiveJoints)
      );
    cbopt.addConstraint(torqueConstraint);
  }
  cbopt.optDef();
  if (this->config->logData)
    cbopt.logConstraints(10, kickTimeToImpact, this->logsDirPath, false);
  cb2.evaluateSpline(jointTrajectories2, trajTime, 0);
  for (int i = 0; i < jointTrajectories.size(); ++i) {
    jointTrajectories[i].insert(
      jointTrajectories[i].end(),
      jointTrajectories2[i].begin(),
      jointTrajectories2[i].end());
  }
  /*for (int i = 0; i < jointTrajectories[0].size(); ++i) {
    LOG_INFO(jointTrajectories[0][i] << " " <<
            jointTrajectories[1][i] << " " <<
            jointTrajectories[2][i] << " " <<
            jointTrajectories[3][i] << " " <<
            jointTrajectories[4][i] << " " <<
            jointTrajectories[5][i])
  }*/
  if (this->config->logData) {
    Json::Value jsonPlanning, jsonTraj;
    JSON_ASSIGN(jsonPlanning, "torsoToSupport", JsonUtils::MatrixToJson(this->torsoToSupport));
    JSON_ASSIGN(jsonPlanning, "supportToKick", JsonUtils::MatrixToJson(this->supportToKick));
    JSON_ASSIGN(jsonPlanning, "endEffectorTransformed", JsonUtils::MatrixToJson(eeTrans));
    Json::Value jsonCPoses;
    Json::Value jsonJointPos;
    jsonJointPos.append(JsonUtils::MatrixToJson(jointPosPre));
    jsonJointPos.append(JsonUtils::MatrixToJson(jointPosPost));
    for (int i = 0; i < cPosesPre.size(); ++i) {
      jsonCPoses.append(JsonUtils::MatrixToJson(cPosesPre[i]));
    }
    for (int i = 0; i < cPosesPost.size(); ++i) {
      jsonCPoses.append(JsonUtils::MatrixToJson(cPosesPost[i]));
    }
    JSON_ASSIGN(jsonPlanning, "cPoses", jsonCPoses);
    JSON_ASSIGN(jsonTraj, "jointPoses", jsonJointPos);
    JSON_ASSIGN(jsonTraj, "opt1knots", JsonUtils::MatrixToJson(cb1.getKnots()));
    JSON_ASSIGN(jsonTraj, "opt2knots", JsonUtils::MatrixToJson(cb2.getKnots()));
    JSON_ASSIGN(jsonTraj, "jointBoundVels", JsonUtils::MatrixToJson(jointBoundVels));
    JSON_ASSIGN(jsonTraj, "kickTimeToImpact", kickTimeToImpact);
    JSON_ASSIGN(jsonTraj, "totalKickTime", kickTimeToImpact + trajTime.back());

    Json::Value jsonEE;
    auto linkChain = this->kM->getLinkChain(this->kickLeg);
    unsigned chainSize = linkChain->size;
    Matrix<Scalar, 3, 1> eePrevPos;
    eePrevPos.setZero();
    for (size_t j = 0; j < jointTrajectories[0].size(); ++j) {
      Matrix<Scalar, Dynamic, 1> joints(chainSize);
      for (size_t i = 0; i < jointTrajectories.size(); ++i) {
        joints[i] = jointTrajectories[i][j];
      }
      this->kM->setChainPositions(this->kickLeg, joints, JointStateType::sim); // impact pose joints
      auto stA =
        this->supportToTorso *
        this->kM->getForwardEffector(
          this->kickLeg, toUType(LegEEs::ankle), JointStateType::sim);
      auto eeTrans = stA * this->endEffector;
      JSON_APPEND(
        jsonEE,
        "endEffectorCmd",
        JsonUtils::MatrixToJson(eeTrans)
      );
      Matrix<Scalar, 3, 1> eePos = Matrix<Scalar, 3, 1>(eeTrans(0, 3), eeTrans(1, 3), eeTrans(2, 3));
      Matrix<Scalar, 3, 1> eeVel = (eePos - eePrevPos) / this->cycleTime;
      Matrix<Scalar, 6, 1> eeTraj;
      eeTraj << eePos, eeVel;
      JSON_APPEND(
        jsonEE,
        "endEffectorTraj",
        JsonUtils::MatrixToJson(eeTraj)
      );
      eePrevPos = eePos;
    }
    JSON_ASSIGN(this->dataLogger->getRoot(), "planning", jsonPlanning);
    JSON_ASSIGN(this->dataLogger->getRoot(), "trajectory", jsonTraj);
    JSON_ASSIGN(this->dataLogger->getRoot(), "endEffector", jsonEE);
  }
}

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void JointSpaceKick<Scalar>::requestExecution(const bool& addArmsMovement)
{
  if (addArmsMovement) {
    //! Make center of mass task
    auto comState = this->kM->getComStateWrtFrame(this->supportLeg, toUType(LegEEs::footCenter));
    auto comResidual = vector<bool>(3, true);
    comResidual[2] = false;
    vector<boost::shared_ptr<MotionTask<Scalar> > > tasks;
    auto comTask =
      this->kM->makeComTask(
        static_cast<RobotFeet>(this->supportLeg), LegEEs::footCenter, comState.position, this->armsTaskJoints, 1.0, 0.85, comResidual);
    boost::static_pointer_cast<ComTask<Scalar> >(comTask)->setFirstStep(false);
    tasks.push_back(comTask);
    //auto postureTask =
    //  this->kM->makePostureTask(
    //    this->kM->getJointPositions(), this->armsTaskJoints, 1e-4, 0.5);
    //tasks.push_back(postureTask);
    //Matrix<Scalar, Dynamic, 1> desJoints;
    //desJoints.resize(toUType(Joints::count));
    //desJoints.setZero();
    int timeStep = 1;
    auto kickChain = this->kM->getLinkChain(this->kickLeg);
    //for (size_t i = 0; i < kickChain->size; ++i) {
    //  desJoints[kickChain->start + i] = this->jointTrajectories[i][timeStep-1];
    //}
    //auto kickTask = this->kM->makePostureTask(desJoints, this->kickTaskJoints, 1.0, 1.0);
    //tasks.push_back(kickTask);
    vector<unsigned> jointIds;
    vector<bool> activeJoints(toUType(Joints::count), false);
    for (size_t i = 0; i < toUType(Joints::count); ++i) {
      if (this->armsTaskJoints[i] || this->kickTaskJoints[i]) {
        jointIds.push_back(i);
      }
    }
    activeJoints = this->armsTaskJoints;
    TaskIkSolver<Scalar> tis =
      TaskIkSolver<Scalar>(
        this->motionModule->getKinematicsModule(), 1, activeJoints, true, this->cycleTime);
    tis.init();
    AL::ALValue jointTimes;
    AL::ALValue jointPositions;
    jointTimes.clear();
    jointPositions.clear();
    jointTimes.arraySetSize(jointIds.size());
    jointPositions.arraySetSize(jointIds.size());
    while(true) {
      if (timeStep > this->jointTrajectories[0].size())
        break;
      tis.reset(true);
      //for (size_t i = 0; i < kickChain->size; ++i) {
      //  desJoints[kickChain->start + i] = this->jointTrajectories[i][timeStep-1];
      //}
      //cout << "desJoints: "<< desJoints.transpose() << endl;
      //kickTask->setTargetPosture(desJoints);
      for (auto& task : tasks) {
        if (task) tis.addTask(task);
      }
      //Matrix<Scalar, Dynamic, 1> joints = tis.solve(1);
      Matrix<Scalar, Dynamic, 1> simJoints =
        this->kM->getJointPositions(Joints::first, toUType(Joints::count), JointStateType::sim);
      for (size_t i = 0; i < kickChain->size; ++i) {
        //joints[kickChain->start + i] = this->jointTrajectories[i][timeStep-1];
        simJoints[kickChain->start + i] = this->jointTrajectories[i][timeStep-1];
      }
      this->kM->setJointPositions(Joints::first, simJoints, JointStateType::sim);
      tis.solve(10);
      simJoints = this->kM->getJointPositions(Joints::first, toUType(Joints::count), JointStateType::sim);
      //auto simComState = this->kM->computeComWrtBase(this->supportLeg, toUType(LegEEs::footCenter), JointStateType::sim);
      //cout << "simComState:" << simComState.transpose() << endl;
      for (size_t i = 0; i < jointIds.size(); ++i) {
        jointPositions[i].arrayPush(simJoints[jointIds[i]]);
        jointTimes[i].arrayPush(timeStep * this->cycleTime);
      }
      timeStep++;
    }
    this->totalTimeToKick = (timeStep-1) *  this->cycleTime;
    this->naoqiJointInterpolation(jointIds, jointTimes, jointPositions, true);
  } else {
    auto linkChain = this->kM->getLinkChain(this->kickLeg);
    unsigned chainStart = linkChain->start;
    unsigned chainSize = linkChain->size;
    vector<unsigned> jointIds;
    for (size_t i = chainStart; i < chainStart + chainSize; ++i)
      jointIds.push_back(i);
    AL::ALValue jointTimes;
    AL::ALValue jointPositions;
    jointTimes.clear();
    jointPositions.clear();
    jointTimes.arraySetSize(chainSize);
    jointPositions.arraySetSize(chainSize);
    unsigned trajStep = 0;
    while (true) {
      for (int i = 0; i < chainSize; ++i) {
        jointPositions[i].arrayPush(jointTrajectories[i][trajStep]);
        jointTimes[i].arrayPush((trajStep + 1) * this->cycleTime);
      }
      ++trajStep;
      if (trajStep == jointTrajectories[0].size()) break;
    }
    this->totalTimeToKick = (trajStep + 1) * this->cycleTime;
    //this->kickTimeStep = 0.f;
    this->naoqiJointInterpolation(jointIds, jointTimes, jointPositions, true);
    //plotJointTrajectories();
  }
}
#endif

/*template <typename Scalar>
void JointSpaceKick<Scalar>::logJointStatesActual()
{
  auto js = this->kM->getJointPositions();
  Json::Value& root = this->dataLogger->getRoot();
  JSON_APPEND(
    root["jointStates"],
    "jointStatesActual",
    JsonUtils::MatrixToJson(js->position)
  );
}
*/

template <typename Scalar>
void JointSpaceKick<Scalar>::logEndEffectorActual()
{
  this->setTransformFrames(JointStateType::actual);
  Json::Value& root = this->dataLogger->getRoot();
  Matrix<Scalar, 4, 4> supportToEE =
    this->supportToKick * this->endEffector;
  JSON_APPEND(
    root["endEffector"],
    "actualTrans",
    JsonUtils::MatrixToJson(supportToEE)
  );
  Matrix<Scalar, 3, 1> eePos = Matrix<Scalar, 3, 1>(supportToEE(0, 3), supportToEE(1, 3), supportToEE(2, 3));
  static Matrix<Scalar, 3, 1> eePrevPos = eePos;
  Matrix<Scalar, 3, 1> eeVel = (eePos - eePrevPos) / this->cycleTime;
  Matrix<Scalar, 6, 1> eeTraj;
  eeTraj << eePos, eeVel;
  JSON_APPEND(
    root["endEffector"],
    "actualTraj",
    JsonUtils::MatrixToJson(eeTraj)
  );
  eePrevPos = eePos;
}

template <typename Scalar>
void JointSpaceKick<Scalar>::plotKick()
{
  PlotEnv<Scalar>
    plotEnv(
      "JointSpaceKick", "x", "y", "z",
      Matrix<Scalar, 2, 1>(-0.05, 0.1),
      Matrix<Scalar, 2, 1>(-.5, .5),
      Matrix<Scalar, 2, 1>(-.5, .5)
    );
  LOG_INFO("Plotting kick parameters...")
  Scalar centerSpacing = fabsf(this->supportToKick(1, 3) / 2);
  Scalar offset = this->supportLeg == LinkChains::lLeg ? centerSpacing : -centerSpacing;
  plotEnv.setSphere(Matrix<Scalar, 3, 1>(-(this->ballPosition[1] + offset), this->ballPosition[0], this->ballRadius), this->ballRadius);
  for (const auto& pose : cPosesPre) {
    plotEnv.plotPoint(Matrix<Scalar, 3, 1>(-(pose(1, 3) + offset), pose(0, 3), pose(2, 3)));
  }
  for (const auto& pose : cPosesPost) {
    plotEnv.plotPoint(Matrix<Scalar, 3, 1>(-(pose(1, 3) + offset), pose(0, 3), pose(2, 3)));
  }

  std::ostringstream cmdstr;
  string contourLog = ConfigManager::getLogsDirPath() + string("KickModule/FootCurveLeftXY.txt");
  cmdstr << "replot '" << contourLog << "' using 2:1:3 with lines title '' lc rgb 'red'";
  plotEnv.setCmd(cmdstr);
  cmdstr.str(std::string());

  contourLog = ConfigManager::getLogsDirPath() + string("KickModule/FootCurveRightXY.txt");
  cmdstr << "replot '" << contourLog << "' using 2:1:3 with lines title '' lc rgb 'red'";
  plotEnv.setCmd(cmdstr);
  cmdstr.str(std::string());
  if (this->supportLeg == LinkChains::rLeg) {
    plotEnv.setFrame(
      Matrix<Scalar, 3, 1>(centerSpacing, 0.0, 0.0),
      Matrix<Scalar, 3, 1>(0.0, 0.0, 0.0),
      0.05
    );
  } else {
    plotEnv.setFrame(
      Matrix<Scalar, 3, 1>(-centerSpacing, 0.0, 0.0),
      Matrix<Scalar, 3, 1>(0.0, 0.0, 0.0),
      0.05
    );
  }
  vector<Scalar> eeX;
  vector<Scalar> eeY;
  vector<Scalar> eeZ;
  auto linkChain = this->kM->getLinkChain(this->kickLeg);
  unsigned chainSize = linkChain->size;
  for (size_t j = 0; j < jointTrajectories[0].size(); ++j) {
    Matrix<Scalar, Dynamic, 1> joints(chainSize);
    for (size_t i = 0; i < jointTrajectories.size(); ++i) {
      joints[i] = jointTrajectories[i][j];
    }
    this->kM->setChainPositions(this->kickLeg, joints, JointStateType::sim); // impact pose joints
    auto stA =
      this->supportToTorso *
      this->kM->getForwardEffector(this->kickLeg, toUType(LegEEs::ankle), JointStateType::sim);
    auto eeTrans = stA * this->endEffector;
    eeX.push_back(-(eeTrans(1,3) + offset)); // Plotting env has x = -y
    eeY.push_back(eeTrans(0,3)); // Plotting env has y = x
    eeZ.push_back(eeTrans(2,3));
  }
  plotEnv.plot3D("End-Effector Position", eeX, eeY, eeZ);
}

template class JointSpaceKick<MType>;
