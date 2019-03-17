/**
 * @file MotionModule/src/KickModule/CSpaceBSplineKick.cpp
 *
 * This file implements the class CSpaceBSplineKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#include "MotionModule/include/KickModule/Types/CSpaceBSplineKick.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MotionLogger.h"
#include "MotionModule/include/BalanceModule/BalanceDefinitions.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/TrajectoryPlanner/ZmpConstraint.h"
#include "MotionModule/include/JointRequest.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "BehaviorConfigs/include/MBConfigs/MBKickConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"
#include "Utils/include/Splines/BSpline.h"
#include "Utils/include/PlotEnv.h"

using namespace Utils;

template <typename Scalar>
CSpaceBSplineKick<Scalar>::CSpaceBSplineKick(
  MotionModule* motionModule,
  const boost::shared_ptr<CSpaceBSplineKickConfig>& config) :
  KickModule<Scalar>(motionModule, config, "CSpaceBSplineKick")
{
}

template <typename Scalar>
bool CSpaceBSplineKick<Scalar>::initiate()
{
  LOG_INFO("CSpaceBSplineKick.initiate() called...")
  try {
    setupKickBase();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    return false;
  }
  if (this->getBehaviorCast()->postureConfig)
    this->behaviorState = this->posture;
  else if (this->getBehaviorCast()->balanceConfig)
    this->behaviorState = this->balance;
  else
    this->behaviorState = this->kick;
  return true;
}

template <typename Scalar>
void CSpaceBSplineKick<Scalar>::update()
{
  static bool kickSetup = false;
  static unsigned kickStep = 0;
  if (this->behaviorState == this->posture) {
    //LOG_INFO("behaviorState: posture")
    if (this->lastChildCfg &&
        this->lastChildCfg->id == (unsigned)MBIds::posture)
    {
      this->behaviorState = this->balance;
    } else {
      this->setupPosture();
    }
  } else if (this->behaviorState == this->balance) {
    //LOG_INFO("behaviorState: balance")
    if (this->lastChildCfg &&
        this->lastChildCfg->id == (unsigned)MBIds::balance)
    {
      static float wait = 0.f;
      if (wait > 1.0) {
        //this->killChild();
        this->behaviorState = this->kick;
      } else {
        wait += this->cycleTime;
      }
      if (!this->getChild()) { // Child finished
        this->behaviorState = this->kick;
      }
    } else {
      this->setupBalance();
    }
  } else if (this->behaviorState == this->kick) {
    //LOG_INFO("behaviorState: kick")
    if (this->getBehaviorCast()->balanceConfig->type == (int)MBBalanceTypes::zmpControl) {
      if (!kickSetup) {
        if (!this->kickFailed) {
          auto zmpConfig = boost::make_shared <ZmpControlConfig> ();
          zmpConfig->supportLeg = this->supportLeg;
          zmpConfig->keepOtherLegContact = false;
          zmpConfig->regularizePosture = true;
          zmpConfig->keepTorsoUpright = false;
          zmpConfig->activeJoints = vector<unsigned>(activeJoints.begin(), activeJoints.end());
          //for (size_t i = 2; i < 12; ++i)
          //  zmpConfig->activeJoints[i] = true;
          this->getBehaviorCast()->balanceConfig = zmpConfig;
          this->setupBalance();
        } else {
          this->behaviorState = this->postKickPosture;
        }
        kickSetup = true;
      } else {
        Matrix<Scalar, 4, 4> targetPose;
        targetPose.setIdentity();
        targetPose.block(0, 3, 3, 1) = cartTraj.block(kickStep, 0, 1, 3).transpose();
        for (int i = 12; i < 18; ++i)
          activeJoints[i] = false;
        auto kickTask =
          this->kM->makeCartesianTask(this->kickLeg, this->endEffector, targetPose, activeJoints, 1.0, 0.95);
        this->addMotionTask(kickTask);
        //this->logEndEffectorActual();
        //this->logJointPositionsActual();
        kickStep++;
        if (kickStep >= this->cartTraj.rows()) {
          kickStep = 0;
          this->killChild();
          this->behaviorState = this->postKickPosture;
        }
      }
    }
  } else if (this->behaviorState == this->postKickPosture) {
    POSTURE_STATE_OUT(MotionModule) = PostureState::unknown;
    kickSetup = false;
    //finish();
    //return;
    if (this->lastChildCfg &&
      this->lastChildCfg->id == (unsigned)MBIds::posture)
    {
      finish();
    } else {
      this->setupPosture();
    }
  }
}

template <typename Scalar>
void CSpaceBSplineKick<Scalar>::finish()
{
  LOG_INFO("CSpaceBSplineKick.finish() called...")
  this->inBehavior = false;
}

template <typename Scalar>
CSpaceBSplineKickConfigPtr CSpaceBSplineKick<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <CSpaceBSplineKickConfig> (this->config);
}

template <typename Scalar>
void CSpaceBSplineKick<Scalar>::setupKickBase()
{
  LOG_INFO("CSpaceBSplineKick.setupKickBase()")
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
  if (reqVel.x != -1.f || target.x != -1.f) {
    this->ballPosition = Matrix<Scalar, 3, 1>(ball.x, ball.y, this->ballRadius);
    if (target.x != -1.f) { // if target is defined use this
      this->targetPosition = Matrix<Scalar, 3, 1>(target.x, target.y, this->ballRadius);
      auto ballToTarget = this->targetPosition - this->ballPosition;
      this->targetDistance = ballToTarget.norm();
      this->ballToTargetUnit = ballToTarget / this->targetDistance;
      this->targetAngle = atan2(this->ballToTargetUnit[1], this->ballToTargetUnit[0]);
      this->desImpactVelKnown = false;
    } else if (reqVel.x != -1) { // if des velocity is defined do this
      this->desImpactVel = Matrix<Scalar, 3, 1>(reqVel.x, reqVel.y, 0.f);
      auto velMag = norm(reqVel);
      this->ballToTargetUnit = Matrix<Scalar, 3, 1>(reqVel.x / velMag, reqVel.y / velMag, 0.f);
      this->targetAngle = atan2(reqVel.y, reqVel.x);
      this->desImpactVelKnown = true;
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
    this->setTransformFrames(JointStateType::actual);
    if (this->config->logData) {
      Json::Value jsonSetup;
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
    Matrix<Scalar, Dynamic, 1> balanceJoints(toUType(Joints::count));
    if (this->supportLeg == LinkChains::lLeg) {
      balanceJoints = Matrix<Scalar, Dynamic, 1>::Map(
        &balanceDefs[0][0],
        sizeof(balanceDefs[0]) / sizeof(balanceDefs[0][0]));
    } else {
      balanceJoints = Matrix<Scalar, Dynamic, 1>::Map(
        &balanceDefs[1][0],
        sizeof(balanceDefs[1]) / sizeof(balanceDefs[1][0]));
    }
    this->kM->setJointPositions(Joints::first, balanceJoints, JointStateType::sim);
    this->setTransformFrames(JointStateType::sim);
    this->solveForImpact();
    defineTrajectory();
    activeJoints = vector<bool>(toUType(Joints::count), false);
    auto kickChain = this->kM->getLinkChain(this->kickLeg);
    auto supportChain = this->kM->getLinkChain(this->supportLeg);
    for (size_t i = 1; i < supportChain->size; ++i)
      activeJoints[supportChain->start + i] = true;
    for (size_t i = 1; i < kickChain->size; ++i)
      activeJoints[kickChain->start + i] = true;
  } else {
    throw BehaviorException(
      this,
      "Required kick parameters 'ball', 'reqVel' or 'target are not well-defined",
      false
    );
  }
}

template <typename Scalar>
void CSpaceBSplineKick<Scalar>::solveForImpact()
{
  this->impactPose.setIdentity();
  this->impactPose(0, 3) = this->ballPosition[0] - this->ballToTargetUnit[0] * this->ballRadius * 1.5;
  this->impactPose(1, 3) = this->ballPosition[1] - this->ballToTargetUnit[1] * this->ballRadius * 1.5;
  this->impactPose(2, 3) = this->ballPosition[2];
}

template <typename Scalar>
void CSpaceBSplineKick<Scalar>::defineTrajectory()
{
  Matrix<Scalar, 4, 4> retractionPose;
  retractionPose.setIdentity();
  auto retRadius = 0.03;
  auto retHeight = 0.0;
  auto angle = 0.0;
  retractionPose.block(0, 3, 3, 1) = Matrix<Scalar, 3, 1>(-retRadius * cos(angle), -retRadius * sin(angle), retHeight);
  auto eeTrans = this->supportToKick * this->endEffector;
  auto retTrans = eeTrans * retractionPose;

  auto chainSize = this->kM->getLinkChain(this->kickLeg)->size;
  Matrix<Scalar, Dynamic, Dynamic> impactJoints;
  auto impactPoseT = this->torsoToSupport * this->impactPose;
  vector<bool> activeJoints = vector<bool>(chainSize, true);
  activeJoints[0] = false;
  Matrix<Scalar, Dynamic, 1> angles;
  angles = this->kM->solveJacobianIK(this->kickLeg, this->endEffector, impactPoseT, 50, JointStateType::sim, true, 5e-4, 0.5, activeJoints);
  if (angles[0] != angles[0]) {
      cout << "Requested kick cannot be performed." << endl;
      this->kickFailed = true;
      return;
  }
  impactJoints = angles.transpose();

  Scalar kickTime = 1.0;
  auto nControlPoints = 11;
  auto bSplineDegree = 3;
  auto knotInterval = kickTime / 5;
  Matrix<Scalar, Dynamic, Dynamic> controlPoints;
  Matrix<Scalar, Dynamic, 1> knots;
  Matrix<Scalar, 1, 3> initialVelocity, finalVelocity;
  initialVelocity.setZero();

  //! Setting up pre-impact trajectory.
  //! Required cartesian velocities at initial and final poses
  finalVelocity.resize(3); //x-y-z, r-p-y
  if (!this->desImpactVelKnown) {
    this->computeDesImpactVel(impactJoints);
  }
  this->desImpactVel[0] = std::min((Scalar)this->desImpactVel[0], (Scalar)1.0);
  this->desImpactVel[1] = std::min((Scalar)this->desImpactVel[1], (Scalar)1.0);
  finalVelocity = this->desImpactVel.transpose(); // x-y-z velocity
  //! Cartesian velocities in torso frame.
  finalVelocity.block(0, 0, 3, 1) =
    this->torsoToSupport.block(0, 0, 3, 3) * finalVelocity.segment(0, 3);

  controlPoints.resize(nControlPoints, 3);
  controlPoints.block(0, 0, 1, 3) = eeTrans.block(0, 3, 3, 1).transpose();
  controlPoints.block(3, 0, 1, 3) = retTrans.block(0, 3, 3, 1).transpose();
  controlPoints.block(6, 0, 1, 3) = this->impactPose.block(0, 3, 3, 1).transpose();
  controlPoints.block(9, 0, 1, 3) = eeTrans.block(0, 3, 3, 1).transpose();
  controlPoints.block(10, 0, 1, 3) = eeTrans.block(0, 3, 3, 1).transpose();
  //controlPoints(9, 2) += 0.01;
  //controlPoints(10, 2) += 0.01;

  controlPoints.block(1, 0, 1, 3) = controlPoints.block(0, 0, 1, 3) + knotInterval / bSplineDegree * initialVelocity;
  controlPoints.block(2, 0, 1, 3) = 0.5 * (controlPoints.block(1, 0, 1, 3) + controlPoints.block(3, 0, 1, 3));
  controlPoints.block(5, 0, 1, 3) = controlPoints.block(6, 0, 1, 3) - knotInterval / bSplineDegree * finalVelocity;
  controlPoints.block(4, 0, 1, 3) = 0.5 * (controlPoints.block(3, 0, 1, 3) + controlPoints.block(5, 0, 1, 3));

  controlPoints.block(7, 0, 1, 3) = controlPoints.block(6, 0, 1, 3) + knotInterval / bSplineDegree * finalVelocity;
  //controlPoints.block(8, 0, 1, 3) = 0.5 * (controlPoints.block(9, 0, 1, 3) + controlPoints.block(7, 0, 1, 3));
  controlPoints.block(8, 0, 1, 3) = controlPoints.block(7, 0, 1, 3) + knotInterval / bSplineDegree * finalVelocity;

  auto nKnots = nControlPoints + bSplineDegree + 1;
  knots.resize(nKnots);
  for (size_t i = 0; i < 4; ++i)
    knots[i] = 0.0;
  for (size_t i = 4; i < 7; ++i)
    knots[i] = knots[i-1] + knotInterval;
  for (size_t i = 7; i < 11; ++i)
    knots[i] = knots[6] + knotInterval;
  for (size_t i = 11; i < 15; ++i)
    knots[i] = knots[10] + knotInterval * 3;
  cout << "Knots vector:\n" << knots.transpose() << endl;
  cout << "controlPoints:\n" << controlPoints << endl;
  bSpline =
    boost::make_shared<BSpline<Scalar> >(
      bSplineDegree, 3, controlPoints, knots, this->cycleTime);
  bSpline->setup();
  cartTraj = bSpline->getSpline(0);
  /*auto cTraj = bSpline->getSpline(0);
  cout << "cTraj: " << cTraj << endl;

  //! Plotting
  GnuPlotEnv::PlotEnv<Scalar>
    plotEnv(
      "CSpaceBSplineKick", "x", "y", "z",
      Matrix<Scalar, 2, 1>(-0.2, 0.2),
      Matrix<Scalar, 2, 1>(-.5, .5),
      Matrix<Scalar, 2, 1>(-.5, .5)
    );
  cout << "cTraj: " << cTraj.rows() << endl;
  plotEnv.plot3D(cTraj.col(0), cTraj.col(1), cTraj.col(2));*/

  /*if (this->logData) {
    Json::Value jsonPlanning, jsonTraj;
    JSON_ASSIGN(jsonPlanning, "torsoToSupport", JsonUtils::matrixToJson(this->torsoToSupport));
    JSON_ASSIGN(jsonPlanning, "supportToKick", JsonUtils::matrixToJson(this->supportToKick));
    JSON_ASSIGN(jsonPlanning, "endEffectorTransformed", JsonUtils::matrixToJson(eeTrans));
    Json::Value jsoncPoses;
    Json::Value jsoncPosesT;
    for (int i = 0; i < cPoses.size(); ++i) {
      jsoncPoses.append(JsonUtils::matrixToJson(cPoses[i]));
      jsoncPosesT.append(JsonUtils::matrixToJson(cPosesT[i]));
    }
    JSON_ASSIGN(jsonPlanning, "cPoses", jsoncPoses);
    JSON_ASSIGN(jsonPlanning, "cPosesT", jsoncPosesT);
    JSON_ASSIGN(jsonTraj, "optknots", JsonUtils::matrixToJson(cb1.getKnots()));
    JSON_ASSIGN(jsonTraj, "jointPoses", JsonUtils::matrixToJson(jointPos));
    JSON_ASSIGN(jsonTraj, "jointBoundVels", JsonUtils::matrixToJson(jointBoundVels));
    JSON_ASSIGN(jsonTraj, "kickTimeToImpact", kickTimeToImpact);
    JSON_ASSIGN(jsonTraj, "totalKickTime", kickTimeToImpact + trajTime.back());

    Json::Value jsonEE;
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
        this->kM->getForwardEffector(
          this->kickLeg, ANKLE, JointStateType::sim);
      auto eeTrans = stA * this->endEffector;
      JSON_APPEND(
        jsonEE,
        "endEffectorCmd",
        JsonUtils::matrixToJson(eeTrans)
      );
    }
    JSON_ASSIGN(this->dataLogger->getRoot(), "planning", jsonPlanning);
    JSON_ASSIGN(this->dataLogger->getRoot(), "trajectory", jsonTraj);
    JSON_ASSIGN(this->dataLogger->getRoot(), "endEffector", jsonEE);
  }*/
}

template class CSpaceBSplineKick<MType>;
