/**
 * @file MotionModule/BallThrow/Types/WBBallThrow.h
 *
 * This file implements the class WBBallThrow
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/BallThrow/Types/WBBallThrow.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/TrajectoryPlanner/CbOptimizer.h"
#include "MotionModule/include/TrajectoryPlanner/ZmpConstraint.h"
#include "MotionModule/include/TrajectoryPlanner/TorqueConstraint.h"
#include "BehaviorConfigs/include/MBConfigs/MBBallThrowConfig.h"
#include "Utils/include/Splines/CubicSpline.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/JsonUtils.h"
#include "Utils/include/JsonLogger.h"

template <typename Scalar>
WBBallThrow<Scalar>::WBBallThrow(
  MotionModule* motionModule,
  const boost::shared_ptr<WBBallThrowConfig>& config) :
  BallThrow<Scalar>(motionModule, config, "WBBallThrow")
{
  DEFINE_FSM_STATE(WBBallThrow<Scalar>, GrabBall, grabBall);
  DEFINE_FSM_STATE(WBBallThrow<Scalar>, Retract, retract);
  DEFINE_FSM_STATE(WBBallThrow<Scalar>, ThrowBall, throwBall);
  DEFINE_FSM(fsm, WBBallThrow<Scalar>, grabBall);
}

template <typename Scalar>
bool WBBallThrow<Scalar>::initiate()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  try {
    LOG_INFO("WBBallThrow.initiate() called...")
    if (!fsm->state) {
      throw
      BehaviorException(
        this,
        "Invalid start fsm state requested.",
        true
      );
    }
    return true;
  } catch (BehaviorException& e) {
    LOG_EXCEPTION(e.what())
    return false;
  }
  #else
  LOG_ERROR("Behavior WBBallThrow undefined without Naoqi")
  return false;
  #endif
}

template <typename Scalar>
void WBBallThrow<Scalar>::update()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  //LOG_INFO("NavigationTestSuite::update() called...")
  if (this->config->logData) {
    auto& root = this->dataLogger->getRoot();
    Matrix<Scalar, 4, 4> tl = this->kM->getForwardEffector(LinkChains::lArm, 0);
    Matrix<Scalar, 4, 4> tr = this->kM->getForwardEffector(LinkChains::rArm, 0);
    JSON_APPEND(root["eeTraj"], "lArm", JsonUtils::MatrixToJson(tl.block(0, 3, 3, 1)));
    JSON_APPEND(root["eeTraj"], "rArm", JsonUtils::MatrixToJson(tr.block(0, 3, 3, 1)));
    JSON_APPEND(root["eeTraj"], "time", this->motionModule->getModuleTime());
  }
  if (fsm->update())
    finish();
  #endif
}

template <typename Scalar>
void WBBallThrow<Scalar>::GrabBall::onStart()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  this->bPtr->openHand(RobotHands::lHand);
  this->bPtr->openHand(RobotHands::rHand);
  #endif
  Scalar circleMidTheta = 25.0 * M_PI / 180.0;
  auto chainSize = this->bPtr->kM->getLinkChain(LinkChains::lArm)->size;
  vector<vector<Scalar> > jointTrajectories;
  jointTrajectories.resize(chainSize);
  for (int i = 0; i < jointTrajectories.size(); ++i)
    jointTrajectories[i].resize(1);
  jointTrajectories[0][0] =  60.0 * M_PI / 180.0 - circleMidTheta;
  jointTrajectories[1][0] =  0.0 * M_PI / 180.0,
  jointTrajectories[2][0] = -44.0 * M_PI / 180.0,
  jointTrajectories[3][0] = -55.0 * M_PI / 180.0,
  jointTrajectories[4][0] = 0.0 * M_PI / 180.0;
  this->bPtr->executeArmsTrajs(jointTrajectories, this->bPtr->timeToGrab);
  this->bPtr->execTime = 0.0;
}

template <typename Scalar>
void WBBallThrow<Scalar>::GrabBall::onRun()
{
  if (this->bPtr->execTime > this->bPtr->timeToGrab) {
    this->nextState = this->bPtr->retract.get();
  } else {
    this->bPtr->execTime += this->bPtr->cycleTime;
  }
}

template <typename Scalar>
void WBBallThrow<Scalar>::Retract::onStart()
{
  auto chainStart =
    this->bPtr->kM->getLinkChain(LinkChains::lArm)->start;
  auto chainSize =
    this->bPtr->kM->getLinkChain(LinkChains::lArm)->size;
  auto nPoses = 2;
  Matrix<Scalar, Dynamic, 1> knots;
  Matrix<Scalar, Dynamic, Dynamic> desiredJoints;
  Matrix<Scalar, Dynamic, Dynamic> desiredBoundVels;
  desiredJoints.resize(nPoses, chainSize);
  desiredBoundVels.resize(2, chainSize);
  desiredBoundVels.setZero();
  Scalar circleMidTheta = 25.0 * M_PI / 180.0;
  desiredJoints(0, 0) =  60.0 * M_PI / 180.0 - circleMidTheta;
  desiredJoints(0, 1) =  0.0 * M_PI / 180.0,
  desiredJoints(0, 2) = -44.0 * M_PI / 180.0,
  desiredJoints(0, 3) = -55.0 * M_PI / 180.0,
  desiredJoints(0, 4) = 0.0 * M_PI / 180.0;
  desiredJoints(1, 0) =  -118 * M_PI / 180.0;
  desiredJoints(1, 1) =  0.0 * M_PI / 180.0,
  desiredJoints(1, 2) = -44.0 * M_PI / 180.0,
  desiredJoints(1, 3) = -55.0 * M_PI / 180.0,
  desiredJoints(1, 4) = 0.0 * M_PI / 180.0;
  knots.resize(desiredJoints.rows() - 1);
  for (int i = 0; i < knots.size(); ++i)
    knots[i] = this->bPtr->timeToRetract;
  vector < vector<Scalar> > jointTrajectories;
  auto cb1 =
    CubicSpline<Scalar>(
      chainSize,
      desiredJoints,
      knots,
      this->bPtr->cycleTime,
      desiredBoundVels
    );
  cb1.setup();
  vector<Scalar> trajTime;
  cb1.evaluateSpline(jointTrajectories, trajTime, 0);
  this->bPtr->executeArmsTrajs(jointTrajectories, this->bPtr->cycleTime);
  this->bPtr->execTime = 0.0;
}

template <typename Scalar>
void WBBallThrow<Scalar>::Retract::onRun()
{
  if (this->bPtr->execTime > this->bPtr->timeToRetract) {
    this->nextState = this->bPtr->throwBall.get();
  } else {
    this->bPtr->execTime += this->bPtr->cycleTime;
  }
}

template <typename Scalar>
void WBBallThrow<Scalar>::ThrowBall::onStart()
{
  auto chainStart = this->bPtr->kM->getLinkChain(LinkChains::lArm)->start;
  auto chainSize = this->bPtr->kM->getLinkChain(LinkChains::lArm)->size;
  auto nPoses = 3;
  Matrix<Scalar, Dynamic, 1> knots;
  Matrix<Scalar, Dynamic, Dynamic> desiredJoints;
  Matrix<Scalar, Dynamic, Dynamic> desiredBoundVels;
  desiredJoints.resize(nPoses, chainSize);
  desiredBoundVels.resize(2, chainSize);
  desiredBoundVels.setZero();

  Matrix<Scalar, 3, 1> desThrowVel;
  desThrowVel[0] =
    this->bPtr->getBehaviorCast()->throwVelocity *
    cos(this->bPtr->getBehaviorCast()->throwAngle * MathsUtils::DEG_TO_RAD);
  desThrowVel[1] =
    this->bPtr->getBehaviorCast()->throwVelocity *
    sin(this->bPtr->getBehaviorCast()->throwAngle * MathsUtils::DEG_TO_RAD);
  desThrowVel[2] = 0.0;
  Matrix<Scalar, Dynamic, 1> desCartesianVel;
  desCartesianVel.resize(6);
  desCartesianVel.setZero();
  desCartesianVel.block(0, 0, 3, 1) = desThrowVel; // x-y-z velocity

  Scalar circleMidTheta = 25.0 * M_PI / 180.0;
  desiredJoints(0, 0) =  -118 * M_PI / 180.0;
  desiredJoints(0, 1) =  0 * M_PI / 180.0,
  desiredJoints(0, 2) = -44.0 * M_PI / 180.0,
  desiredJoints(0, 3) = -55.0 * M_PI / 180.0,
  desiredJoints(0, 4) = 0.0 * M_PI / 180.0;
  desiredJoints(1, 0) =  circleMidTheta - 100.0 * M_PI / 180.0;
  desiredJoints(1, 1) =  0.0 * M_PI / 180.0,
  desiredJoints(1, 2) = -44.0 * M_PI / 180.0,
  desiredJoints(1, 3) = -55.0 * M_PI / 180.0,
  desiredJoints(1, 4) = 0.0 * M_PI / 180.0;
  desiredJoints(2, 0) =  circleMidTheta - 90.0 * M_PI / 180.0;
  desiredJoints(2, 1) =  10.0 * M_PI / 180.0,
  desiredJoints(2, 2) = -44.0 * M_PI / 180.0,
  desiredJoints(2, 3) = -55.0 * M_PI / 180.0,
  desiredJoints(2, 4) = 0.0 * M_PI / 180.0;

  //! Cartesian velocities in torso frame.
  //cout << "desCartesianVel: " << desCartesianVel << endl;
  this->bPtr->kM->setChainPositions(LinkChains::lArm, desiredJoints.row(2), JointStateType::sim); // third row
  Matrix<Scalar, Dynamic, Dynamic> jacobian =
    this->bPtr->kM->computeLimbJ(LinkChains::lArm, 0, JointStateType::sim).block(0, 0, 3, 5);
  //cout << "computeLimbJ : " << jacobian <<endl;
  jacobian.col(1).setZero();
  jacobian.col(2).setZero();
  jacobian.col(3).setZero();
  jacobian.col(4).setZero();
  Matrix<Scalar, Dynamic, 1> preThrowJVel = MathsUtils::pseudoInverseSolve(
    jacobian,
    Matrix<Scalar, Dynamic, 1>(desCartesianVel.block(0, 0, 3, 1)));
  //cout << "preThrowJVel: " << preThrowJVel << endl;
  desiredBoundVels.row(1) = preThrowJVel.transpose(); // Second row
  knots.resize(desiredJoints.rows() - 1);
  for (int i = 0; i < knots.size(); ++i)
    knots[i] = 0.5;
  vector < vector<Scalar> > jointTrajectories;
  auto cb1 =
    CubicSpline<Scalar>(
      chainSize,
      desiredJoints,
      knots,
      this->bPtr->cycleTime,
      desiredBoundVels
    );
  cb1.setup();
  auto cbopt = CbOptimizer<Scalar>(this->bPtr->motionModule, LinkChains::lArm, LinkChains::lLeg, &cb1);
  vector<bool> activeChains(toUType(LinkChains::count), false);
  activeChains[toUType(LinkChains::lArm)] = true;
  activeChains[toUType(LinkChains::rArm)] = true;
  boost::shared_ptr<ZmpConstraint<Scalar> > zmpConstraint =
    boost::make_shared<ZmpConstraint<Scalar>>(
      this->bPtr->motionModule, LinkChains::lLeg, JointStateType::sim, activeChains);
  auto minMaxX = Matrix<Scalar, 2, 1>(-0.02, 0.05);
  auto minMaxY = Matrix<Scalar, 2, 1>(-0.1, 0.0);
  zmpConstraint->setMinMaxX(minMaxX);
  zmpConstraint->setMinMaxY(minMaxY);
  vector<bool> activeJoints(toUType(Joints::count), false);
  for (size_t i = 2; i < 12; ++i)
    activeJoints[i] = true;
  boost::shared_ptr<TorqueConstraint<Scalar>> torqueConstraint =
    boost::make_shared<TorqueConstraint<Scalar>>(
      this->bPtr->motionModule, LinkChains::lLeg, JointStateType::sim, activeJoints);
  cbopt.addConstraint(zmpConstraint);
  //cbopt.addConstraint(torqueConstraint);
  cbopt.optDef();
  vector<Scalar> trajTime;
  cb1.evaluateSpline(jointTrajectories, trajTime, 0);
  this->bPtr->timeToThrow = trajTime.back();
  this->bPtr->executeArmsTrajs(jointTrajectories, this->bPtr->cycleTime);
  this->bPtr->execTime = 0.0;
}

template <typename Scalar>
void WBBallThrow<Scalar>::ThrowBall::onRun()
{
  if (this->bPtr->execTime > this->bPtr->timeToThrow) {
    this->nextState = nullptr;
  } else {
    this->bPtr->execTime += this->bPtr->cycleTime;
  }
}

template <typename Scalar>
void WBBallThrow<Scalar>::executeArmsTrajs(
  const vector<vector<Scalar> >& jointTrajectories,
  const Scalar& stepTime)
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  auto size = jointTrajectories.size() * 2;
  AL::ALValue jointTimes;
  AL::ALValue jointPositions;
  jointTimes.clear();
  jointPositions.clear();
  jointTimes.arraySetSize(size);
  jointPositions.arraySetSize(size);
  auto totalSteps = jointTrajectories[0].size();
  vector<unsigned> jointIds;
  for (int i = 0; i < size; ++i) {
    jointIds.push_back(toUType(Joints::lShoulderPitch) + i);
    jointPositions[i].arraySetSize(totalSteps);
    jointTimes[i].arraySetSize(totalSteps);
  }
  for (int j = 0; j < totalSteps; ++j) {
    for (int i = 0; i < size; ++i) {
      if (i >= 5) {
        if (i >= 6) jointPositions[i][j] = -jointTrajectories[i - 5][j];
        else jointPositions[i][j] = jointTrajectories[i - 5][j];
      } else {
        jointPositions[i][j] = jointTrajectories[i][j];
      }
      jointTimes[i][j] = (j + 1) * stepTime;
    }
  }
  this->execTime = 0.0;
  this->naoqiJointInterpolation(jointIds, jointTimes, jointPositions, true);
  #endif
}

template <typename Scalar>
void WBBallThrow<Scalar>::finish()
{
  this->inBehavior = false;
}

template <typename Scalar>
boost::shared_ptr<WBBallThrowConfig> WBBallThrow<Scalar>::getBehaviorCast()
{
  return SPC(WBBallThrowConfig, this->config);
}

template class WBBallThrow<MType>;
