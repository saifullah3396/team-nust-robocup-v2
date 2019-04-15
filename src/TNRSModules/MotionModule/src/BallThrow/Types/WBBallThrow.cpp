/**
 * @file MotionModule/src/BallThrow/Types/WBBallThrow.cpp
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
#include "Utils/include/Constants.h"
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
}

template <typename Scalar>
void WBBallThrow<Scalar>::update()
{
  //LOG_INFO("NavigationTestSuite::update() called...")
  if (this->config->logData) {
    auto& root = this->dataLogger->getRoot();
    Matrix<Scalar, 4, 4> tl = this->kM->getForwardEffector(LinkChains::lArm, 0);
    Matrix<Scalar, 4, 4> tr = this->kM->getForwardEffector(LinkChains::rArm, 0);
    JSON_APPEND(root["eeTraj"], "lArm", JsonUtils::matrixToJson(tl.block(0, 3, 3, 1)));
    JSON_APPEND(root["eeTraj"], "rArm", JsonUtils::matrixToJson(tr.block(0, 3, 3, 1)));
    JSON_APPEND(root["eeTraj"], "time", this->motionModule->getModuleTime());
  }
  if (fsm->update())
    finish();
}

template <typename Scalar>
void WBBallThrow<Scalar>::GrabBall::onStart()
{
  this->bPtr->openHand(RobotHands::lHand);
  this->bPtr->openHand(RobotHands::rHand);
  Scalar circleMidTheta = 25.0 * MathsUtils::DEG_TO_RAD;
  auto chainSize = this->bPtr->kM->getLinkChain(LinkChains::lArm)->size;
  this->bPtr->jointTrajectories.resize(chainSize);
  for (int i = 0; i < this->bPtr->jointTrajectories.size(); ++i)
    this->bPtr->jointTrajectories[i].resize(1);
  this->bPtr->jointTrajectories[0][0] =  60.0 * MathsUtils::DEG_TO_RAD - circleMidTheta;
  this->bPtr->jointTrajectories[1][0] =  0.0 * MathsUtils::DEG_TO_RAD,
  this->bPtr->jointTrajectories[2][0] = -44.0 * MathsUtils::DEG_TO_RAD,
  this->bPtr->jointTrajectories[3][0] = -55.0 * MathsUtils::DEG_TO_RAD,
  this->bPtr->jointTrajectories[4][0] = 0.0 * MathsUtils::DEG_TO_RAD;
#ifdef NAOQI_MOTION_PROXY_AVAILABLE
  this->bPtr->executeArmsTrajs(this->bPtr->jointTrajectories, this->bPtr->timeToGrab);
#else
  this->bPtr->jointsI = this->bPtr->kM->getJointPositions();
  this->bPtr->jointsDelta.resize(toUType(Joints::count));
  for (size_t i = 0; i < toUType(Joints::count); ++i) {
    if (i >= toUType(HardwareIds::lArmStart) && i < toUType(HardwareIds::lArmStart) + toUType(HardwareIds::nLArm)) {
      this->bPtr->jointsDelta[i] = this->bPtr->jointTrajectories[i-toUType(HardwareIds::lArmStart)][0] - this->bPtr->jointsI[i];
    } else if (i >= toUType(HardwareIds::rArmStart) && i < toUType(HardwareIds::rArmStart) + toUType(HardwareIds::nRArm)) {
      if (i >= toUType(HardwareIds::rArmStart) + 1)
        this->bPtr->jointsDelta[i] = -this->bPtr->jointTrajectories[i-toUType(HardwareIds::rArmStart)][0] - this->bPtr->jointsI[i];
      else
        this->bPtr->jointsDelta[i] = this->bPtr->jointTrajectories[i-toUType(HardwareIds::rArmStart)][0] - this->bPtr->jointsI[i];
    } else {
      this->bPtr->jointsDelta[i] = NAN;
    }
  }
#endif
  this->bPtr->execTime = 0.0;
}

template <typename Scalar>
void WBBallThrow<Scalar>::GrabBall::onRun()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    if (this->bPtr->execTime > this->bPtr->timeToGrab) {
      this->nextState = this->bPtr->retract.get();
    } else {
      this->bPtr->execTime += this->bPtr->cycleTime;
    }
  #else
    if (this->bPtr->execTime > this->bPtr->timeToGrab) {
      this->nextState = this->bPtr->retract.get();
    } else {
      auto step = this->bPtr->execTime / this->bPtr->timeToGrab;
      this->bPtr->setJointCmds(this->bPtr->interpolate(step));
      this->bPtr->execTime += this->bPtr->cycleTime;
    }
  #endif
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
  Scalar circleMidTheta = 25.0 * MathsUtils::DEG_TO_RAD;
  desiredJoints(0, 0) =  60.0 * MathsUtils::DEG_TO_RAD - circleMidTheta;
  desiredJoints(0, 1) =  0.0 * MathsUtils::DEG_TO_RAD,
  desiredJoints(0, 2) = -44.0 * MathsUtils::DEG_TO_RAD,
  desiredJoints(0, 3) = -55.0 * MathsUtils::DEG_TO_RAD,
  desiredJoints(0, 4) = 0.0 * MathsUtils::DEG_TO_RAD;
  desiredJoints(1, 0) =  -119 * MathsUtils::DEG_TO_RAD;
  desiredJoints(1, 1) =  12.5 * MathsUtils::DEG_TO_RAD,
  desiredJoints(1, 2) = -40.0 * MathsUtils::DEG_TO_RAD,
  desiredJoints(1, 3) = -75.0 * MathsUtils::DEG_TO_RAD,
  desiredJoints(1, 4) = 29 * MathsUtils::DEG_TO_RAD;
  knots.resize(desiredJoints.rows() - 1);
  for (int i = 0; i < knots.size(); ++i)
    knots[i] = this->bPtr->timeToRetract;
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
  cb1.evaluateSpline(this->bPtr->jointTrajectories, trajTime, 0);
#ifdef NAOQI_MOTION_PROXY_AVAILABLE
  this->bPtr->executeArmsTrajs(this->bPtr->jointTrajectories, this->bPtr->cycleTime);
#endif
  this->bPtr->execTime = 0.0;
}

template <typename Scalar>
void WBBallThrow<Scalar>::Retract::onRun()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    if (this->bPtr->execTime > this->bPtr->timeToRetract) {
      this->nextState = this->bPtr->throwBall.get();
    } else {
      this->bPtr->execTime += this->bPtr->cycleTime;
    }
  #else
    static unsigned step = 0;
    if (this->bPtr->execTime > this->bPtr->timeToRetract) {
      step = 0;
      this->nextState = this->bPtr->throwBall.get();
    } else {
      Matrix<Scalar, Dynamic, 1> jointCmds(toUType(Joints::count));
      for (size_t i = 0; i < toUType(Joints::count); ++i) {
        if (i >= toUType(HardwareIds::lArmStart) && i < toUType(HardwareIds::lArmStart) + toUType(HardwareIds::nLArm)) {
          jointCmds[i] = this->bPtr->jointTrajectories[i-toUType(HardwareIds::lArmStart)][step];
        } else if (i >= toUType(HardwareIds::rArmStart) && i < toUType(HardwareIds::rArmStart) + toUType(HardwareIds::nRArm)) {
          if (i >= toUType(HardwareIds::rArmStart) + 1)
            jointCmds[i] = -this->bPtr->jointTrajectories[i-toUType(HardwareIds::rArmStart)][step];
          else
            jointCmds[i] = this->bPtr->jointTrajectories[i-toUType(HardwareIds::rArmStart)][step];
        } else {
          jointCmds[i] = NAN;
        }
      }
      this->bPtr->setJointCmds(jointCmds);
      this->bPtr->execTime += this->bPtr->cycleTime;
      step++;
      if (step >= this->bPtr->jointTrajectories[0].size()) {
        step = 0;
        this->nextState = this->bPtr->throwBall.get();
      }
    }
  #endif
}

template <typename Scalar>
void WBBallThrow<Scalar>::ThrowBall::onStart()
{
  auto chainStart = this->bPtr->kM->getLinkChain(LinkChains::lArm)->start;
  auto chainSize = this->bPtr->kM->getLinkChain(LinkChains::lArm)->size;
  auto nPoses = 2;
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

  //Scalar circleMidTheta = 25.0 * MathsUtils::DEG_TO_RAD;
  desiredJoints(0, 0) = -119 * MathsUtils::DEG_TO_RAD;
  desiredJoints(0, 1) = 12.5 * MathsUtils::DEG_TO_RAD;
  desiredJoints(0, 2) = -40.0 * MathsUtils::DEG_TO_RAD;
  desiredJoints(0, 3) = -75.0 * MathsUtils::DEG_TO_RAD;
  desiredJoints(0, 4) = 29 * MathsUtils::DEG_TO_RAD;
  desiredJoints(1, 0) = -119 * MathsUtils::DEG_TO_RAD;
  desiredJoints(1, 1) = 12.5 * MathsUtils::DEG_TO_RAD;
  desiredJoints(1, 2) = 0.0 * MathsUtils::DEG_TO_RAD; // Ball leaves the hands at 0 degrees
  desiredJoints(1, 3) = -75.0 * MathsUtils::DEG_TO_RAD;
  desiredJoints(1, 4) = 29 * MathsUtils::DEG_TO_RAD;
  /*desiredJoints(2, 0) =  circleMidTheta - 90.0 * MathsUtils::DEG_TO_RAD;
  desiredJoints(2, 1) =  10.0 * MathsUtils::DEG_TO_RAD,
  desiredJoints(2, 2) = -44.0 * MathsUtils::DEG_TO_RAD,
  desiredJoints(2, 3) = -55.0 * MathsUtils::DEG_TO_RAD,
  desiredJoints(2, 4) = 0.0 * MathsUtils::DEG_TO_RAD;*/

  ///< Cartesian velocities in torso frame.
  // Find jacobian for when ball leaves
  this->bPtr->kM->setChainPositions(LinkChains::lArm, desiredJoints.row(2), JointStateType::sim); // third row
  Matrix<Scalar, Dynamic, Dynamic> jacobian =
    this->bPtr->kM->computeLimbJ(LinkChains::lArm, 0, JointStateType::sim).block(0, 0, 3, 5);
  // Set jacobian dependance only on elbow yaw
  jacobian.col(0).setZero();
  jacobian.col(1).setZero();
  //jacobian.col(2).setZero();
  jacobian.col(3).setZero();
  jacobian.col(4).setZero();

  Matrix<Scalar, Dynamic, 1> preThrowJVel = MathsUtils::pseudoInverseSolve(
    jacobian,
    Matrix<Scalar, Dynamic, 1>(desCartesianVel.block(0, 0, 3, 1)));
  //desiredBoundVels.row(1) = preThrowJVel.transpose(); // Second row

  ///< Set elbow yaw final position to 40.0 degrees
  desiredJoints(1, 2) = 40.0 * MathsUtils::DEG_TO_RAD;
  knots.resize(desiredJoints.rows() - 1);
  for (int i = 0; i < knots.size(); ++i)
    knots[i] = 0.3;
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
    boost::shared_ptr<ZmpConstraint<Scalar>>(new ZmpConstraint<Scalar>(
      this->bPtr->motionModule, LinkChains::lLeg, JointStateType::sim, activeChains));
  auto minMaxX = Matrix<Scalar, 2, 1>(-0.02, 0.05);
  auto minMaxY = Matrix<Scalar, 2, 1>(-0.1, 0.0);
  zmpConstraint->setMinMaxX(minMaxX);
  zmpConstraint->setMinMaxY(minMaxY);
  vector<bool> activeJoints(toUType(Joints::count), false);
  for (size_t i = 2; i < 12; ++i)
    activeJoints[i] = true;
  boost::shared_ptr<TorqueConstraint<Scalar>> torqueConstraint =
    boost::shared_ptr<TorqueConstraint<Scalar>>(new TorqueConstraint<Scalar>(
      this->bPtr->motionModule, LinkChains::lLeg, JointStateType::sim, activeJoints));
  //cbopt.addConstraint(zmpConstraint);
  //cbopt.addConstraint(torqueConstraint);
  cbopt.setMaxVelocityCutoff(
    preThrowJVel.norm() > Constants::rElbowYawVelLimit ? 1.0 : preThrowJVel.norm() / Constants::rElbowYawVelLimit);
  cbopt.optDef();
  vector<Scalar> trajTime;
  cb1.evaluateSpline(this->bPtr->jointTrajectories, trajTime, 0);
  cbopt.logConstraints(100, trajTime.back(), this->bPtr->logsDirPath, true);
  this->bPtr->timeToThrow = trajTime.back();
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  this->bPtr->executeArmsTrajs(this->bPtr->jointTrajectories, this->bPtr->cycleTime);
  #endif
  this->bPtr->execTime = 0.0;
}

template <typename Scalar>
void WBBallThrow<Scalar>::ThrowBall::onRun()
{
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    if (this->bPtr->execTime > this->bPtr->timeToThrow) {
      this->nextState = nullptr;
    } else {
      this->bPtr->execTime += this->bPtr->cycleTime;
    }
  #else
    static unsigned step = 0;
    if (this->bPtr->execTime > this->bPtr->timeToThrow) {
      step = 0;
      this->nextState = nullptr;
    } else {
      Matrix<Scalar, Dynamic, 1> jointCmds(toUType(Joints::count));
      for (size_t i = 0; i < toUType(Joints::count); ++i) {
        if (i >= toUType(HardwareIds::lArmStart) && i < toUType(HardwareIds::lArmStart) + toUType(HardwareIds::nLArm)) {
          jointCmds[i] = this->bPtr->jointTrajectories[i-toUType(HardwareIds::lArmStart)][step];
        } else if (i >= toUType(HardwareIds::rArmStart) && i < toUType(HardwareIds::rArmStart) + toUType(HardwareIds::nRArm)) {
          if (i >= toUType(HardwareIds::rArmStart) + 1)
            jointCmds[i] = -this->bPtr->jointTrajectories[i-toUType(HardwareIds::rArmStart)][step];
          else
            jointCmds[i] = this->bPtr->jointTrajectories[i-toUType(HardwareIds::rArmStart)][step];
        } else {
          jointCmds[i] = NAN;
        }
      }
      this->bPtr->setJointCmds(jointCmds);
      this->bPtr->execTime += this->bPtr->cycleTime;
      step++;
      if (step >= this->bPtr->jointTrajectories[0].size()) {
        step = 0;
        this->nextState = nullptr;
      }
    }
  #endif
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
