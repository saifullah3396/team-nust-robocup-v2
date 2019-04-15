/**
 * @file MotionModule/src/MovementModule/Types/KinResolutionWalk.cpp
 *
 * This file implements the class KinResolutionWalk
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "BehaviorManager/include/StateMachine.h"
#include "MotionModule/include/BalanceModule/ZmpPreviewController.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/ComState.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/KinematicsModule/TorsoState.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MovementModule/Types/KinResolutionWalk.h"
#include "MotionModule/include/MovementModule/WalkParameters.h"
#include "MotionModule/include/MovementModule/WalkZmpRefGen.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"
#include "Utils/include/DataHolders/TNRSFootstep.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/VisionUtils.h"
#include "Utils/include/Splines/BSpline.h"
#include "Utils/include/PlotEnv.h"

template<typename Scalar>
Scalar KinResolutionWalk<Scalar>::maxHipLOffset;
template<typename Scalar>
Scalar KinResolutionWalk<Scalar>::maxHipROffset;

template <typename Scalar>
KinResolutionWalk<Scalar>::KinResolutionWalk(
  MotionModule* motionModule,
  const boost::shared_ptr<KinResolutionWalkConfig>& config) :
  MovementModule<Scalar>(motionModule, config, "KinResolutionWalk")
{
  params = boost::shared_ptr<WalkParameters<Scalar> >(new WalkParameters<Scalar>());
  params->loadConfig();
  walkZmpRefGen =
    boost::shared_ptr<WalkZmpRefGen<Scalar> >(new WalkZmpRefGen<Scalar>(
      this->motionModule,
      this->kM->getGlobalBaseIndex(),
      params->nPreviews,
      &stepsQueue));
}
template <typename Scalar>
KinResolutionWalk<Scalar>::~KinResolutionWalk()
{
  for (auto& c :  controllers)
    delete c;
}

template <typename Scalar>
boost::shared_ptr<KinResolutionWalkConfig> KinResolutionWalk<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <KinResolutionWalkConfig> (this->config);
}

template <typename Scalar>
bool KinResolutionWalk<Scalar>::initiate()
{
  ///< Set robotInMotion memory parameter
  ROBOT_IN_MOTION_OUT(MotionModule) = false;

  /// Set global feet orientation to identity
  globalFeetOrientation.setIdentity();

  ///< Set global base
  RobotFeet globalBase = RobotFeet::lFoot;
  currentBase = globalBase;

  ///< Set the global base to be the left foot
  this->kM->setGlobalBase(globalBase, LegEEs::footCenter);

  ///< Set other leg as right foot
  auto otherLeg = globalBase == RobotFeet::lFoot ? RobotFeet::rFoot : RobotFeet::lFoot;

  ///< Set normalized walk velocity between -1 and 1
  getBehaviorCast()->velocityInput.clip(-1, 1);

  ///< Generate first balance shift step
  addFirstStep();

  ///< Set walk zmp reference offsets
  walkZmpRefGen->setRefOffset(getBehaviorCast()->refOffset);

  ///< Fill the preview window
  while (!walkZmpRefGen->previewsAvailable(this->runTime)) {
    genNextStep();
  }

  //for(int i = 0; i  <10; ++i)
  //  genNextStep();
  //drawSteps();

  ///< Initiate the Zmp reference generator for walk
  if (!walkZmpRefGen->initiate())
    return false;

  ///< Set current joint estimates to simulated states
  this->kM->setStateFromTo(JointStateType::actual, JointStateType::sim);

  ///< Initiate the preview controllers for x-y directions
  controllers.resize(2);
  for (int i = 0; i < controllers.size(); ++i) {
    controllers[i] = new ZmpPreviewController<Scalar>(this->kM->getComModel(i));
    controllers[i]->setPreviewLength(params->nPreviews);
    controllers[i]->initController();
  }

  params->comHeight = this->kM->getComStateWrtFrame(
    static_cast<LinkChains>(this->kM->getGlobalBaseIndex()), toUType(LegEEs::footCenter)).position[2];

  ///< Set up center of mass and zmp reference logs
  comLog.open(
    (ConfigManager::getLogsDirPath() + string("KinResolutionWalk/Com.txt")).c_str(),
    std::ofstream::out | std::ofstream::trunc
  );
  comLog << "# X Y" << endl;
  comLog.close();
  zmpRegLog.open(
    (ConfigManager::getLogsDirPath() + string("KinResolutionWalk/zmpRef.txt")).c_str(),
    std::ofstream::out | std::ofstream::trunc
  );
  zmpRegLog << "# X Y" << endl;
  zmpRegLog.close();
  return true;
}

template <typename Scalar>
void KinResolutionWalk<Scalar>::reinitiate(const BehaviorConfigPtr& cfg)
{
  this->config = cfg;
}

template <typename Scalar>
void KinResolutionWalk<Scalar>::update()
{
  static Matrix<Scalar, 4, 4> totalTransform = Matrix<Scalar, 4, 4>::Identity();
  static int nSteps = 0;
  getBehaviorCast()->velocityInput.clip(-1, 1);
  ///< If the curent total number of steps are not enough
  ///< to fill the previewable zmp references add another step
  ///< relative to the latest step available in queue
  while (!walkZmpRefGen->previewsAvailable(this->runTime)) {
    genNextStep();
  }

  ///< Move the zmp references one step forward
  walkZmpRefGen->update(this->runTime);

  for (const auto& fs : stepsQueue) {
    if (fabsf(this->runTime - fs->timeAtFinish) <= 1e-3) {
      ///< Last step is finished so change base support reference frame to
      ///< The last step foot which is at stepsQueue.front()
      totalTransform = totalTransform * stepsQueue.front()->trans;
      currentBase = stepsQueue.front()->foot;

      this->kM->setGlobalBase(currentBase, LegEEs::footCenter);

      ///< Retransform the zmp references to the new support foot frame
      walkZmpRefGen->retransform(this->runTime);
      nSteps++;

      Matrix<Scalar, 4, 4> trans = this->kM->getGlobalToOther();
      Matrix<Scalar, 3, 1> cp, cv, ca;
      Matrix<Scalar, 3, 1> xState = controllers[0]->getTrueState();
      Matrix<Scalar, 3, 1> yState = controllers[1]->getTrueState();
      cp[0] = xState[0];
      cv[0] = xState[1];
      ca[0] = xState[2];
      cp[1] = yState[0];
      cv[1] = yState[1];
      ca[1] = yState[2];
      cp[2] = params->comHeight;
      cv[2] = 0.0;
      ca[2] = 0.0;

      cp = MathsUtils::transformVector(trans, cp);
      cv = trans.block(0, 0, 3, 3) * cv;
      ca = trans.block(0, 0, 3, 3) * ca;
      xState[0] = cp[0];
      xState[1] = cv[0];
      xState[2] = ca[0];
      yState[0] = cp[1];
      yState[1] = cv[1];
      yState[2] = ca[1];
      controllers[0]->setTrueState(xState);
      controllers[1]->setTrueState(yState);

      ///< Remove the last step from steps sequence
      popStep();

      ///< Create the trajectory for the next step relative to new support
      ///< foot frame
      this->genStepTrajectory();
      if (nSteps >= getBehaviorCast()->maxNSteps) {
        nSteps = 0;
        finish();
      }
      break;
    }
  }

  auto& zmpRef = walkZmpRefGen->getCurrentRef();
  Matrix<Scalar, 3, 1> desComPosition;
  desComPosition[0] =
    controllers[0]->step(
      Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1> >(
        zmpRef->x.linearize(), zmpRef->x.size()))[0];
  desComPosition[1] =
    controllers[1]->step(
      Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1> >(
        zmpRef->y.linearize(), zmpRef->y.size()))[0];
  desComPosition[2] = params->comHeight;
  Matrix<Scalar, 3, 1> currentComPosition =
    this->kM->getComStateWrtFrame(static_cast<LinkChains>(currentBase), toUType(LegEEs::footCenter)).position;
  Matrix<Scalar, 3, 1> desComVelocity = (desComPosition - currentComPosition) / this->cycleTime;

  // Define desired velocities for each joint of limbs other than support limb
  vector<Matrix<Scalar, Dynamic, 1>> limbVelocitiesD(toUType(LinkChains::count));
  // Zero velocity in joint space
  limbVelocitiesD[toUType(LinkChains::head)] = Matrix<Scalar, 2, 1>::Zero();
  // Zero velocity in joint space
  limbVelocitiesD[toUType(LinkChains::lArm)] = Matrix<Scalar, 5, 1>::Zero();
  // Zero velocity in joint space
  limbVelocitiesD[toUType(LinkChains::rArm)] = Matrix<Scalar, 5, 1>::Zero();
  // Zero velocity in cartesian space (meaning for end-effector pose)
  limbVelocitiesD[toUType(LinkChains::lLeg)] = Matrix<Scalar, 6, 1>::Zero();
  limbVelocitiesD[toUType(LinkChains::rLeg)] = Matrix<Scalar, 6, 1>::Zero();

  if (stepTraj.rows() > 0) {
    Matrix<Scalar, 6, 1> currStepConfig = MathsUtils::matToVector(this->kM->getGlobalToOther());
    Matrix<Scalar, 6, 1> desStepConfig = stepTraj.row(stepTrajIndex).transpose();
    limbVelocitiesD[toUType(currentBase == RobotFeet::lFoot ? RobotFeet::rFoot : RobotFeet::lFoot)]
      = (desStepConfig - currStepConfig) / this->cycleTime;
  }

  vector<unsigned> limbMotionSpace(toUType(LinkChains::count));
  limbMotionSpace[toUType(LinkChains::head)] = 0; // Joint space
  limbMotionSpace[toUType(LinkChains::lArm)] = 0; // Joint space
  limbMotionSpace[toUType(LinkChains::rArm)] = 0; // Joint space
  limbMotionSpace[toUType(LinkChains::lLeg)] = 1; // Cartesian space
  limbMotionSpace[toUType(LinkChains::rLeg)] = 1; // Cartesian space
  vector<int> eeIndices(toUType(LinkChains::count));
  eeIndices[toUType(LinkChains::head)] = 0; // default
  eeIndices[toUType(LinkChains::lArm)] = 0; // default
  eeIndices[toUType(LinkChains::rArm)] = 0; // default
  eeIndices[toUType(LinkChains::lLeg)] = toUType(LegEEs::footCenter);
  eeIndices[toUType(LinkChains::rLeg)] = toUType(LegEEs::footCenter);
  Matrix<Scalar, 6, 1> desComVelocity6 = Matrix<Scalar, 6, 1>::Zero();
  desComVelocity6.head(3) = desComVelocity;
  Matrix<Scalar, 4, 4> torsoPose = this->kM->getGlobalToBody();

  Matrix<Scalar, 4, 4> torsoTarget = Matrix<Scalar, 4, 4>::Identity();
  /*if (stepsQueue.front()->foot == RobotFeet::lFoot)
    MathsUtils::makeRotationXYZ(torsoTarget, -this->kM->getJointPosition(Joints::lAnkleRoll), 0.0, 0.0);
  else
    MathsUtils::makeRotationXYZ(torsoTarget, -this->kM->getJointPosition(Joints::rAnkleRoll), 0.0, 0.0);*/
  desComVelocity6.tail(3) = MathsUtils::getOrientationDiff(torsoPose, torsoTarget) / this->cycleTime;
  Matrix<Scalar, Dynamic, 1> cmdJoints =
    this->kM->solveComIK(
      static_cast<LinkChains>(currentBase),
      desComVelocity6,
      limbMotionSpace,
      limbVelocitiesD,
      eeIndices,
      JointStateType::actual
    );
  if (stepTraj.rows() > 0) {
    if (getBehaviorCast()->addHipCompensation && nSteps > 0) {
      ///< Goes from zero to hipOffset and then back to zero
      if (currentBase == RobotFeet::lFoot) {
        cmdJoints[toUType(Joints::lHipRoll)] +=
          maxHipLOffset * sin(M_PI * (stepTrajIndex + 1) * this->cycleTime / params->totalStepTime);
        LOG_INFO("loffset:" << maxHipLOffset * sin(M_PI * (stepTrajIndex + 1) * this->cycleTime / params->totalStepTime));
      } else {
        LOG_INFO("cmdJoints[toUType(Joints::rHipRoll)]:" << cmdJoints[toUType(Joints::rHipRoll)] * 180 / 3.14);
        cmdJoints[toUType(Joints::rHipRoll)] -=
          maxHipROffset * sin(M_PI * (stepTrajIndex + 1) * this->cycleTime / params->totalStepTime);
        LOG_INFO("roffset:" << -maxHipLOffset * sin(M_PI * (stepTrajIndex + 1) * this->cycleTime / params->totalStepTime));
        LOG_INFO("cmdJoints[toUType(Joints::rHipRoll)]:" << cmdJoints[toUType(Joints::rHipRoll)] * 180 / 3.14);
      }
    }
    stepTrajIndex++;
  }
  for (int i = 0; i < 12; ++i)
    cmdJoints[i] = NAN;
  this->setJointCmds(cmdJoints);
  Matrix<Scalar, 3, 1> zmpRefT;
  zmpRefT << zmpRef->x[0], zmpRef->y[0], 0.0;
  zmpRefT = MathsUtils::transformVector(totalTransform, zmpRefT);
  zmpRegLog.open(
    (ConfigManager::getLogsDirPath() + string("KinResolutionWalk/zmpRef.txt")).c_str(),
    std::ofstream::out | std::ofstream::app
  );
  zmpRegLog << zmpRefT[0] << " " << zmpRefT[1] << endl;
  zmpRegLog.close();
  comLog.open(
    (ConfigManager::getLogsDirPath() + string("KinResolutionWalk/Com.txt")).c_str(),
    std::ofstream::out | std::ofstream::app
  );
  auto comState = this->kM->getComStateWrtFrame(
      static_cast<LinkChains>(this->kM->getGlobalBaseIndex()), toUType(LegEEs::footCenter));

  comState.position = MathsUtils::transformVector(totalTransform, comState.position);
  desComPosition = MathsUtils::transformVector(totalTransform, desComPosition);
  Matrix<Scalar, 3, 1> zmpT;
  zmpT << comState.zmp[0], comState.zmp[1], 0.0;
  zmpT = MathsUtils::transformVector(totalTransform, zmpT);
  comLog << this->motionModule->getModuleTime() << "  "
         << comState.position[0] << " "
         << comState.position[1] << " "
         << desComPosition[0] << " "
         << desComPosition[1] << " "
         << zmpT[0] << " "
         << zmpT[1] << endl;
  comLog.close();
}

template <typename Scalar>
void KinResolutionWalk<Scalar>::finish()
{
  LOG_INFO("KinResolutionWalk.finish() called...")
  POSTURE_STATE_OUT(MotionModule) = PostureState::unknown;
  this->inBehavior = false;
}

template<typename Scalar>
void KinResolutionWalk<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG(
      "MotionBehaviors",
      (Scalar, KinResolutionWalk.maxHipLOffset, maxHipLOffset),
      (Scalar, KinResolutionWalk.maxHipROffset, maxHipROffset),
    )
    maxHipLOffset *= MathsUtils::DEG_TO_RAD;
    maxHipROffset *= MathsUtils::DEG_TO_RAD;
    loaded = true;
  }
}

template <typename Scalar>
void KinResolutionWalk<Scalar>::genStepTrajectory()
{
  static const auto nDim = 6;
  static const auto nControlPoints = 6;
  static const auto bSplineDegree = 3;
  static constexpr auto nKnots = nControlPoints + bSplineDegree + 1;
  auto knotInterval = params->totalStepTime / 3;
  Matrix<Scalar, Dynamic, 1> knots;
  knots.resize(nKnots);
  knots.head(4).setConstant(0.0);
  knots.tail(4).setConstant(params->totalStepTime);
  for (size_t i = 4; i < (nKnots - 4); ++i)
    knots[i] = knots[i-1] + knotInterval;

  Matrix<Scalar, Dynamic, Dynamic> controlPoints;
  Matrix<Scalar, 4, 4> supportToSwingFoot = this->kM->getGlobalToOther();
  Matrix<Scalar, 4, 4> toTrans = getFrontStep()->trans;

  //LOG_INFO("global to other:\n" << this->kM->getGlobalToOther());
  globalFeetOrientation = this->kM->getGlobalToOther() * globalFeetOrientation;
  //LOG_INFO("globalFeetOrientation:\n" << globalFeetOrientation);

  toTrans.block(0, 0, 3, 3) = toTrans.block(0, 0, 3, 3) * globalFeetOrientation.block(0, 0, 3, 3);
  controlPoints.resize(nControlPoints, nDim);
  controlPoints.row(0) = MathsUtils::matToVector(supportToSwingFoot).transpose();
  controlPoints.row(1) = controlPoints.row(0);
  controlPoints.row(2) = controlPoints.row(1);
  controlPoints.row(3) = MathsUtils::matToVector(toTrans).transpose();
  controlPoints.row(4) = controlPoints.row(3);
  controlPoints.row(5) = controlPoints.row(4);
  controlPoints(2, 2) = params->stepHeight;
  //controlPoints.block(2, 3, 1, 3) = (controlPoints.block(1, 3, 1, 3) + controlPoints.block(4, 3, 1, 3)) / 2;
  //controlPoints.block(3, 3, 1, 3) = controlPoints.block(2, 3, 1, 3);
  controlPoints(3, 2) = params->stepHeight;

  //LOG_INFO("Control points: " << controPoints);
  boost::shared_ptr<BSpline<Scalar> > bSpline =
    boost::shared_ptr<BSpline<Scalar> >(
      new BSpline<Scalar>(bSplineDegree, nDim, controlPoints, knots, this->cycleTime));
  bSpline->setup();
  stepTraj = bSpline->getSpline(0);
  //LOG_INFO("stepTraj: " << stepTraj)
  ///< Plotting
  /*GnuPlotEnv::PlotEnv<Scalar>
    plotEnv(
      "CSpaceBSplineKick", "x", "y", "z",
      Matrix<Scalar, 2, 1>(-0.2, 0.2),
      Matrix<Scalar, 2, 1>(-.5, .5),
      Matrix<Scalar, 2, 1>(-.5, .5)
    );*/
  stepTrajIndex = 0;
  //plotEnv.plot3D("BSpline", cartTraj.col(0), cartTraj.col(1), cartTraj.col(2));
}

template <typename Scalar>
TNRSFootstep<Scalar> KinResolutionWalk<Scalar>::computeStepFromTo(
  const TNRSFootstep<Scalar>& from,
  const TNRSFootstep<Scalar>& to,
  Matrix<Scalar, 4, 4>& transFromTo)
{
  Matrix<Scalar, 3, 3> toRotation, fromRotation;
  MathsUtils::makeRotationZ(toRotation, to.pose2D.getTheta());
  MathsUtils::makeRotationZ(fromRotation, from.pose2D.getTheta());
  transFromTo =
    MathsUtils::getTInverse(
      MathsUtils::makeTransformation(
        fromRotation, from.pose2D.getX(), from.pose2D.getY(), 0.0)) *
      MathsUtils::makeTransformation(
        toRotation, to.pose2D.getX(), to.pose2D.getY(), 0.0);
  auto stepPose =
    RobotPose2D<Scalar>(
      transFromTo(0, 3),
      transFromTo(1, 3),
      atan2(transFromTo(1, 0), transFromTo(0, 0)) ///< Euler Z angle
    );
  return TNRSFootstep<Scalar>(stepPose, to.foot, transFromTo, params->totalStepTime);
}

template <typename Scalar>
RobotPose2D<Scalar> KinResolutionWalk<Scalar>::stepVelToPose(
  const VelocityInput<Scalar>& vi)
{
  return
    RobotPose2D<Scalar>(
      vi.getX() * params->maxStepLengthX,
      vi.getY() * params->maxStepLengthY,
      vi.getTheta() * params->maxStepRotation);
}

template <typename Scalar>
void KinResolutionWalk<Scalar>::addFirstStep()
{
  auto foot = this->kM->getGlobalBaseIndex() == RobotFeet::lFoot ? RobotFeet::rFoot : RobotFeet::lFoot;
  Matrix<Scalar, 4 ,4> transFromTo = this->kM->getGlobalToOther();
  addStep(boost::shared_ptr<TNRSFootstep<Scalar> >(
    new TNRSFootstep<Scalar>(
      RobotPose2D<Scalar>(
        transFromTo(0, 3),
        transFromTo(1, 3),
        atan2(transFromTo(1, 0), transFromTo(0, 0))),
        foot,
        transFromTo,
        params->firstStepTime)));
}


template <typename Scalar>
void KinResolutionWalk<Scalar>::genNextStep()
{
  auto poseDiff = this->stepVelToPose(getBehaviorCast()->velocityInput);
  Matrix<Scalar, 3, 3> diffRot;
  Matrix<Scalar, 4, 4> diffTrans;
  MathsUtils::makeRotationZ(diffRot, poseDiff.getTheta());
  diffTrans = MathsUtils::makeTransformation(diffRot, poseDiff.getX(), poseDiff.getY(), 0.0);
  //LOG_INFO("diffTrans: " << diffTrans);
  auto prevStep = getBackStep();
  if (!prevStep) { ///< Robot is standing
    ///< Get current foot position
    auto foot = this->kM->getGlobalBaseIndex() == RobotFeet::lFoot ? RobotFeet::rFoot : RobotFeet::lFoot;
    Matrix<Scalar, 4 ,4> transFromTo = this->kM->getGlobalToOther() * diffTrans;
    addStep(boost::shared_ptr<TNRSFootstep<Scalar> >(new TNRSFootstep<Scalar>(
      RobotPose2D<Scalar>(
        transFromTo(0, 3), transFromTo(1, 3), atan2(transFromTo(1, 0), transFromTo(0, 0))),
        foot,
        transFromTo,
        prevStep->timeAtFinish + params->totalStepTime)));
  } else {
    ///< Get current foot position
    auto foot = prevStep->foot == RobotFeet::lFoot ? RobotFeet::rFoot : RobotFeet::lFoot;
    auto pose = RobotPose2D<Scalar>(
      diffTrans(0, 3), diffTrans(1, 3), atan2(diffTrans(1, 0), diffTrans(0, 0)));
    ///< Clip foot if it is going out of max range
    pose.x() = VisionUtils::clip(pose.getX(), params->minStepLengthX, params->maxStepLengthX);
    if (foot == RobotFeet::lFoot) { ///< Theta max range for right foot is inverse of left foot
      pose.theta() = VisionUtils::clip(pose.getTheta(), params->minStepRotation, params->maxStepRotation);
      pose.y() = VisionUtils::clip(pose.getY(), params->minStepLengthY, params->maxStepLengthY);
    } else {
      pose.theta() = VisionUtils::clip(pose.getTheta(), -params->maxStepRotation, params->minStepRotation);
      pose.y() = VisionUtils::clip(pose.getY(), -params->maxStepLengthY, params->minStepLengthY);
    }
    pose.y() += this->params->getFootSeparation(foot);
    Matrix<Scalar, 3, 3> rot;
    MathsUtils::makeRotationZ(rot, pose.getTheta());
    Matrix<Scalar, 4 ,4> transFromTo =
      MathsUtils::makeTransformation(rot, pose.getX(), pose.getY(), 0.0);
    addStep(boost::shared_ptr<TNRSFootstep<Scalar> >(new TNRSFootstep<Scalar>(
        pose,
        foot,
        transFromTo,
        prevStep->timeAtFinish + params->totalStepTime)));
  }
}

template <typename Scalar>
void KinResolutionWalk<Scalar>::drawSteps()
{
  ///< Generate first two steps
  Mat drawing = Mat(Size(500, 500), CV_8UC3);
  cv::Scalar color;
  Matrix<Scalar, 4, 4> trans;
  trans = stepsQueue.front()->trans;
  for (const auto& fs : stepsQueue) {
    color = fs->foot == RobotFeet::lFoot ? cv::Scalar(255,0,0) : cv::Scalar(0,0,255);
    trans *= fs->trans;
    auto transFs = *fs;
    transFs.pose2D = RobotPose2D<Scalar>(trans(0, 3), trans(1, 3), atan2(trans(1, 0), trans(0, 0)));
    VisionUtils::drawRotatedRect(drawing, transFs.getFootRect(Point2f(250, 250), 100), color);
    VisionUtils::displayImage("drawing", drawing, 1.0);
    cv::waitKey(0);
  }
}

template <typename Scalar>
void KinResolutionWalk<Scalar>::addStep(
  const boost::shared_ptr<TNRSFootstep<Scalar> >& fs)
{
  return stepsQueue.push_back(fs);
}

template <typename Scalar>
boost::shared_ptr<TNRSFootstep<Scalar> >& KinResolutionWalk<Scalar>::getFrontStep()
{
  return stepsQueue.front();
}

template <typename Scalar>
boost::shared_ptr<TNRSFootstep<Scalar> >& KinResolutionWalk<Scalar>::getBackStep()
{
  return stepsQueue.back();
}

template <typename Scalar>
void KinResolutionWalk<Scalar>::popStep()
{
  return stepsQueue.pop_front();
}

template class KinResolutionWalk<MType>;
