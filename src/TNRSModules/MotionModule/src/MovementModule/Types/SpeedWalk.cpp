/**
 * @file MotionModule/MovementModule/Types/SpeedWalk.cpp
 *
 * This file implements the class SpeedWalk
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "BehaviorManager/include/StateMachine.h"
#include "MotionModule/include/BalanceModule/ZmpPreviewController.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/ComState.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/KinematicsModule/TaskIkSolver.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MovementModule/Types/SpeedWalk.h"
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
vector<Scalar> SpeedWalk<Scalar>::taskWeights;
template<typename Scalar>
vector<Scalar> SpeedWalk<Scalar>::taskGains;

template <typename Scalar>
SpeedWalk<Scalar>::SpeedWalk(
  MotionModule* motionModule,
  const boost::shared_ptr<SpeedWalkConfig>& config) :
  MovementModule<Scalar>(motionModule, config, "SpeedWalk")
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
SpeedWalk<Scalar>::~SpeedWalk()
{
  for (auto& c :  controllers)
    delete c;
}

template <typename Scalar>
boost::shared_ptr<SpeedWalkConfig> SpeedWalk<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <SpeedWalkConfig> (this->config);
}

template <typename Scalar>
bool SpeedWalk<Scalar>::initiate()
{
  //! Set robotInMotion memory parameter
  ROBOT_IN_MOTION_OUT(MotionModule) = false;

  //! Set global base
  RobotFeet globalBase = RobotFeet::lFoot;

  //! Set the global base to be the left foot
  this->kM->setGlobalBase(globalBase, LegEEs::footCenter);

  //! Set other leg as right foot
  auto otherLeg = globalBase == RobotFeet::lFoot ? RobotFeet::rFoot : RobotFeet::lFoot;

  //! Set normalized walk velocity between -1 and 1
  getBehaviorCast()->velocityInput.clip(-1, 1);

  //! Generate first balance shift step
  addFirstStep();

  //! Fill the preview window
  while (!walkZmpRefGen->previewsAvailable(this->runTime)) {
    genNextStep();
  }

  //for(int i = 0; i  <10; ++i)
  //  genNextStep();
  //drawSteps();

  //! Initiate the Zmp reference generator for walk
  if (!walkZmpRefGen->initiate())
    return false;

  //! Set current joint estimates to simulated states
  this->kM->setStateFromTo(JointStateType::actual, JointStateType::sim);

  //! Initiate the preview controllers for x-y directions
  controllers.resize(2);
  for (int i = 0; i < controllers.size(); ++i) {
    controllers[i] = new ZmpPreviewController<Scalar>(this->kM->getComModel(i));
    controllers[i]->setPreviewLength(params->nPreviews);
    controllers[i]->initController();
  }

  //! Set shoulder pitch roll joint limits so it does not self collide if used
  this->kM->getTaskSolver()->setMinJointLimit(MathsUtils::degToRads(7.0), toUType(Joints::lShoulderRoll));
  this->kM->getTaskSolver()->setMaxJointLimit(MathsUtils::degToRads(12.0), toUType(Joints::lShoulderRoll));
  this->kM->getTaskSolver()->setMaxJointLimit(MathsUtils::degToRads(-7.0), toUType(Joints::rShoulderRoll));
  this->kM->getTaskSolver()->setMinJointLimit(MathsUtils::degToRads(-12.0), toUType(Joints::rShoulderRoll));

  this->kM->getTaskSolver()->setMinJointLimit(MathsUtils::degToRads(85.0), toUType(Joints::lShoulderPitch));
  this->kM->getTaskSolver()->setMaxJointLimit(MathsUtils::degToRads(95.0), toUType(Joints::lShoulderPitch));
  this->kM->getTaskSolver()->setMaxJointLimit(MathsUtils::degToRads(95.0), toUType(Joints::rShoulderPitch));
  this->kM->getTaskSolver()->setMinJointLimit(MathsUtils::degToRads(85.0), toUType(Joints::rShoulderPitch));
  //params->comHeight = this->kM->getComStateWrtFrame(
   // static_cast<LinkChains>(this->kM->getGlobalBaseIndex()), toUType(LegEEs::footBase)).position[2];

  this->kM->getTaskSolver()->setMaxVelocityLimitGain(1.0);
  auto activeJoints = vector<bool>(toUType(Joints::count), false);
  auto baseChain = this->kM->getGlobalBase();
  auto otherChain = this->kM->getLinkChain(static_cast<LinkChains>(otherLeg));
  for (size_t i = baseChain->start; i < baseChain->end; ++i)
    activeJoints[i] = true;
  for (size_t i = otherChain->start; i < otherChain->end; ++i)
    activeJoints[i] = true;
  tasks.resize(toUType(IkTasks::count));

  //! Initiate a step task
  Matrix<Scalar, 4, 4> target = this->kM->getGlobalToOther();
  tasks[toUType(IkTasks::step)] =
    this->kM->makeCartesianTask(
      static_cast<LinkChains>(otherLeg),
      toUType(LegEEs::footCenter),
      target,
      activeJoints,
      taskWeights[toUType(IkTasks::step)],
      taskGains[toUType(IkTasks::step)]
    );

  if (getBehaviorCast()->keepTorsoUpright) {
    //! Torso task to keep torso orientation to the initial orientation
    //! This can be useful if the torso is to be kept upright
    Matrix<Scalar, 4, 4> target = Matrix<Scalar, 4, 4>::Identity();
    auto activeResidual = vector<bool>(6, false); //! X-Y-Z, Roll-Pitch-Yaw
    //! Only use Pitch and Yaw tracking
    activeResidual[3] = true;
    activeResidual[4] = true;
    tasks[toUType(IkTasks::torso)] =
      this->kM->makeTorsoTask(
        globalBase,
        LegEEs::footCenter,
        target,
        activeJoints,
        taskWeights[toUType(IkTasks::torso)],
        taskGains[toUType(IkTasks::torso)],
        activeResidual
      );
  }

  //! Make a center of mass task to control its desired trajectory
  auto comResidual = vector<bool>(3, true);
  //! Initiate a center of mass tracking task
  /*for (size_t i = toUType(HardwareIds::lArmStart); i < toUType(HardwareIds::lArmStart) + 2; ++i)
    activeJoints[i] = true;
  for (size_t i = toUType(HardwareIds::rArmStart); i < toUType(HardwareIds::rArmStart) + 2; ++i)
    activeJoints[i] = true;*/
  tasks[toUType(IkTasks::com)] =
    this->kM->makeComTask(
      globalBase,
      LegEEs::footCenter,
      Matrix<Scalar, 3, 1>(),
      activeJoints,
      taskWeights[toUType(IkTasks::com)],
      taskGains[toUType(IkTasks::com)],
      comResidual
    );

  Matrix<Scalar, Dynamic, 1> joints = this->kM->getJointPositions();
  if (getBehaviorCast()->minimizeJointVels) {
    //! Initiate a task to minimize the joint velocities
    tasks[toUType(IkTasks::minJointVel)] =
      this->kM->makePostureTask(
        joints,
        activeJoints,
        taskWeights[toUType(IkTasks::minJointVel)],
        taskGains[toUType(IkTasks::minJointVel)]
      );
  }
  //! Set up center of mass and zmp reference logs
  comLog.open(
    (ConfigManager::getLogsDirPath() + string("SpeedWalk/Com.txt")).c_str(),
    std::ofstream::out | std::ofstream::trunc
  );
  comLog << "# X Y" << endl;
  comLog.close();
  zmpRegLog.open(
    (ConfigManager::getLogsDirPath() + string("SpeedWalk/zmpRef.txt")).c_str(),
    std::ofstream::out | std::ofstream::trunc
  );
  zmpRegLog << "# X Y" << endl;
  zmpRegLog.close();
  return true;
}

template <typename Scalar>
void SpeedWalk<Scalar>::reinitiate(const BehaviorConfigPtr& cfg)
{
  this->config = cfg;
}

template <typename Scalar>
void SpeedWalk<Scalar>::update()
{
  static Matrix<Scalar, 4, 4> totalTransform = Matrix<Scalar, 4, 4>::Identity();
  static int nSteps = 0;
  getBehaviorCast()->velocityInput.clip(-1, 1);
  //! If the curent total number of steps are not enough
  //! to fill the previewable zmp references add another step
  //! relative to the latest step available in queue
  while (!walkZmpRefGen->previewsAvailable(this->runTime)) {
    genNextStep();
  }

  //! Move the zmp references one step forward
  walkZmpRefGen->update(this->runTime);

  for (const auto& fs : stepsQueue) {
    if (fabsf(this->runTime - fs->timeAtFinish) <= 1e-3) {
      //LOG_INFO("fs->timeAtFinish: " << fs->timeAtFinish);
      //! Last step is finished so change base support reference frame to
      //! The last step foot which is at stepsQueue.front()
      totalTransform = totalTransform * stepsQueue.front()->trans;
      auto newBase = stepsQueue.front()->foot;
      auto other = newBase == RobotFeet::lFoot ? RobotFeet::rFoot : RobotFeet::lFoot;
      this->kM->setGlobalBase(newBase, LegEEs::footCenter);
      boost::static_pointer_cast<CartesianTask<Scalar> >(tasks[static_cast<int>(IkTasks::step)])->setChainIndex(static_cast<LinkChains>(other));
      boost::static_pointer_cast<CartesianTask<Scalar> >(tasks[static_cast<int>(IkTasks::step)])->setEndEffector(
        this->kM->getLinkChain(static_cast<LinkChains>(other))->endEffectors[toUType(LegEEs::footCenter)]);
      boost::static_pointer_cast<ComTask<Scalar> >(tasks[static_cast<int>(IkTasks::com)])->setBaseFrame(newBase);
      if (getBehaviorCast()->keepTorsoUpright) {
        //auto prevTarget = boost::static_pointer_cast<TorsoTask<Scalar> >(tasks[static_cast<int>(IkTasks::torso)])->getTarget();
        //boost::static_pointer_cast<TorsoTask<Scalar> >(tasks[static_cast<int>(IkTasks::torso)])->setTarget(this->kM->getGlobalToOther() * prevTarget);
        Matrix<Scalar, 4, 4> target = Matrix<Scalar, 4, 4>::Identity();
        boost::static_pointer_cast<TorsoTask<Scalar> >(tasks[static_cast<int>(IkTasks::torso)])->setTarget(target);
        boost::static_pointer_cast<TorsoTask<Scalar> >(tasks[static_cast<int>(IkTasks::torso)])->setBaseFrame(newBase);
        boost::static_pointer_cast<TorsoTask<Scalar> >(tasks[static_cast<int>(IkTasks::torso)])->setEndEffector(
          this->kM->getLinkChain(static_cast<LinkChains>(newBase))->endEffectors[toUType(LegEEs::footCenter)]);
      }

      //! Retransform the zmp references to the new support foot frame
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

      //! Remove the last step from steps sequence
      popStep();

      //! Create the trajectory for the next step relative to new support
      //! foot frame
      this->genStepTrajectory();
      if (nSteps >= getBehaviorCast()->maxNSteps) {
        nSteps = 0;
        finish();
      }
      break;
    }
  }

  Matrix<Scalar, 3, 1> desComPosition;
  auto& zmpRef = walkZmpRefGen->getCurrentRef();
  desComPosition[0] =
    controllers[0]->step(
      Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1> >(
        zmpRef->x.linearize(), zmpRef->x.size()))[0];
  desComPosition[1] =
    controllers[1]->step(
      Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1> >(
        zmpRef->y.linearize(), zmpRef->y.size()))[0];
  desComPosition[2] = params->comHeight;
  //Matrix<Scalar, 4, 4> target = this->kM->getGlobalToOther();
  Matrix<Scalar, Dynamic, 1> targetJoints = this->kM->getJointPositions();
  if (getBehaviorCast()->minimizeJointVels) {
    //! Initiate a task to minimize the joint velocities
    boost::static_pointer_cast<PostureTask<Scalar> >(tasks[toUType(IkTasks::minJointVel)])->setTargetPosture(targetJoints);
  }

  if (getBehaviorCast()->keepTorsoUpright) {
    //! Initiate a task to minimize the joint velocities
    Matrix<Scalar, 4, 4> torsoTarget;
    if (stepsQueue.front()->foot == RobotFeet::lFoot)
      MathsUtils::makeRotationXYZ(torsoTarget, -this->kM->getJointPosition(Joints::lAnkleRoll), 0.0, 0.0);
    else
      MathsUtils::makeRotationXYZ(torsoTarget, -this->kM->getJointPosition(Joints::rAnkleRoll), 0.0, 0.0);
    boost::static_pointer_cast<TorsoTask<Scalar> >(tasks[toUType(IkTasks::torso)])->setTarget(torsoTarget);
  }

  if (stepTraj.rows() > 0) {
    Matrix<Scalar, 4, 4> stepTarget = Matrix<Scalar, 4, 4>::Identity();
    Matrix<Scalar, 6, 1> desStepConfig = stepTraj.row(stepTrajIndex).transpose();
    stepTarget.block(0, 3, 3, 1) = desStepConfig.block(0, 0, 3, 1);
    Matrix<Scalar, 3, 1> eulers = desStepConfig.block(3, 0, 3, 1);
    stepTarget.block(0, 0, 3, 3) = MathsUtils::eulerToMat(eulers);
    boost::static_pointer_cast<CartesianTask<Scalar> >(tasks[static_cast<int>(IkTasks::step)])->setTarget(stepTarget);
    stepTrajIndex++;
  }

  boost::static_pointer_cast<ComTask<Scalar> >(tasks[static_cast<int>(IkTasks::com)])->setTargetCom(desComPosition);
  boost::static_pointer_cast<ComTask<Scalar> >(tasks[static_cast<int>(IkTasks::com)])->setFirstStep(true);

  Matrix<Scalar, 3, 1> zmpRefT;
  zmpRefT << zmpRef->x[0], zmpRef->y[0], 0.0;
  zmpRefT = MathsUtils::transformVector(totalTransform, zmpRefT);
  zmpRegLog.open(
    (ConfigManager::getLogsDirPath() + string("SpeedWalk/zmpRef.txt")).c_str(),
    std::ofstream::out | std::ofstream::app
  );
  zmpRegLog << zmpRefT[0] << " " << zmpRefT[1] << endl;
  zmpRegLog.close();
  comLog.open(
    (ConfigManager::getLogsDirPath() + string("SpeedWalk/Com.txt")).c_str(),
    std::ofstream::out | std::ofstream::app
  );
  auto comState = this->kM->getComStateWrtFrame(
      static_cast<LinkChains>(this->kM->getGlobalBaseIndex()), toUType(LegEEs::footBase));

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

  for (auto& task : tasks) {
    if (task) {
      this->addMotionTask(task);
    }
  }
}

template <typename Scalar>
void SpeedWalk<Scalar>::finish()
{
  LOG_INFO("SpeedWalk.finish() called...")
  this->inBehavior = false;
}

template<typename Scalar>
void SpeedWalk<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    taskGains.resize(toUType(IkTasks::count));
    taskWeights.resize(toUType(IkTasks::count));
    GET_CONFIG(
      "MotionBehaviors",
      (Scalar, SpeedWalk.comWeight, taskWeights[toUType(IkTasks::com)]),
      (Scalar, SpeedWalk.comGain, taskGains[toUType(IkTasks::com)]),
      (Scalar, SpeedWalk.stepWeight, taskWeights[toUType(IkTasks::step)]),
      (Scalar, SpeedWalk.stepGain, taskGains[toUType(IkTasks::step)]),
      (Scalar, SpeedWalk.minJointVelWeight, taskWeights[toUType(IkTasks::minJointVel)]),
      (Scalar, SpeedWalk.minJointVelGain, taskGains[toUType(IkTasks::minJointVel)]),
      (Scalar, SpeedWalk.torsoWeight, taskWeights[toUType(IkTasks::torso)]),
      (Scalar, SpeedWalk.torsoGain, taskGains[toUType(IkTasks::torso)]),
    )
    loaded = true;
  }
}

template <typename Scalar>
void SpeedWalk<Scalar>::genStepTrajectory()
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
  //! Plotting
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
TNRSFootstep<Scalar> SpeedWalk<Scalar>::computeStepFromTo(
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
      atan2(transFromTo(1, 0), transFromTo(0, 0)) //! Euler Z angle
    );
  return TNRSFootstep<Scalar>(stepPose, to.foot, transFromTo, params->totalStepTime);
}

template <typename Scalar>
RobotPose2D<Scalar> SpeedWalk<Scalar>::stepVelToPose(
  const VelocityInput<Scalar>& vi)
{
  return
    RobotPose2D<Scalar>(
      vi.getX() * params->maxStepLengthX,
      vi.getY() * params->maxStepLengthY,
      vi.getTheta() * params->maxStepRotation);
}

template <typename Scalar>
void SpeedWalk<Scalar>::addFirstStep()
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
void SpeedWalk<Scalar>::genNextStep()
{
  auto poseDiff = this->stepVelToPose(getBehaviorCast()->velocityInput);
  Matrix<Scalar, 3, 3> diffRot;
  Matrix<Scalar, 4, 4> diffTrans;
  MathsUtils::makeRotationZ(diffRot, poseDiff.getTheta());
  diffTrans = MathsUtils::makeTransformation(diffRot, poseDiff.getX(), poseDiff.getY(), 0.0);
  //LOG_INFO("diffTrans: " << diffTrans);
  auto prevStep = getBackStep();
  if (!prevStep) { //! Robot is standing
    //! Get current foot position
    auto foot = this->kM->getGlobalBaseIndex() == RobotFeet::lFoot ? RobotFeet::rFoot : RobotFeet::lFoot;
    Matrix<Scalar, 4 ,4> transFromTo = this->kM->getGlobalToOther() * diffTrans;
    addStep(boost::shared_ptr<TNRSFootstep<Scalar> >(new TNRSFootstep<Scalar>(
      RobotPose2D<Scalar>(
        transFromTo(0, 3), transFromTo(1, 3), atan2(transFromTo(1, 0), transFromTo(0, 0))),
        foot,
        transFromTo,
        prevStep->timeAtFinish + params->totalStepTime)));
  } else {
    //! Get current foot position
    auto foot = prevStep->foot == RobotFeet::lFoot ? RobotFeet::rFoot : RobotFeet::lFoot;
    auto pose = RobotPose2D<Scalar>(
      diffTrans(0, 3), diffTrans(1, 3), atan2(diffTrans(1, 0), diffTrans(0, 0)));
    //! Clip foot if it is going out of max range
    pose.x() = VisionUtils::clip(pose.getX(), params->minStepLengthX, params->maxStepLengthX);
    if (foot == RobotFeet::lFoot) { //! Theta max range for right foot is inverse of left foot
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
void SpeedWalk<Scalar>::drawSteps()
{
  //! Generate first two steps
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
void SpeedWalk<Scalar>::addStep(
  const boost::shared_ptr<TNRSFootstep<Scalar> >& fs)
{
  return stepsQueue.push_back(fs);
}

template <typename Scalar>
boost::shared_ptr<TNRSFootstep<Scalar> >& SpeedWalk<Scalar>::getFrontStep()
{
  return stepsQueue.front();
}

template <typename Scalar>
boost::shared_ptr<TNRSFootstep<Scalar> >& SpeedWalk<Scalar>::getBackStep()
{
  return stepsQueue.back();
}

template <typename Scalar>
void SpeedWalk<Scalar>::popStep()
{
  return stepsQueue.pop_front();
}

template class SpeedWalk<MType>;
