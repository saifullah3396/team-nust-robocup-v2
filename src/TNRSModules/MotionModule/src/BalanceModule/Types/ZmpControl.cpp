/**

 * @file MotionModule/BalanceModule/ZmpControl.cpp
 *
 * This file implements the class ZmpControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include "MotionModule/include/BalanceModule/BalanceDefinitions.h"
#include "MotionModule/include/BalanceModule/Types/ZmpControl.h"
#include "MotionModule/include/BalanceModule/ZmpPreviewController.h"
#include "MotionModule/include/BalanceModule/ZmpRefGenerator.h"
#include "MotionModule/include/BalanceModule/ZmpRef.h"
#include "MotionModule/include/BalanceModule/BalanceZmpRefGen.h"
#include "MotionModule/include/KinematicsModule/ComState.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/KinematicsModule/TaskIkSolver.h"
#include "MotionModule/include/JointRequest.h"
#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"
#include "Utils/include/ConfigMacros.h"

template<typename Scalar>
unsigned ZmpControl<Scalar>::nPreviews;
template<typename Scalar>
vector<Scalar> ZmpControl<Scalar>::taskWeights;
template<typename Scalar>
vector<Scalar> ZmpControl<Scalar>::taskGains;

template<typename Scalar>
ZmpControl<Scalar>::ZmpControl(
  MotionModule* motionModule,
  const boost::shared_ptr<ZmpControlConfig>& config) :
  BalanceModule<Scalar>(motionModule, config, "ZmpControl")
{
  controllers.resize(2); // x-y dimensions
  if (getBehaviorCast()->supportLeg == LinkChains::lLeg) {
    postureTarget = Matrix<Scalar, Dynamic, 1>::Map(
      &balanceDefs[0][0],
      sizeof(balanceDefs[0]) / sizeof(balanceDefs[0][0]));
  } else {
    postureTarget = Matrix<Scalar, Dynamic, 1>::Map(
      &balanceDefs[1][0],
      sizeof(balanceDefs[1]) / sizeof(balanceDefs[1][0]));
  }
}

template<typename Scalar>
ZmpControl<Scalar>::~ZmpControl()
{
  for (size_t i = 0; i < controllers.size(); ++i)
    delete controllers[i];
}

template<typename Scalar>
bool ZmpControl<Scalar>::initiate()
{
  LOG_INFO("ZmpControl.initiate()")
  //! Set up center of mass and zmp reference logs
  comLog.open(
    (ConfigManager::getLogsDirPath() + string("ZmpControl/Com.txt")).c_str(),
    std::ofstream::out | std::ofstream::trunc
  );
  comLog << "# X Y" << endl;
  comLog.close();
  zmpRegLog.open(
    (ConfigManager::getLogsDirPath() + string("ZmpControl/ZmpRef.txt")).c_str(),
    std::ofstream::out | std::ofstream::trunc
  );
  zmpRegLog << "# X Y" << endl;
  zmpRegLog.close();

  //! Set the support leg as global base for kinematic computations
  LinkChains& supportLeg = getBehaviorCast()->supportLeg;
  this->kM->setGlobalBase(static_cast<RobotFeet>(supportLeg), LegEEs::footCenter);

  //! Set current joint estimates to simulated states
  this->kM->setStateFromTo(JointStateType::actual, JointStateType::sim);

  //! Set max joint velocities to be used
  this->kM->getTaskSolver()->setMaxVelocityLimitGain(1.0);

  //! Set up the reference generator based on given zmp targets
  refGenerator =
    boost::shared_ptr<BalanceZmpRefGen<Scalar>>(new BalanceZmpRefGen<Scalar>(
        this->motionModule,
        static_cast<RobotFeet>(getBehaviorCast()->supportLeg),
        nPreviews,
        (Scalar)getBehaviorCast()->timeToReachB,
        getBehaviorCast()->target
    ));
  refGenerator->initiate();

  //! Setup the center of mass desired position
  auto comState = this->kM->getComStateWrtFrame(supportLeg, toUType(LegEEs::footBase));
  desComPosition.setZero();
  desComPosition[2] = comState.position[2];

  //! Initiate the preview controllers for x-y directions
  for (int i = 0; i < controllers.size(); ++i) {
    controllers[i] = new ZmpPreviewController<Scalar>(this->kM->getComModel(i));
    controllers[i]->setPreviewLength(nPreviews);
    controllers[i]->initController();
  }

  //! Set shoulder pitch roll joint limits so it does not self collide if used
  //this->kM->getTaskSolver()->setMinJointLimit(MathsUtils::degToRads(12.0), L_SHOULDER_ROLL);
  //this->kM->getTaskSolver()->setMaxJointLimit(MathsUtils::degToRads(22.0), L_SHOULDER_ROLL);
  //this->kM->getTaskSolver()->setMaxJointLimit(MathsUtils::degToRads(-12.0), R_SHOULDER_ROLL);
  //this->kM->getTaskSolver()->setMinJointLimit(MathsUtils::degToRads(-22.0), R_SHOULDER_ROLL);

  otherLeg =
    getBehaviorCast()->supportLeg == LinkChains::lLeg ? LinkChains::rLeg : LinkChains::lLeg;
  activeJoints = vector<bool>(getBehaviorCast()->activeJoints.begin(), getBehaviorCast()->activeJoints.end());

  tasks.resize(toUType(IkTasks::count));
  //! Make a center of mass task to control its desired trajectory
  auto comResidual = vector<bool>(3, true);

  //! Remove center of mass z-axis tracking
  comResidual[2] = false;
  tasks[toUType(IkTasks::com)] =
    this->kM->makeComTask(
      static_cast<RobotFeet>(supportLeg),
      LegEEs::footCenter,
      comState.position,
      activeJoints,
      taskWeights[toUType(IkTasks::com)],
      taskGains[toUType(IkTasks::com)],
      comResidual
    );

  if (getBehaviorCast()->keepOtherLegContact) {
    //! keep the other leg on ground contact with high priority
    tasks[toUType(IkTasks::contact)] =
      this->kM->makeContactTask(
        otherLeg,
        toUType(LegEEs::footBase),
        activeJoints,
        taskWeights[toUType(IkTasks::contact)],
        taskGains[toUType(IkTasks::contact)]
      );
  }

  if (getBehaviorCast()->regularizeIk) {
    //! Use predefined joints as target for end posture after balance
    //! This is necessary for convergence towards a feasible posture
    tasks[toUType(IkTasks::regularization)] =
      this->kM->makePostureTask(
        postureTarget,
        activeJoints,
        taskWeights[toUType(IkTasks::regularization)],
        taskGains[toUType(IkTasks::regularization)]
      );
  }

  if (getBehaviorCast()->useTargetPosture) {
    //! Use predefined joints as target for end posture after balance
    //! This is necessary for convergence towards a feasible posture
    tasks[toUType(IkTasks::posture)] =
      this->kM->makePostureTask(
        postureTarget,
        activeJoints,
        taskWeights[toUType(IkTasks::posture)],
        taskGains[toUType(IkTasks::posture)]
      );
  }

  if (getBehaviorCast()->keepTorsoUpright) {
    //! Torso task to keep torso orientation to the initial orientation
    //! This can be useful if the torso is to be kept upright
    Matrix<Scalar, 4, 4> target = this->kM->getGlobalToBody();
    auto activeResidual = vector<bool>(6, false); //! X-Y-Z, Roll-Pitch-Yaw
    //! Only use Pitch tracking
    activeResidual[4] = true;
    tasks[toUType(IkTasks::torso)] =
      this->kM->makeTorsoTask(
        static_cast<RobotFeet>(supportLeg),
        LegEEs::footBase,
        target,
        activeJoints,
        taskWeights[toUType(IkTasks::torso)],
        taskGains[toUType(IkTasks::torso)],
        activeResidual
      );
  }
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  //! In case motion proxy is turned on, we cannot use real-time updates.
  //! We can then simply find ik for all center of mass states and interpolate
  //! between them.
  LOG_ERROR("ZmpControl is undefined with USE_NAOQI_MOTION_PROXY=ON");
  return false;

  //! Make a task solver
  TaskIkSolver<Scalar> tis =
    TaskIkSolver<Scalar>(
      this->motionModule->getKinematicsModule(), 1, activeJoints, true, this->cycleTime);
  tis.init();
  tis.setActiveJoints(activeJoints);
  unsigned timeStep = 1;
  static bool contactTargetUpdated = true;
  vector<Matrix<Scalar, Dynamic, 1> > joints;
  vector<Scalar> times;
  while(true) {
    if (timeStep * this->cycleTime >= (Scalar)getBehaviorCast()->timeToReachB)
      break;
    if (!contactTargetUpdated && timeStep * this->cycleTime >= (Scalar)getBehaviorCast()->timeToReachB / 2) {
      Matrix<Scalar, 4, 4> target = boost::static_pointer_cast<CartesianTask<Scalar> >(tasks[toUType(IkTasks::contact)])->getTarget();
      target(2, 3) += 0.01;
      boost::static_pointer_cast<CartesianTask<Scalar> >(tasks[toUType(IkTasks::contact)])->setTarget(target);
      tasks[toUType(IkTasks::contact)]->setGain(0.05);
      tasks[toUType(IkTasks::posture)] =
        this->kM->makePostureTask(
          postureTarget, activeJoints, taskWeights[toUType(IkTasks::posture)], taskGains[toUType(IkTasks::posture)]);
      contactTargetUpdated = true;
    }
    tis.reset(false);
    refGenerator->update(timeStep * this->cycleTime);
    zmpRef = refGenerator->getCurrentRef();
    zmpRegLog.open(
      (ConfigManager::getLogsDirPath() + string("ZmpControl/ZmpRef.txt")).c_str(),
      std::ofstream::out | std::ofstream::app
    );
    zmpRegLog << zmpRef->x[0] << " " << zmpRef->y[0] << endl;
    zmpRegLog.close();
    auto desState =
      controllers[0]->step(
        Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1> >(zmpRef->x.linearize(), zmpRef->x.size()));
    desComPosition[0] = desState[0];
    desState =
      controllers[1]->step(
        Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1> >(zmpRef->y.linearize(), zmpRef->y.size()));
    desComPosition[1] = desState[1];
    comLog.open(
      (ConfigManager::getLogsDirPath() + string("ZmpControl/Com.txt")).c_str(),
      std::ofstream::out | std::ofstream::app
    );
    auto simComState = this->kM->computeComWrtBase(getBehaviorCast()->supportLeg, toUType(LegEEs::footBase), JointStateType::sim);
    comLog << this->motionModule->getModuleTime() << "  "
           << desComPosition[0] << " "
           << desComPosition[1] << " "
           << simComState[0] << " "
           << simComState[1]
           << endl;
    comLog.close();
    boost::static_pointer_cast<ComTask<Scalar> >(tasks[toUType(IkTasks::com)])->setTargetCom(desComPosition);
    boost::static_pointer_cast<ComTask<Scalar> >(tasks[toUType(IkTasks::com)])->setFirstStep(false);
    for (auto& task : tasks) {
      if (task) tis.addTask(task);
    }
    joints.push_back(tis.solve(1));
    times.push_back(timeStep * this->cycleTime);
    timeStep++;
  }
  Matrix<bool, Dynamic, 1> eAj(toUType(Joints::count));
  for (size_t i = 0; i < toUType(Joints::count); ++i)
    eAj[i] = activeJoints[i];
  this->naoqiJointInterpolation(joints, times, eAj, true);
  #endif
  return true;
}

template <typename Scalar>
void ZmpControl<Scalar>::reinitiate(const BehaviorConfigPtr& cfg)
{
  #ifndef NAOQI_MOTION_PROXY_AVAILABLE
  auto casted = boost::static_pointer_cast <ZmpControlConfig> (cfg);
  getBehaviorCast()->keepOtherLegContact = casted->keepOtherLegContact;
  getBehaviorCast()->useTargetPosture = casted->useTargetPosture;
  getBehaviorCast()->regularizeIk = casted->regularizeIk;
  getBehaviorCast()->keepTorsoUpright = casted->keepTorsoUpright;
  getBehaviorCast()->activeJoints = casted->activeJoints;
  activeJoints = vector<bool>(getBehaviorCast()->activeJoints.begin(), getBehaviorCast()->activeJoints.end());

  // Set dummy target
  Matrix<Scalar, 3, 1> comTarget;
  comTarget.setZero();
  auto comResidual = vector<bool>(3, true);
  comResidual[2] = false;
  tasks[toUType(IkTasks::com)] =
    this->kM->makeComTask(
      static_cast<RobotFeet>(getBehaviorCast()->supportLeg),
      LegEEs::footCenter,
      comTarget,
      activeJoints,
      taskWeights[toUType(IkTasks::com)],
      taskGains[toUType(IkTasks::com)],
      comResidual);
  // Reassign all tasks
  if (getBehaviorCast()->keepOtherLegContact) {
    //LOG_INFO("Adding new contact task")
    // keep the other leg on ground contact with high priority
    tasks[toUType(IkTasks::contact)] =
      this->kM->makeContactTask(
        otherLeg,
        toUType(LegEEs::footBase),
        activeJoints,
        taskWeights[toUType(IkTasks::contact)],
        taskGains[toUType(IkTasks::contact)]);
  } else {
    //LOG_INFO("Removing contact task")
    tasks[toUType(IkTasks::contact)].reset();
  }

  if (getBehaviorCast()->useTargetPosture) {
    //LOG_INFO("Adding new posture task")
    // Use predefined joints as target for end posture after balance
    // This is necessary to get a good posture
    // Add a posture task to be reached during balance
    tasks[toUType(IkTasks::posture)] =
      this->kM->makePostureTask(
        postureTarget,
        activeJoints,
        taskWeights[toUType(IkTasks::posture)],
        taskGains[toUType(IkTasks::posture)]);
  } else {
    //LOG_INFO("Removing posture task")
    tasks[toUType(IkTasks::posture)].reset();
  }

  if (getBehaviorCast()->regularizeIk) {
    //LOG_INFO("Adding new posture task")
    // Use predefined joints as target for end posture after balance
    // This is necessary to get a good posture
    // Add a posture task to be reached during balance
    tasks[toUType(IkTasks::regularization)] =
      this->kM->makePostureTask(
        postureTarget,
        activeJoints,
        taskWeights[toUType(IkTasks::regularization)],
        taskGains[toUType(IkTasks::regularization)]);
  } else {
    //LOG_INFO("Removing posture task")
    tasks[toUType(IkTasks::regularization)].reset();
  }

  if (getBehaviorCast()->keepTorsoUpright) {
    //LOG_INFO("Adding new torso task")
    // Torso task to keep torso orientation to the initial orientation
    // This can be useful in some cases
    auto& supportLeg = getBehaviorCast()->supportLeg;
    /*Matrix<Scalar, 4, 4> target = MathsUtils::getTInverse(this->kM->getForwardEffector(supportLeg, toUType(LegEEs::footBase)));
    vector<bool> activeJointsTorso = vector<bool>(toUType(Joints::count), false);
    for (int i = 1; i < this->kM->getLinkChain(supportLeg)->size; ++i)
      activeJointsTorso[this->kM->getLinkChain(supportLeg)->start + i] = true;
    tasks[toUType(IkTasks::torso)] =  this->kM->makeTorsoTask(supportLeg, toUType(LegEEs::footBase), target, activeJointsTorso, taskWeights[toUType(IkTasks::torso)], taskGains[toUType(IkTasks::torso)]);*/
    Matrix<Scalar, 4, 4> target = this->kM->getGlobalToBody();
    auto activeResidual = vector<bool>(6, false); // X-Y-Z, Roll-Pitch-Yaw
    tasks[toUType(IkTasks::torso)] =
      this->kM->makeTorsoTask(
      static_cast<RobotFeet>(supportLeg),
      LegEEs::footBase,
      target,
      activeJoints,
      taskWeights[toUType(IkTasks::torso)],
      taskGains[toUType(IkTasks::torso)],
      activeResidual);
  } else {
    //LOG_INFO("Removing torso task")
    tasks[toUType(IkTasks::torso)].reset();
  }
  LOG_INFO("ZmpControl.reinitiate() finished...")
  #endif
}

template<typename Scalar>
void ZmpControl<Scalar>::update()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  if (this->runTime > (Scalar)getBehaviorCast()->timeToReachB)
    finish();
  #else
  if (this->runTime > this->maxTime) {
    finish();
  } else {
    trackZmp();
  }
  #endif
}

template<typename Scalar>
void ZmpControl<Scalar>::finish()
{
  LOG_INFO("ZmpControl.finish() called...")
  this->inBehavior = false;
}

template<typename Scalar>
void ZmpControl<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    taskGains.resize(toUType(IkTasks::count));
    taskWeights.resize(toUType(IkTasks::count));
    Scalar comTWeight, comTGain;
    Scalar posTWeight, posTGain;
    Scalar regTWeight, regTGain;
    Scalar conTWeight, conTGain;
    Scalar torsoTWeight, torsoTGain;
    GET_CONFIG(
      "MotionBehaviors",
      (unsigned, ZmpControl.nPreviews, nPreviews),
      (Scalar, ZmpControl.comTWeight, comTWeight),
      (Scalar, ZmpControl.comTGain, comTGain),
      (Scalar, ZmpControl.posTWeight, posTWeight),
      (Scalar, ZmpControl.posTGain, posTGain),
      (Scalar, ZmpControl.regTWeight, regTWeight),
      (Scalar, ZmpControl.regTGain, regTGain),
      (Scalar, ZmpControl.conTWeight, conTWeight),
      (Scalar, ZmpControl.conTGain, conTGain),
      (Scalar, ZmpControl.torsoTWeight, torsoTWeight),
      (Scalar, ZmpControl.torsoTGain, torsoTGain),
      (Scalar, ZmpControl.maxTime, maxTime),
    )
    taskWeights[toUType(IkTasks::com)] = comTWeight;
    taskGains[toUType(IkTasks::com)] = comTGain;
    taskWeights[toUType(IkTasks::contact)] = conTWeight;
    taskGains[toUType(IkTasks::contact)] = conTGain;
    taskWeights[toUType(IkTasks::posture)] = posTWeight;
    taskGains[toUType(IkTasks::posture)] = posTGain;
    taskWeights[toUType(IkTasks::regularization)] = regTWeight;
    taskGains[toUType(IkTasks::regularization)] = regTGain;
    taskWeights[toUType(IkTasks::torso)] = torsoTWeight;
    taskGains[toUType(IkTasks::torso)] = torsoTGain;
    loaded = true;
  }
}

template<typename Scalar>
ZmpControlConfigPtr ZmpControl<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <ZmpControlConfig> (this->config);
}

template<typename Scalar>
void ZmpControl<Scalar>::trackZmp()
{
  refGenerator->update(this->runTime);
  zmpRef = refGenerator->getCurrentRef();
  zmpRegLog.open(
    (ConfigManager::getLogsDirPath() + string("ZmpControl/ZmpRef.txt")).c_str(),
    std::ofstream::out | std::ofstream::app
  );
  zmpRegLog << zmpRef->x[0] << " " << zmpRef->y[0] << endl;
  zmpRegLog.close();
  desComPosition[0] =
    controllers[0]->step(
      Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1> >(
        zmpRef->x.linearize(), zmpRef->x.size()))[0];
  desComPosition[1] =
    controllers[1]->step(
      Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1> >(
        zmpRef->y.linearize(), zmpRef->y.size()))[0];
  auto comState =
    this->kM->getComStateWrtFrame(
      static_cast<LinkChains>(refGenerator->getRefFrame()), toUType(LegEEs::footBase));
  comLog.open(
    (ConfigManager::getLogsDirPath() + string("ZmpControl/Com.txt")).c_str(),
    std::ofstream::out | std::ofstream::app
  );
  auto comFromBase = this->kM->computeComWrtBase(getBehaviorCast()->supportLeg, toUType(LegEEs::footCenter));
  auto simComState = this->kM->computeComWrtBase(getBehaviorCast()->supportLeg, toUType(LegEEs::footCenter), JointStateType::sim);
  comLog << this->motionModule->getModuleTime() << "  "
         << comState.position[0] << " "
         << comState.position[1] << " "
         //<< comState.velocity[0] << " "
         //<< comState.velocity[1] << " "
         //<< comState.accel[0] << " "
         //<< comState.accel[1] << " "
         << desComPosition[0] << " "
         << desComPosition[1] << " "
         //<< desComVelocity[0] << " "
         //<< desComVelocity[1] << " "
         //<< desComAccel[0] << " "
         //<< desComAccel[1] << " "
         << comState.zmp[0] << " "
         << comState.zmp[1] << " "
         << comFromBase[0] << " "
         << comFromBase[1] << " "
         << simComState[0] << " "
         << simComState[1]
         << endl;
  comLog.close();
  boost::static_pointer_cast<ComTask<Scalar> >(tasks[toUType(IkTasks::com)])->setTargetCom(desComPosition);
  boost::static_pointer_cast<ComTask<Scalar> >(tasks[toUType(IkTasks::com)])->setFirstStep(true);
  if (tasks[toUType(IkTasks::regularization)])
    boost::static_pointer_cast<PostureTask<Scalar> >(tasks[toUType(IkTasks::regularization)])->setTargetPosture(this->kM->getJointPositions());
  for (auto& task : tasks) {
    if (task) {
      this->addMotionTask(task);
    }
  }
}

template class ZmpControl<MType>;
