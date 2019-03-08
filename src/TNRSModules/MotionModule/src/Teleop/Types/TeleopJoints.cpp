/**
 * @file MotionModule/src/Teleop/Types/TeleopJoints.cpp
 *
 * This file implements the class TeleopJoints
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/Teleop/Types/TeleopJoints.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/KinematicsModule/TaskIkSolver.h"
#include "Utils/include/Behaviors/MBConfigs/MBTeleopConfig.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/HardwareIds.h"

template<typename Scalar>
Scalar TeleopJoints<Scalar>::jointTaskGain = 0.01;

template <typename Scalar>
TeleopJointsConfigPtr TeleopJoints<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<TeleopJointsConfig> (this->config);
}

template <typename Scalar>
bool TeleopJoints<Scalar>::initiate()
{
  LOG_INFO("TeleopJoints.initiate() called...");
  this->kM->getTaskSolver()->setMaxVelocityLimitGain(0.25);
  auto activeJoints = vector<bool>(toUType(Joints::count), true);
  Matrix<Scalar, Dynamic, 1> target = this->kM->getJointPositions();
  jointTask = this->kM->makePostureTask(target, activeJoints, 1.0, jointTaskGain);
  return true;
}

template <typename Scalar>
void TeleopJoints<Scalar>::update()
{
  Matrix<Scalar, Dynamic, 1> commands =
    Map<Matrix<Scalar, 1, Dynamic> >(
      getBehaviorCast()->jointCommands.data(),
      getBehaviorCast()->jointCommands.size());
  jointTask->setTargetPosture(commands);
  this->addMotionTask(jointTask);
}

template <typename Scalar>
void TeleopJoints<Scalar>::finish()
{
  LOG_INFO("TeleopJoints.finish() called...")
  this->inBehavior = false;
}

template<typename Scalar>
void TeleopJoints<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG(
      "MotionBehaviors",
      (Scalar, TeleopJoints.jointTaskGain, jointTaskGain),
    );
    loaded = true;
  }
}

template class TeleopJoints<MType>;
