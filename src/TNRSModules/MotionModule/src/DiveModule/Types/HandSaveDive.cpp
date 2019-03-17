/**
 * @file MotionModule/DiveModule/Types/HandSaveDive.h
 *
 * This file implements the class HandSaveDive
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBDiveConfig.h"
#include "MotionModule/include/DiveModule/Types/HandSaveDive.h"
#include "MotionModule/include/MotionGenerator.h"
#include "MotionModule/include/BalanceModule/Types/ZmpControl.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "Utils/include/HardwareIds.h"

template<typename Scalar>
vector<Scalar> HandSaveDive<Scalar>::taskWeights;
template<typename Scalar>
vector<Scalar> HandSaveDive<Scalar>::taskGains;

static const MType diveLeftDef[1][24] =
  { // Time + joints Target
    {0.0, 16.0, 90.0, 35, -90.0, -3.0, -90.0, 90.0, 0.0, 45.0, 63.0, 0.0,
    -14.2, 18.3, -45, 120, -67, 4.5, -14.2, 0.0, -30.0, 100.0, -50, 7.0 },
  };

static const MType diveRightDef[1][24] =
  { // Time + joints Target
    {0.0, 16.0, 90.0, 0.0, -45.0, -63, 0.0, 90.0, -35, 90.0, 3.0, 90.0,
    -14.2, 0.0, -30.0, 100.0, -50, -7.0, -14.2, -18.3, -45, 120, -67, -4.5},
  };

template <typename Scalar>
HandSaveDive<Scalar>::HandSaveDive(
  MotionModule* motionModule,
  const boost::shared_ptr<HandSaveDiveConfig>& config) :
  DiveModule<Scalar>(motionModule, config, "HandSaveDive")
{
}

template <typename Scalar>
bool HandSaveDive<Scalar>::initiate()
{
  LOG_INFO("HandSaveDive.initiate() called...")
  auto zmpControlCfg =
    boost::make_shared<ZmpControlConfig>();
  zmpControlCfg->supportLeg = getBehaviorCast()->supportLeg,
  zmpControlCfg->keepOtherLegContact = true;
  zmpControlCfg->regularizePosture = false;
  zmpControlCfg->keepTorsoUpright = false;

  Matrix<Scalar, Dynamic, 1> targetJoints(toUType(Joints::count));
  if (getBehaviorCast()->supportLeg == static_cast<LinkChains>(RobotFeet::lFoot)) {
    targetJoints = Matrix<Scalar, Dynamic, 1>::Map(
      &diveLeftDef[0][0],
      sizeof(diveLeftDef[0]) / sizeof(diveLeftDef[0][0]));
  } else {
    targetJoints = Matrix<Scalar, Dynamic, 1>::Map(
      &diveRightDef[1][0],
      sizeof(diveRightDef[1]) / sizeof(diveRightDef[1][0]));
  }
  targetJoints *= M_PI / 180.f;

  tasks.resize(NUM_TASKS);
  auto activeJoints = vector<bool>(toUType(Joints::count), false);
  for (size_t i = 0; i < toUType(HardwareIds::nLLeg); ++i)
    activeJoints[toUType(HardwareIds::lLegStart) + i] = false;
  for (size_t i = 0; i < toUType(HardwareIds::nRLeg); ++i)
    activeJoints[toUType(HardwareIds::rLegStart) + i] = false;
  zmpControlCfg->activeJoints = vector<unsigned>(activeJoints.begin(), activeJoints.end());
  this->setupChildRequest(zmpControlCfg, true);

  Matrix<Scalar, 4, 4> target =
    MathsUtils::getTInverse(
      this->kM->getForwardEffector(getBehaviorCast()->supportLeg, toUType(LegEEs::footCenter)));
  Matrix<Scalar, 4, 4> rot;
  //MathsUtils::makeRotationX(rot, -5 * M_PI / 180.0);
  //target *= rot;
  target(0, 3) = 0.05;
  target(1, 3) = NAN;
  target(2, 3) -= 0.15;
  //activeJoints = vector<bool>(toUType(Joints::count), false);
  //for (int i = 1; i < this->kM->getLinkChain(supportLeg)->size; ++i)
  //  activeJoints[this->kM->getLinkChain(supportLeg)->start + i] = true;
  tasks[TORSO_TASK] =
    this->kM->makeTorsoTask(
      static_cast<RobotFeet>(getBehaviorCast()->supportLeg),
      LegEEs::footCenter,
      target,
      activeJoints,
      taskWeights[TORSO_TASK],
      taskGains[TORSO_TASK]);

  //activeJoints = vector<bool>(toUType(Joints::count), false);
  //for (int i = 1; i < this->kM->getLinkChain(supportLeg)->size; ++i)
    //activeJoints[this->kM->getLinkChain(supportLeg)->start + i] = true;

  //auto otherLeg = getBehaviorCast()->supportLeg == CHAIN_L_LEG ? CHAIN_R_LEG : CHAIN_L_LEG;
  //tasks[OTHER_LEG_TASK] = this->kM->makeCartesianTask(otherLeg, )
  //tasks[POSTURE_TASK] = this->kM->makePostureTask(targetJoints, activeJoints, taskWeights[POSTURE_TASK], taskGains[POSTURE_TASK]);
  return true;
}

template <typename Scalar>
void HandSaveDive<Scalar>::update()
{
  for (const auto& task : tasks) {
    if (task) this->addMotionTask(task);
  }
}

template <typename Scalar>
void HandSaveDive<Scalar>::finish()
{
  LOG_INFO("HandSaveDive.finish() called...")
  this->inBehavior = false;
}

template<typename Scalar>
void HandSaveDive<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    taskGains.resize(NUM_TASKS);
    taskWeights.resize(NUM_TASKS);
    Scalar posTWeight, posTGain;
    Scalar olTaskWeight, olTaskGain;
    Scalar handTaskWeight, handTaskGain;
    Scalar torsoTaskWeight, torsoTaskGain;
    GET_CONFIG(
      "MotionBehaviors",
      (Scalar, HandSaveDive.posTWeight, posTWeight),
      (Scalar, HandSaveDive.posTGain, posTGain),
      (Scalar, HandSaveDive.olTaskWeight, olTaskWeight),
      (Scalar, HandSaveDive.olTaskGain, olTaskGain),
      (Scalar, HandSaveDive.handTaskWeight, handTaskWeight),
      (Scalar, HandSaveDive.handTaskGain, handTaskGain),
      (Scalar, HandSaveDive.torsoTaskWeight, torsoTaskWeight),
      (Scalar, HandSaveDive.torsoTaskGain, torsoTaskGain),
    );
    taskWeights[POSTURE_TASK] = posTWeight;
    taskGains[POSTURE_TASK] = posTGain;
    taskWeights[OTHER_LEG_TASK] = olTaskWeight;
    taskGains[OTHER_LEG_TASK] = olTaskGain;
    taskWeights[HAND_TASK] = handTaskWeight;
    taskGains[HAND_TASK] = handTaskGain;
    taskWeights[TORSO_TASK] = torsoTaskWeight;
    taskGains[TORSO_TASK] = torsoTaskGain;
    loaded = true;
  }
}

template <typename Scalar>
boost::shared_ptr<HandSaveDiveConfig> HandSaveDive<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<HandSaveDiveConfig> (this->config);
}

template class HandSaveDive<MType>;
