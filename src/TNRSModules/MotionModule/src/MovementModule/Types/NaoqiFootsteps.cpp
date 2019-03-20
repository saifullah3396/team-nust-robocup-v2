/**
 * @file MotionModule/MovementModule/Types/NaoqiFootsteps.cpp
 *
 * This file implements the class NaoqiFootsteps
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#ifdef V6_CROSS_BUILD
#include <qi/alvalue.h>
#else
#include <alvalue/alvalue.h>
#endif
#include "LocalizationModule/include/LocalizationRequest.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MovementModule/Types/NaoqiFootsteps.h"
#include "Utils/include/DataHolders/TNRSFootstep.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/DataHolders/PositionInput.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/PathPlanner/State.h"

template <typename Scalar>
Scalar NaoqiFootsteps<Scalar>::footstepTime;

template <typename Scalar>
boost::shared_ptr<NaoqiFootstepsConfig> NaoqiFootsteps<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <NaoqiFootstepsConfig> (this->config);
}

template <typename Scalar>
void NaoqiFootsteps<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    //! read parameters from config file:
    GET_CONFIG(
      "PathPlanner",
      (Scalar, Walk.footstepTime, footstepTime),
    );
    loaded = true;
  }
}

template <typename Scalar>
bool NaoqiFootsteps<Scalar>::initiate()
{
  LOG_INFO("NaoqiFootsteps.initiate() called...")
  ROBOT_IN_MOTION_OUT(MotionModule) = false;
  if (getBehaviorCast()->startPosture)
    behaviorState = setPosture;
  else
    behaviorState = setupWalk;
  plannedPath = getBehaviorCast()->plannedPath;
  STEP_LEG_OUT(MotionModule) = RobotFeet::unknown;
  return true;
}

template <typename Scalar>
void NaoqiFootsteps<Scalar>::reinitiate(const BehaviorConfigPtr& cfg)
{
  LOG_INFO("NaoqiFootsteps.reinitiate() called...")
  this->config = cfg;
  plannedPath = getBehaviorCast()->plannedPath;
  behaviorState = setupWalk;
}

template <typename Scalar>
void NaoqiFootsteps<Scalar>::update()
{
  if (behaviorState == setPosture) {
    //cout << "NaoqiFootsteps setPosture" << endl;
    setPostureAction();
  } else if (behaviorState == setupWalk) {
    //cout << "NaoqiFootsteps setupWalk" << endl;
    setupWalkAction();
  } else if (behaviorState == updateWalk) {
    //cout << "NaoqiFootsteps updateWalk" << endl;
    updateWalkAction();
  } else if (behaviorState == stopWalk) {
    //cout << "NaoqiFootsteps updateWalk" << endl;
    stopWalkAction();
  } else if (behaviorState == postStopWalk) {
    //cout << "NaoqiFootsteps postStopWalk" << endl;
    postStopWalkAction();
  }
}


template <typename Scalar>
void NaoqiFootsteps<Scalar>::setPostureAction() {
  if (getBehaviorCast()->startPosture) {
    // if posture is not equal to desired one
    if (getBehaviorCast()->startPosture->targetPosture !=
        POSTURE_STATE_OUT(MotionModule))
    {
      this->setupChildRequest(getBehaviorCast()->startPosture);
    }
  }
  behaviorState = setupWalk;
}

template <typename Scalar>
void NaoqiFootsteps<Scalar>::setupWalkAction() {
  if (initWalk()) { // Initiates the walk execution as defined in child
    // Walk initaiton success
    currentStep = 0;
    prevStep = -1;
    ROBOT_IN_MOTION_OUT(MotionModule) = true;
    behaviorState = updateWalk;
  } else {
    // Walk initiation failed we stop
    behaviorState = stopWalk;
  }
}

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
bool NaoqiFootsteps<Scalar>::initWalk()
{
  try {
    auto pathIter = this->plannedPath.begin();
    const TNRSFootstep<float>* fromPlanned = pathIter.base();
    pathIter++;
    int unchangeable = 0;
    AL::ALValue naoqiFs = this->getFootsteps()[1];
    queue<boost::shared_ptr<TNRSFootstep<Scalar>>> empty;
    swap(steps, empty);
    if (naoqiFs.getSize() > 0) {
      for (size_t i = 0; i < naoqiFs.getSize(); ++i) {
        string legStr;
        if ((*pathIter).foot == RobotFeet::lFoot) legStr = "LLeg";
        else if ((*pathIter).foot == RobotFeet::rFoot) legStr = "RLeg";
        if (((string)naoqiFs[i][0]) == legStr) {
          fromPlanned = pathIter.base();
          pathIter++;
          unchangeable++;
        }
        Matrix<Scalar, 4, 4> stepTrans;
        Matrix<Scalar, 3, 3> stepRot;
        stepTrans.setIdentity();
        stepTrans(0, 3) = naoqiFs[i][2][0];
        stepTrans(1, 3) = naoqiFs[i][2][1];
        MathsUtils::makeRotationZ(stepRot, naoqiFs[i][2][2]);
        stepTrans.block(0, 0, 3, 3) = stepRot;
        auto stepFoot = RobotFeet::lFoot;
        if (((string)naoqiFs[i][0]) != "LLeg")
          stepFoot = RobotFeet::rFoot;
        auto s =
          boost::make_shared<TNRSFootstep<Scalar> >(
            RobotPose2D<Scalar>(
              naoqiFs[i][2][0], naoqiFs[i][2][1], naoqiFs[i][2][2]),
            stepFoot, stepTrans);
        steps.push(s);
      }
    }
    unsigned nSteps = this->plannedPath.size() - 1 - unchangeable;
    if (nSteps < 1)
      return false;
    AL::ALValue footName;
    AL::ALValue footSteps;
    AL::ALValue timeList;
    footName.arraySetSize(this->plannedPath.size() - 1 - unchangeable);
    footSteps.arraySetSize(this->plannedPath.size() - 1 - unchangeable);
    timeList.arraySetSize(this->plannedPath.size() - 1 - unchangeable);
    int stepIndex = 0;
    while (pathIter != this->plannedPath.end()) {
      // Get a step from the planned path
      Matrix<Scalar, 4, 4> stepTrans;
      TNRSFootstep<float> step = this->getFootstep(*fromPlanned, *pathIter, stepTrans);
      // Generate a NaoQi footstep array
      footSteps[stepIndex].arraySetSize(3);
      // Foot type
      if (step.foot == RobotFeet::lFoot) footName[stepIndex] = "LLeg";
      else if (step.foot == RobotFeet::rFoot) footName[stepIndex] = "RLeg";
      // Foot position
      footSteps[stepIndex][0] = step.pose2D.getX();
      footSteps[stepIndex][1] = step.pose2D.getY();
      footSteps[stepIndex][2] = step.pose2D.getTheta();
      steps.push(boost::make_shared<TNRSFootstep<Scalar> >(step));
      // Time taken by footstep
      timeList[stepIndex] = (stepIndex + 1) * this->footstepTime;
      ++stepIndex;
      /*cout << "state:" << endl;
        cout << "sX: " << (*pathIter).getX() << endl;
        cout << "sY: " << (*pathIter).getY() << endl;
        cout << "sT: " << (*pathIter).getTheta() << endl;
        cout << "sT: " << (*pathIter).getLeg() << endl;*/
      fromPlanned = pathIter.base();
      pathIter++;
    }
    startStep = N_FOOTSTEPS_OUT(MotionModule);
    // clears existing steps and starts new ones
    //cout << footName << endl;
    //cout << footSteps << endl;
    //cout << timeList << endl;
    this->naoqiSetFootsteps(footName, footSteps, timeList, true);
  } catch (const exception& e) {
    LOG_EXCEPTION(e.what())
    return false;
  }
  return true;
}
#else
template <typename Scalar>
bool NaoqiFootsteps<Scalar>::initWalk() {
  LOG_ERROR("NaoqiFootsteps<Scalar>::initWalk() undefined without AL::ALMotionProxy")
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
unsigned NaoqiFootsteps<Scalar>::getCurrentStep() {
  return N_FOOTSTEPS_OUT(MotionModule) - startStep;
}
#else
template <typename Scalar>
unsigned NaoqiFootsteps<Scalar>::getCurrentStep() {
  LOG_ERROR("NaoqiFootsteps<Scalar>::getCurrentStep() undefined without AL::ALMotionProxy")
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
bool NaoqiFootsteps<Scalar>::isWalkFinished() {
  return !this->naoqiMoveIsActive();
}
#else
template <typename Scalar>
bool NaoqiFootsteps<Scalar>::isWalkFinished() {
  LOG_ERROR("NaoqiFootsteps<Scalar>::isWalkFinished() undefined without AL::ALMotionProxy")
}
#endif

template <typename Scalar>
void NaoqiFootsteps<Scalar>::updateWalkAction()
{
  currentStep = this->getCurrentStep();
  //if (currentStep >= plannedPath.size() - 1) { // finished all steps
  if (isWalkFinished()) {
    POSTURE_STATE_OUT(MotionModule) =
      PostureState::standWalk;
    behaviorState = stopWalk;
  } else { // A new step is just started
    if (currentStep != prevStep) {
      // Get the time at which a new step is started
      footStepStartTime = this->motionModule->getModuleTime();
      prevStep = currentStep;
      if (currentStep > 0 && !steps.empty()) {
        // from torso to support foot and then new step foot to torso
        if (steps.front()->foot == RobotFeet::lFoot) {
          // From torso to right foot support frame
          torsoToSupportFoot(1, 3) = -0.05; // footspacing
          STEP_LEG_OUT(MotionModule) = RobotFeet::lFoot;
        } else {
          // From torso to left foot support frame
          //torsoToSupportFoot =
            //L_FOOT_TRANS_OUT(MotionModule).template cast<Scalar>();
          torsoToSupportFoot(1, 3) = 0.05; // footspacing
          STEP_LEG_OUT(MotionModule) = RobotFeet::rFoot;
        }
        Matrix<Scalar, 4, 4> newPosition =
          torsoToSupportFoot * steps.front()->trans * torsoToSupportFoot;
        PositionInput<float> input = // float used since localization request requires float
          PositionInput<float>(
            newPosition(0, 3),
            newPosition(1, 3),
            MathsUtils::matToEuler((Matrix<Scalar, 3, 3>) newPosition.block(0, 0, 3, 3))[2]
          );
        PositionUpdatePtr pu =
          boost::make_shared<PositionUpdate>(input);
        BaseModule::publishModuleRequest(pu);
        steps.pop();
      }
      // Set the position input to localization module from current
      // odometry data
      //if (currentStep < 2) {
        /*cout << "Current step: " << currentStep << endl;
        cout << "StepX: " << steps[currentStep].getX() << endl;
        cout << "StepY: " << steps[currentStep].getY() << endl;
        cout << "StepTheta: " << steps[currentStep].getTheta() << endl;
        cout << "pi.StepX: " << pi.getX() << endl;
        cout << "pi.StepY: " << pi.getY() << endl;
        cout << "pi.StepTheta: " << pi.getTheta() << endl;
        cout << "Time ratio:" << timeRatio << endl;
        cout << "pi: " << pi.mat.transpose() << endl;*/
      //}
      //cout << "pi: " << pi.mat.transpose() << endl;
      //pi *= timeRatio;
    }// else { // A step is in motion
      // Get the time passed for the step in progress
     // Scalar stepTimePassed =
      //  this->motionModule->getModuleTime() - footStepStartTime;

      //cout << "footStepStartTime: " << footStepStartTime << endl;
      //cout << "stepTimePassed: " << stepTimePassed << endl;

      /*bool stepInProgress;
      if (currentStep == 0) { // If this is first step
        // First step always takes more time naoqi's fault.
        // Therefore add a 200 milliseconds of time in addition
        static Scalar overheadTime = 0.2;
        stepInProgress =
          stepTimePassed <= footstepTime + overheadTime ||
          stepTimePassed - footstepTime + overheadTime < 0.01;
      } else {
        stepInProgress =
          stepTimePassed <= footstepTime ||
          stepTimePassed - footstepTime < 0.01;
      }
      if (stepInProgress) { // If the step is still in progress
        // ratio of time passed
        //Scalar timeRatio;
        if (currentStep == 0) {
          // First step always takes more time.
          // Therefore add a 200 milliseconds of time in addition
          timeRatio = stepTimePassed / (footstepTime + 0.2);
        } else {
          timeRatio = stepTimePassed / footstepTime;
        }
      }*/
   //}
  }
}

template <typename Scalar>
void NaoqiFootsteps<Scalar>::stopWalkAction()
{
  if (getBehaviorCast()->endPosture) {
    // if posture is not equal to desired one
    if (getBehaviorCast()->endPosture->targetPosture !=
        POSTURE_STATE_OUT(MotionModule))
    {
      this->setupChildRequest(getBehaviorCast()->endPosture);
    }
  } else {
    behaviorState = postStopWalk;
  }
}

template <typename Scalar>
void NaoqiFootsteps<Scalar>::postStopWalkAction()
{
  finish();
}

template <typename Scalar>
TNRSFootstep<float> NaoqiFootsteps<Scalar>::getFootstep(
  const TNRSFootstep<float>& from,
  const TNRSFootstep<float>& to,
  Matrix<Scalar, 4, 4>& stepTrans)
{
  Matrix<Scalar, 3, 3> toRotation;
  MathsUtils::makeRotationZ(toRotation, to.pose2D.getTheta());
  Matrix<Scalar, 3, 3> fromRotation;
  MathsUtils::makeRotationZ(fromRotation, from.pose2D.getTheta());

  stepTrans =
    MathsUtils::getTInverse(
      MathsUtils::makeTransformation(
        fromRotation,
        from.pose2D.getX(),
        from.pose2D.getY(),
        0.0)) * MathsUtils::makeTransformation(
      toRotation,
      to.pose2D.getX(),
      to.pose2D.getY(),
      0.0);
  TNRSFootstep<float> footstep;
  footstep.pose2D.x() = stepTrans(0, 3);
  footstep.pose2D.y() = stepTrans(1, 3);
  footstep.pose2D.theta() =
    MathsUtils::matToEuler((Matrix<Scalar, 3, 3>) stepTrans.block(0, 0, 3, 3))[2];
  footstep.foot = to.foot;
  return footstep;
}

template <typename Scalar>
void NaoqiFootsteps<Scalar>::finish()
{
  LOG_INFO("NaoqiFootsteps.finish() called...")
  STEP_LEG_OUT(MotionModule) = RobotFeet::unknown;
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  this->stopMove();
  #endif
  ROBOT_IN_MOTION_OUT(MotionModule) = false;
  this->inBehavior = false;
}

template class NaoqiFootsteps<MType>;
