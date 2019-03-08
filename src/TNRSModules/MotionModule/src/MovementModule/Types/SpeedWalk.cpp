/**
 * @file MotionModule/MovementModule/Types/SpeedWalk.cpp
 *
 * This file implements the class SpeedWalk
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "BehaviorManager/include/StateMachine.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MovementModule/TNRSFootstep.h"
#include "MotionModule/include/MovementModule/Types/SpeedWalk.h"
#include "MotionModule/include/MovementModule/WalkParameters.h"
#include "MotionModule/include/MovementModule/WalkZmpRefGen.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/Behaviors/MBConfigs/MBPostureConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBMovementConfig.h"
#include "Utils/include/VisionUtils.h"

template <typename Scalar>
SpeedWalk<Scalar>::SpeedWalk(
  MotionModule* motionModule,
  const boost::shared_ptr<SpeedWalkConfig>& config) :
  MovementModule<Scalar>(motionModule, config, "SpeedWalk")
{
  DEFINE_FSM_STATE(SpeedWalk<Scalar>, PlanSteps, planSteps);
  DEFINE_FSM_STATE(SpeedWalk<Scalar>, ExecuteDSS, executeDSS);
  DEFINE_FSM_STATE(SpeedWalk<Scalar>, ExecuteSSS, executeSSS);
  DEFINE_FSM(fsm, SpeedWalk<Scalar>, planSteps);
  params = boost::make_shared<WalkParameters<Scalar> >();
  walkZmpRefGen =
    boost::make_shared<WalkZmpRefGen<Scalar> >(
      this->motionModule,
      this->kM->getGlobalBaseIndex(),
      params->nPreviews,
      &stepsQueue);
}

template <typename Scalar>
boost::shared_ptr<SpeedWalkConfig> SpeedWalk<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <SpeedWalkConfig> (this->config);
}

template <typename Scalar>
bool SpeedWalk<Scalar>::initiate()
{
  LOG_INFO("SpeedWalk.initiate() called...")
  ROBOT_IN_MOTION_OUT(MotionModule) = false;
  //! Set normalized walk velocity between -1 and 1
  this->kM->setGlobalBase(RobotFeet::lFoot, LegEEs::footCenter);
  getBehaviorCast()->velocityInput.clip(-1, 1);
  addFirstStep(); //! Generate first balance shift step
  //for (const auto& fs : stepsQueue)
  //{
  //  fs->print();
  //}
  if (!walkZmpRefGen->initiate())
    return false;

  //! Generate first two steps
  //Mat drawing = Mat(Size(500, 500), CV_8UC3);
  //genNextStep(vi);
  /*auto step = this->getFrontStep();
  cout << "Step at front:" << step << endl;
  step->print();
  cv::Scalar color;
  color = step->foot == RobotFeet::lFoot ? cv::Scalar(255,0,0) : cv::Scalar(0,0,255);
  VisionUtils::drawRotatedRect(drawing, step->getFootRect(Point2f(250, 250), 100), color);
  for (int i = 0; i < 10; ++i) {
    this->addStep(genNextStep(this->getBackStep()));
    step = this->getBackStep();
    cout << "Step at front:" << step << endl;
    color = step->foot == RobotFeet::lFoot ? cv::Scalar(255,0,0) : cv::Scalar(0,0,255);
    step->print();
    VisionUtils::drawRotatedRect(drawing, step->getFootRect(Point2f(250, 250), 100), color);
    VisionUtils::displayImage("drawing", drawing, 1.0);
    cv::waitKey(0);
  }*/
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
  getBehaviorCast()->velocityInput.clip(-1, 1);
  if (!walkZmpRefGen->previewsAvailable(this->runTime)) {
    cout << "Adding new step..." << endl;
    cout << "nSteps:" << stepsQueue.size() << endl;
    genNextStep();
  }
  if (fsm->update())
    finish();
}

template <typename Scalar>
void SpeedWalk<Scalar>::PlanSteps::onRun()
{
  cout << "This->runtime:" << this->bPtr->runTime << endl;
  for (const auto& fs : this->bPtr->stepsQueue)
  {
    if (this->bPtr->runTime == fs->timeAtFinish)
      this->bPtr->popStep();
  }
  this->bPtr->walkZmpRefGen->update(this->bPtr->runTime);
  //this->genStepTrajectory();
  //this->nextState = this->bPtr->executeDSS.get();
}

template <typename Scalar>
void SpeedWalk<Scalar>::ExecuteDSS::onRun()
{
  /*
  this->solveDSSMotion();
  if (this->executeDSSMotion()) {
    this->popStep();
    this->nextState = this->bPtr->executeSSS.get();
  }
  */
}

template <typename Scalar>
void SpeedWalk<Scalar>::ExecuteSSS::onRun()
{
  /*
  this->solveSSSMotion();
  if (this->executeSSSMotion()) {
    this->nextState = this->bPtr->planSteps.get();
  }
  */
}

template <typename Scalar>
void SpeedWalk<Scalar>::finish()
{
  LOG_INFO("SpeedWalk.finish() called...")
  this->inBehavior = false;
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
  cout << "transFromTo: "<< transFromTo << endl;
  addStep(boost::make_shared<TNRSFootstep<Scalar> >(
    RobotPose2D<Scalar>(
      transFromTo(0, 3),
      transFromTo(1, 3),
      atan2(transFromTo(1, 0), transFromTo(0, 0))),
      foot,
      transFromTo,
      params->totalStepTime));
}


template <typename Scalar>
void SpeedWalk<Scalar>::genNextStep()
{
  auto poseDiff = this->stepVelToPose(getBehaviorCast()->velocityInput);
  Matrix<Scalar, 3, 3> diffRot;
  Matrix<Scalar, 4, 4> diffTrans;
  MathsUtils::makeRotationZ(diffRot, poseDiff.getTheta());
  diffTrans = MathsUtils::makeTransformation(diffRot, poseDiff.getX(), poseDiff.getY(), 0.0);
  cout << "diffTrans: " << diffTrans << endl;
  auto prevStep = getBackStep();
  if (!prevStep) { //! Robot is standing
    //! Get current foot position
    auto foot = this->kM->getGlobalBaseIndex() == RobotFeet::lFoot ? RobotFeet::rFoot : RobotFeet::lFoot;
    Matrix<Scalar, 4 ,4> transFromTo = this->kM->getGlobalToOther() * diffTrans;
    addStep(boost::make_shared<TNRSFootstep<Scalar> >(
      RobotPose2D<Scalar>(
        transFromTo(0, 3), transFromTo(1, 3), atan2(transFromTo(1, 0), transFromTo(0, 0))),
        foot,
        transFromTo,
        prevStep->timeAtFinish + params->totalStepTime));
  } else {
    //! Get current foot position
    auto foot = prevStep->foot == RobotFeet::lFoot ? RobotFeet::rFoot : RobotFeet::lFoot;
    //Matrix<Scalar, 4 ,4> transFromTo = MathsUtils::getTInverse(prevStep->trans) * diffTrans;
    diffTrans(1, 3) += this->params->getFootSeparation(foot);
    Matrix<Scalar, 4 ,4> transFromTo = diffTrans;
    addStep(boost::make_shared<TNRSFootstep<Scalar> >(
      RobotPose2D<Scalar>(
        transFromTo(0, 3), transFromTo(1, 3), atan2(transFromTo(1, 0), transFromTo(0, 0))),
        foot,
        transFromTo,
        prevStep->timeAtFinish + params->totalStepTime));
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
