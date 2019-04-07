/**
 * @file PlanningBehaviors/NavigationBehavior/NavigationBehavior.cpp
 *
 * This file declares the class for the robot navigation.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 06 Oct 2017
 */

#include "BehaviorConfigs/include/PBConfigs/PBNavigationConfig.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/NavigationBehavior.h"
#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/Types/GoToTarget.h"
#include "PlanningModule/include/PlanningBehaviors/NavigationBehavior/Types/PlanTowards.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "UserCommModule/include/UserCommRequest.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "Utils/include/DataHolders/PositionInput.h"
#include "Utils/include/PathPlanner/PathPlanner.h"
#include "Utils/include/VisionUtils.h"

using namespace PathPlannerSpace;

int NavigationBehavior::planMaxTries = 3;
int NavigationBehavior::replanMaxTries = 3;

NavigationBehavior::NavigationBehavior(
  PlanningModule* planningModule,
  const boost::shared_ptr<PBNavigationConfig>& config,
  const string& name) :
  PlanningBehavior(planningModule, config, name),
  DebugBase(name, this),
  pathPlanned(false),
  pathExecuted(false),
  behaviorState(planPath),
  planFailCount(0),
  replanFailCount(0),
  startStep(0)
{
  initDebugBase();
  int tempSendFootsteps;
  int tempDrawFootsteps;
  GET_CONFIG(
    "PlanningBehaviors",
    (int, NavigationBehavior.sendFootsteps, tempSendFootsteps),
    (int, NavigationBehavior.drawFootsteps, tempDrawFootsteps),
  )
  SET_DVAR(int, sendFootsteps, tempSendFootsteps);
  SET_DVAR(int, drawFootsteps, tempDrawFootsteps);
}

boost::shared_ptr<PBNavigationConfig> NavigationBehavior::getBehaviorCast()
{
  return SPC(PBNavigationConfig, this->config);
}

boost::shared_ptr<NavigationBehavior> NavigationBehavior::getType(
  PlanningModule* planningModule, const BehaviorConfigPtr& cfg)
{
  NavigationBehavior* nb;
  switch (cfg->type) {
    case toUType(PBNavigationTypes::goToTarget):
      nb = new GoToTarget(planningModule, SPC(GoToTargetConfig, cfg)); break;
    case toUType(PBNavigationTypes::PlanTowards):
      nb = new PlanTowards(planningModule, SPC(PlanTowardsConfig, cfg)); break;
  }
  return boost::shared_ptr<NavigationBehavior >(nb);
}

void NavigationBehavior::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    //int tempSendFootsteps;
    //int tempDrawFootsteps;
    GET_CONFIG(
      "PlanningBehaviors",
    //  (int, NavigationBehavior.sendFootsteps, tempSendFootsteps),
    //  (int, NavigationBehavior.drawFootsteps, tempDrawFootsteps),
      (int, NavigationBehavior.planMaxTries, planMaxTries),
      (int, NavigationBehavior.replanMaxTries, replanMaxTries),
    )
    //SET_DVAR(int, sendFootsteps, tempSendFootsteps);
    //SET_DVAR(int, drawFootsteps, tempDrawFootsteps);

    ///< read parameters from config file:
    GET_CONFIG(
      "PathPlanner",
      (bool, PathPlanner.forwardSearch, forwardSearch),
    );
    loaded = true;
  }
}

bool NavigationBehavior::initiate()
{
  LOG_INFO("NavigationBehavior.initiate() called...")
  planFailCount = 0;
  replanFailCount = 0;
  pathPlanner = this->getPathPlanner();
  pathPlanner->updateMap();
  goal = getBehaviorCast()->goal;
  if (!setGoal(goal)) {
    if (getBehaviorCast()->reachClosest) {
      if (findPossibleGoal(goal)) {
        //LOG_INFO("NavigationBehavior.setGoal() successful. Continuing...")
      } else {
        LOG_ERROR("NavigationBehavior.setGoal() failed. Exiting...")
        return false;
      }
    } else {
      return false;
      LOG_ERROR("NavigationBehavior.setGoal() failed. Exiting...")
    }
  }
  if (!setStart()) {
    LOG_ERROR("NavigationBehavior.setStart() failed. Exiting...")
    return false;
  }
  pathPlanned = false;
  behaviorState = planPath;
  return true;
}

void NavigationBehavior::reinitiate(const BehaviorConfigPtr& cfg)
{
  LOG_INFO("NavigationBehavior reinitiation...")
  int currentStep = N_FOOTSTEPS_IN(PlanningModule) - startStep;
  if (currentStep < 6 || plannedPath[currentStep].getLeg() != RIGHT) {
    ///< Don't reinitiate unless the current step is right foot since the planner
    ///< always returns the solution with first step as right foot
    ///< Also gives reasonable time between reinitiation calls
    return;
  }

  this->config = cfg;
  if (!pathPlanned || currentStep >= plannedPath.size() - 1) {
    ///< If the path is not already planned, we just call initiate
    initiate();
  } else {
    ///< Update the map
    pathPlanner->updateMap();

    ///< Set new goal position
    bool goalSet = false;
    goal = getBehaviorCast()->goal;
    if (setGoal(goal)) {
      //LOG_INFO("NavigationBehavior.setGoal() successful. Continuing...")
      goalSet = true;
    } else {
      if (getBehaviorCast()->reachClosest) {
        pathPlanned = false;
        goalSet = findPossibleGoal(goal);
      }
    }

    ///< If a goal position is set successfully
    if (goalSet) {
      auto right = plannedPath[currentStep];
      auto left = plannedPath[currentStep - 1];
      auto step =
        State(
          right.getX() - left.getX(),
          right.getY() - left.getY(),
          right.getTheta() - left.getTheta(),
          right.getLeg()
        );
      Matrix<float, 4, 4> lFootT;
      if (getFootTransform(lFootT, LEFT)) {
        auto lTheta =
          MathsUtils::matToEuler(
            (Matrix<float, 3, 3>) lFootT.block(0, 0, 3, 3)
          )[2];
        State leftActual(lFootT(0, 3), lFootT(1, 3), lTheta, LEFT);
        State rightActual =
          State(
            leftActual.getX() + step.getX(),
            leftActual.getY() + step.getY(),
            leftActual.getTheta() + step.getTheta(),
            step.getLeg()
          );
        if (setStart(leftActual, rightActual) && replan()) {
          if (GET_DVAR(int, drawFootsteps))
            drawFootstepsData();
          if (GET_DVAR(int, sendFootsteps))
            sendFootstepsData();
          pathPlanned = true;
          planFailCount = 0;
          behaviorState = executeMotion;
        } else {
          pathPlanned = false;
          ++planFailCount;
        }
      }
    }
  }
}

void NavigationBehavior::update()
{
  if (ROBOT_FALLEN_IN(PlanningModule))
  {
    finish();
  }
  if (behaviorState == planPath) {
    planPathAction();
  } else if (behaviorState == executeMotion) {
    executeMotionAction();
  } else if (behaviorState == validatePath) {
    validatePathAction();
  }
}

void NavigationBehavior::finish()
{
  LOG_INFO("NavigationBehavior.finish() called...")
  killMotionBehavior(MOTION_1);
  inBehavior = false;
}

void NavigationBehavior::planPathAction()
{
  // Number of initial plan tries = 3
  if (forwardSearch) {
    if (replan()) {
      pathPlanned = true;
      replanFailCount = 0;
      behaviorState = executeMotion;
    } else {
      pathPlanned = false;
      if (++replanFailCount >= replanMaxTries) { // Maximum replan tries reached
        //LOG_INFO("Navigation ended cause maximum replan tries reached...")
        finish();
      }
    }
  } else {
    if (plan()) {
      if (GET_DVAR(int, drawFootsteps))
        drawFootstepsData();
      if (GET_DVAR(int, sendFootsteps))
        sendFootstepsData();
      pathPlanned = true;
      planFailCount = 0;
      behaviorState = executeMotion;
    } else {
      pathPlanned = false;
      if (++planFailCount >= planMaxTries) { // Maximum plan tries reached
        //LOG_INFO("Navigation ended cause maximum plan tries reached...")
        finish();
      }
    }
  }
}

void NavigationBehavior::validatePathAction()
{
  //cout << "NavigationBehavior.validatePathAction() called..."<<endl;
  pathPlanner->updateMap();
  int currentStep = N_FOOTSTEPS_IN(PlanningModule) - startStep;
  if (currentStep > 0
      && !ROBOT_IN_MOTION_IN(PlanningModule)) {
    if (!mbInProgress()) {
      //LOG_INFO("Navigation ended cause walk is finished...")
      finish();
    }
  } else {
    if (currentStep > 6 && currentStep == RIGHT && !checkPathValidity()) {
      //LOG_INFO("PathPlanner.checkPathValidity() failed. Path no more valid.")
      /*State right = plannedPath[currentStep];
      State left = plannedPath[currentStep - 1];
      auto step =
        State(
          right.getX() - left.getX(),
          right.getY() - left.getY(),
          right.getTheta() - left.getTheta(),
          right.getLeg()
        );
      Matrix<float, 4, 4> lFootT;
      if (getFootTransform(lFootT, LEFT)) {
        auto lTheta =
          MathsUtils::matToEuler(
            (Matrix<float, 3, 3>) lFootT.block(0, 0, 3, 3)
          )[2];
        State leftActual(lFootT(0, 3), lFootT(1, 3), lTheta, LEFT);
        State rightActual =
          State(
            leftActual.getX() + step.getX(),
            leftActual.getY() + step.getY(),
            leftActual.getTheta() + step.getTheta(),
            step.getLeg()
          );*/
      if (setStart() && replan()) {
        if (GET_DVAR(int, drawFootsteps))
          drawFootstepsData();
        if (GET_DVAR(int, sendFootsteps))
          sendFootstepsData();
        pathPlanned = true;
        replanFailCount = 0;
        behaviorState = executeMotion;
      } else {
        pathPlanned = false;
        if (++replanFailCount >= replanMaxTries) { // Maximum replan tries reached
          //LOG_INFO("Navigation ended cause replanFailCount increased max limit...")
          finish();
        }
      }
      //}
    }
  }
}

bool NavigationBehavior::setStart()
{
  Matrix<float, 4, 4> lFootT, rFootT;
  ///< get real placement of the feet
  if (!getFootTransform(lFootT, LEFT) ||
      !getFootTransform(rFootT, RIGHT))
    return false;
  auto lTheta =
    MathsUtils::matToEuler(
      (Matrix<float, 3, 3>) lFootT.block(0, 0, 3, 3)
    )[2];
  State left(lFootT(0, 3), lFootT(1, 3), lTheta, LEFT);
  auto rTheta =
    MathsUtils::matToEuler(
      (Matrix<float, 3, 3>) rFootT.block(0, 0, 3, 3)
    )[2];
  State right(rFootT(0, 3), rFootT(1, 3), rTheta, RIGHT);
  cout << "Robot standing at: "
       << left.getX() << ", " << left.getY() << ", " << left.getTheta() * 180.0 / M_PI << ", " << left.getLeg() << ", "
       << right.getX() << ", " << right.getY() << ", " << right.getTheta() * 180.0 / M_PI << ", " << right.getLeg()
       << endl;
  return pathPlanner->setStart(left, right);
}

bool NavigationBehavior::setStart(const State& left, const State& right)
{

  return pathPlanner->setStart(left, right);
}

bool NavigationBehavior::setGoal(const RobotPose2D<float>& goal)
{
  return pathPlanner->setGoal(goal.getX(), goal.getY(), goal.getTheta());
}

bool NavigationBehavior::findPossibleGoal(RobotPose2D<float>& goal)
{
  auto pose = ROBOT_POSE_2D_IN(PlanningModule);
  Matrix<float, 2, 1> unit;
  float diffStep = 0.10;
  unit[0] = pose.getX() - goal.getX();
  unit[1] = pose.getY() - goal.getY();
  unit = unit / unit.norm();
  int iter = 0;
  while (true) {
    if (iter > 10) break;
    goal.x() += unit[0] * diffStep;
    goal.y() += unit[1] * diffStep;
    if (setGoal(goal)) {
      return true;
    }
    ++iter;
  }
  return false;
}

bool NavigationBehavior::getFootTransform(
  Matrix<float, 4, 4>& foot, const PathPlannerSpace::Leg& id)
{
  Matrix<float, 4, 4> torsoFrameT;
  //if (!pathPlanned) {
    auto pose = ROBOT_POSE_2D_IN(PlanningModule);
    Matrix<float, 3, 3> rotation;
    MathsUtils::makeRotationZ(rotation, pose.getTheta());
    torsoFrameT =
      MathsUtils::makeTransformation(
        rotation,
        pose.getX(),
        pose.getY(),
        0.0
      );
  //} else {
  //  if (id == LEFT) torsoFrameT =
  //    prevRFoot *
  //    MathsUtils::getTInverse(IVAR(Matrix4f, PlanningModule::rFootOnGround));
  //  else if (id == RIGHT) torsoFrameT =
  //    prevLFoot *
  //    MathsUtils::getTInverse(IVAR(Matrix4f, PlanningModule::lFootOnGround));
  //}
  if (id == LEFT) {
    foot = torsoFrameT * L_FOOT_TRANS_IN(PlanningModule);
    //prevLFoot = foot;
  } else if (id == RIGHT) {
    foot = torsoFrameT * R_FOOT_TRANS_IN(PlanningModule);
    //prevRFoot = foot;
  }
  return true;
}

bool NavigationBehavior::checkPathValidity()
{
  if (!pathPlanner->checkGoal()) {
    //LOG_INFO("PathPlanner.checkGoal() failed. Goal not reachble anymore.")
    return false;
  }
  int currentStep = N_FOOTSTEPS_IN(PlanningModule) - startStep;
  //cout << "startStep: " << startStep << endl;
  //cout << "currentStep: " << currentStep << endl;
  //cout << "N_FOOTSTEPS_IN(PlanningModule): " << N_FOOTSTEPS_IN(PlanningModule) << endl;
  static int lostCount = 0;
  if (!ROBOT_LOCALIZED_IN(PlanningModule)) {
    lostCount++;
    if (lostCount > 10) {
      //LOG_INFO("Navigation ended cause robot lost...")
      finish();
    }
  } else {
    if (currentStep > 0) {
      auto pose = ROBOT_POSE_2D_IN(PlanningModule);
      RobotPose2D<float> diff;
      diff.x() = pose.getX() - plannedPath[currentStep-1].getX();
      diff.y() = pose.getY() - plannedPath[currentStep-1].getY();
      diff.theta() = pose.getTheta() - plannedPath[currentStep-1].getTheta();
      if (diff.getX() > 0.25 || diff.getY() > 0.25 || diff.getTheta() > 45.0 * M_PI / 180.0)
      {
        //cout << "Diff x: " << pose.getX() - plannedPath[currentStep].getX() << endl;
        //cout << "Diff y: " << pose.getY() - plannedPath[currentStep].getY() << endl;
        //cout << "Diff theta: " << (pose.getTheta() -  plannedPath[currentStep].getTheta()) * 180.0 / M_PI << endl;
        //cout << "Robot detracked..." << endl;
        return false;
      }
      /*cout << "Current x: " << plannedPath[currentStep].getX() << endl;
      cout << "Current y: " << plannedPath[currentStep].getY() << endl;
      cout << "Current theta: " << plannedPath[currentStep].getTheta() * 180.0 / M_PI << endl;
      cout << "Current state x: " << pose.getX() << endl;
      cout << "Current state y: " << pose.getY() << endl;
      cout << "Current state theta: " << pose.getTheta() * 180.0 / M_PI << endl;*/
      //cout << "Diff x: " << pose.getX() - plannedPath[currentStep].getX() << endl;
      //cout << "Diff y: " << pose.getY() - plannedPath[currentStep].getY() << endl;
      //cout << "Diff theta: " << (pose.getTheta() -  plannedPath[currentStep].getTheta()) * 180.0 / M_PI << endl;
    }
    lostCount = 0;
  }

  if (!pathPlanner->checkPathValidity()) {
    //cout << "pathinvalid" << endl;
    return false;
  }
  return true;
}

bool NavigationBehavior::plan(const bool& removeFirstStep)
{
  if (pathPlanner->plan()) {
    if (pathPlanner->getPathSize() <= 1) {
      return false;
    } else {
      plannedPath = pathPlanner->getPath();
      /*for (size_t i = 0; i < plannedPath.size(); ++i) {
        cout << "plannedPath[i]: "
             << plannedPath[i].getLeg() << ", "
             << plannedPath[i].getX() << ", "
             << plannedPath[i].getY() << ", "
             << plannedPath[i].getTheta()  * 180.0 / M_PI
             << endl;
      }*/
      startStep = N_FOOTSTEPS_IN(PlanningModule);
      //LOG_INFO("PathPlanner.plan() successful.")
      return true;
    }
  }
}

bool NavigationBehavior::replan()
{
  //bool pathExisted = pathPlanner->pathExists();
  ///< calculate path by replanning (if no planning information exists
  ///< this call is equal to pathPlanner->plan())
  if (pathPlanner->replan()) {
    LOG_INFO("PathPlanner.replan() successful.")
    if (pathPlanner->getPathSize() <= 1) {
      return false;
    } else {
      //cout << "start step: " << startStep << endl;
      startStep = N_FOOTSTEPS_IN(PlanningModule);
      /////< First step is the initial step state and does not count as a step
      plannedPath = pathPlanner->getPath();
      //plannedPath.erase(plannedPath.begin());
      return true;
    }
  }  else {
    LOG_INFO("Replanning... failed")
  }/*else if (pathExisted) {
    LOG_INFO(
      "PathPlanner.replan() failed. Reseting previous planning information.")
    if (!setStart()) {
      return false;
    }
    if (pathPlanner->plan()) {
      if (pathPlanner->getPathSize() <= 1) {
        return false;
      } else {
        startStep = N_FOOTSTEPS_IN(PlanningModule);
        return true;
      }
    }
  }*/
  return false;
}

void NavigationBehavior::drawFootstepsData()
{
  Mat image = Mat(Size(500, 350), CV_8UC3, Scalar(0));
  //cout << "Drawing to image..." << endl;
  for (size_t i = 0; i < plannedPath.size(); ++i) {
    /*cout << "plannedPath[i]: "
         << plannedPath[i].getLeg() << ", "
         << plannedPath[i].getX() << ", "
         << plannedPath[i].getY() << ", "
         << plannedPath[i].getTheta()  * 180.0 / M_PI
         << endl;*/
    RotatedRect footRect =
      RotatedRect(
        Point2f(250 + plannedPath[i].getX() * 50, 350/2 - plannedPath[i].getY() * 50),
        Size2f(0.16 * 50, 0.06 * 50),
        -plannedPath[i].getTheta() * 180 / M_PI
      );
    if (i == 0)
      VisionUtils::drawRotatedRect(image, footRect, cv::Scalar(255, 0, 0));
    else if (i == pathPlanner->getPathSize() - 2)
      VisionUtils::drawRotatedRect(image, footRect, cv::Scalar(0, 0, 255));
    else
      VisionUtils::drawRotatedRect(image, footRect, cv::Scalar(255, 255, 255));
  }
  imshow("footsteps", image);
  waitKey(10);
}

void NavigationBehavior::sendFootstepsData()
{
  if (pathPlanner->getPathSize() > 0) {
    Mat_<float> footRectsData(plannedPath.size(), 4);
    for (size_t i = 0; i < plannedPath.size(); ++i) {
      /*cout << "i: " << i << endl;
      cout << "x: " << plannedPath[i].getX() << endl;
      cout << "y: " << plannedPath[i].getY() << endl;
      cout << "theta: " << plannedPath[i].getTheta() << endl;
      cout << "leg: " << plannedPath[i].getLeg() << endl;*/
      footRectsData(i, 0) = plannedPath[i].getX();
      footRectsData(i, 1) = plannedPath[i].getY();
      footRectsData(i, 2) = -plannedPath[i].getTheta();
      footRectsData(i, 3) = plannedPath[i].getLeg();
    }
    unsigned char* ptr = footRectsData.data;
    int count = plannedPath.size() * 16;
    string footstepsStr = DataUtils::bytesToHexString(ptr, count);
    CommMessage msg(footstepsStr, CommMsgTypes::footsteps);
    SendMsgRequestPtr smr =
      boost::make_shared<SendMsgRequest>(msg);
    BaseModule::publishModuleRequest(smr);
    //cout << "footRectsData:\n" << footRectsData << endl;
  }
}
