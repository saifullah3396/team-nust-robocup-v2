/**
 * @file PlanningBehaviors/KickSequence/Types/BallIntercept.cpp
 *
 * This file implements the class BallIntercept.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "PlanningModule/include/PlanningBehaviors/KickSequence/Types/BallIntercept.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBKickConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "BehaviorConfigs/include/GBConfigs/GBStiffnessConfig.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/Solvers/MotionEquationSolver.h"
#include "Utils/include/VisionUtils.h"
#include "VisionModule/include/VisionRequest.h"
#include <opencv2/core/eigen.hpp>

#ifdef SIMULATION
float BallIntercept::coeffDamping;
#else
float BallIntercept::rollingFriction;
#endif

BallInterceptConfigPtr BallIntercept::getBehaviorCast()
{
  return boost::static_pointer_cast <BallInterceptConfig> (config);
}

void BallIntercept::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    #ifdef SIMULATION
    GET_CONFIG("EnvProperties",
      (float, coeffDamping, coeffDamping),
    )
    #else
    GET_CONFIG("EnvProperties",
      (float, coeffRF, rollingFriction),
    )
    #endif
    loaded = true;
  }
}

void
BallIntercept::initiate()
{
  LOG_INFO("BallIntercept.initiate()...")
  inBehavior = true;
}

void
BallIntercept::update()
{
  //LOG_INFO("BallIntercept.update()...")
  if (requestInProgress()) return;
  behaviorState = ballIncoming;
  if (behaviorState == startup) {
    startupAction();
  } else if (behaviorState == ballIncoming) {
    ballIncomingAction();
  }
}

void BallIntercept::finish()
{
  inBehavior = false;
}

void BallIntercept::startupAction()
{
  //LOG_INFO("BallIntercept.startupAction()...")
  setPostureAndStiffness(PostureState::STAND, StiffnessState::robocup, 0, 2.0);
}

void BallIntercept::ballIncomingAction()
{
  auto bInfo = BALL_INFO_IN(PlanningModule);
  #ifdef SIMULATION
  bInfo.posRel.x = 2.9642791748047; // Fixed ball 
  bInfo.posRel.y = -0.049993824958801;
  // velocity after ball starts rolling an damping equation starts to apply
  bInfo.velRel.x = -1.0867657661438;
  bInfo.velRel.y = 0.f;
  auto timeAtEstimation = high_resolution_clock::now();
  auto targetRight = Vector2f(0.15f, -0.05f);
  auto targetLeft = Vector2f(0.15f, 0.05f);
  auto dmeSolver = DampedMESolver(
    targetRight, 
    Vector2f (bInfo.posRel.x, bInfo.posRel.y), 
    Vector2f (bInfo.velRel.x, bInfo.velRel.y), 
    coeffDamping
  );
  dmeSolver.optDef();
  if (dmeSolver.getSuccess()) {
    auto position = dmeSolver.getEndPosition();
    auto velocity = dmeSolver.getEndVelocity();
    auto timeToReach = dmeSolver.getTimeToReach();
    cout << "position: " << position << endl;
    cout << "velocity: " << velocity << endl;
    //cout << "timeToReach: " << timeToReach << endl; 
    //cout << "dist: " << dmeSolver.getDistFromTarget() << endl; 
    if (dmeSolver.getDistFromTarget() < 0.02 &&
        timeToReach > 4.f) 
    {
      if (!mbInProgress()) {
        auto kConfig =
          boost::make_shared <JSE2DImpKickConfig> (
            Point2f(position[0], position[1]),
            Point2f(velocity[0], velocity[1]),
            timeToReach,
            timeAtEstimation,
            boost::make_shared<InterpToPostureConfig>(PostureState::STAND, 1.0f),
            boost::make_shared<MPComControlConfig>(CHAIN_L_LEG, 1.0f)
          );
        kConfig->target = Point2f(position[0]+1e-6, position[1]);
        setupMBRequest(kConfig);
        inBehavior = false;
      }
    }
  }
  #else
  bInfo.posRel.x = 3; // Fixed ball 
  bInfo.posRel.y = -0.05;
  bInfo.velRel.x = -1.5;
  bInfo.velRel.y = 0.f;
  static bool comToSupport = false;
  auto timeAtEstimation = high_resolution_clock::now();
  auto targetRight = Vector2f(0.15f, -0.05f);
  auto targetLeft = Vector2f(0.15f, 0.05f);
  auto dmeSolver = FrictionMESolver(
    targetRight, 
    Vector2f (bInfo.posRel.x, bInfo.posRel.y), 
    Vector2f (bInfo.velRel.x, bInfo.velRel.y), 
    rollingFriction
  );
  //cout << "bInfo.posRel: " << bInfo.posRel << endl;
  //cout << "bInfo.velRel: " << bInfo.velRel << endl;
  dmeSolver.optDef();
  if (dmeSolver.getSuccess()) {
    auto position = dmeSolver.getEndPosition();
    auto velocity = dmeSolver.getEndVelocity();
    auto timeToReach = dmeSolver.getTimeToReach();
    //cout << "position: " << position << endl;
    //cout << "velocity: " << velocity << endl;
    //cout << "timeToReach: " << timeToReach << endl; 
    if (dmeSolver.getDistFromTarget() < 1.f)
      //cout << "timeToReach: " << timeToReach << endl;
    if (dmeSolver.getDistFromTarget() < 0.02 && timeToReach >= 2.f && timeToReach < 4.f) {
      unsigned supportLeg;
      if ( // Near right foot
        position[0] > targetRight[0] - 0.05f &&
        position[0] - targetRight[0] < 0.05f &&
        position[1] - targetRight[1] > -0.025f &&
        position[1] - targetRight[1] < 0.05f) 
      {
        supportLeg = CHAIN_L_LEG;
        
      } else if ( // Near left foot
        position[0] > targetLeft[0] - 0.05f &&
        position[0] - targetLeft[0] < 0.05f &&
        position[1] - targetLeft[1] < 0.025f &&
        position[1] - targetLeft[1] > -0.05f) 
      {
        supportLeg = CHAIN_R_LEG;
      }
      //cout << "supportLeg: " << supportLeg << endl;
      //cout << "position: " << position << endl;
      if (!comToSupport) {
        if (!mbInProgress()) {
          auto bConfig = 
            boost::make_shared<MPComControlConfig>(supportLeg, 1.0f);
          setupMBRequest(MOTION_1, bConfig);
        }
        comToSupport = true;
      }
    } else if (timeToReach < 1.0f && comToSupport) {
      cout << "Balanced" << endl;
      return;
      if (!mbInProgress()) {
        auto kConfig =
          boost::make_shared <JSE2DImpKickConfig> (
            Point2f(position[0], position[1]),
            Point2f(velocity[0], velocity[1]),
            timeToReach,
            timeAtEstimation
          );
        kConfig->target = Point2f(1.f, -0.05f);
        setupMBRequest(MOTION_1, kConfig);
        inBehavior = false;
      }
    } else {
      cout << "Not enough time" << endl;
    }
  }
  #endif
}

