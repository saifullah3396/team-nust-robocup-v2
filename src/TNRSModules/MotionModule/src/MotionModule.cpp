/**
 * @file MotionModule/src/MotionModule.cpp
 *
 * This file implements the class MotionModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "ControlModule/include/HardwareLayer.h"
#include "MotionModule/include/FallDetector/FallDetector.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/MBManager.h"
#include "MotionModule/include/MotionGenerator.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MotionRequest.h"
#include "MotionModule/include/JointRequest.h"
#include "MotionModule/include/HandsRequest.h"
#include "MotionModule/include/TrajectoryPlanner/TrajectoryPlanner.h"
#include "TeamNUSTSPL/include/TeamNUSTSPL.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBKickConfig.h"
#include "Utils/include/Constants.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/Landmark.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/GoalInfo.h"
#include "Utils/include/DataHolders/BehaviorInfo.h"
#include "Utils/include/HardwareIds.h"

typedef map<unsigned, BehaviorInfo> BehaviorInfoMap;

#ifndef V6_CROSS_BUILD
DEFINE_INPUT_CONNECTOR(MotionModule,
  (int, motionThreadPeriod),
  (vector<float>, jointStiffnessSensors),
  (vector<float>, touchSensors),
  (BallInfo<float>, ballInfo),
  (GoalInfo<float>, goalInfo),
  (RobotPose2D<float>, robotPose2D),
  (bool, landmarksFound),
);
DEFINE_OUTPUT_CONNECTOR(MotionModule,
  (int, motionThreadTimeTaken),
  (vector<float>, jointPositionSensors),
  (vector<float>, handSensors),
  (vector<float>, inertialSensors),
  (vector<float>, fsrSensors),
  (int, nFootsteps),
  (Matrix4f, upperCamInFeet),
  (Matrix4f, lowerCamInFeet),
  (Matrix4f, lFootOnGround),
  (Matrix4f, rFootOnGround),
  (PostureState, postureState),
  (bool, robotFallen),
  (cv::Point_<float>, kickTarget),
  (bool, robotInMotion),
  (BehaviorInfoMap, mBehaviorInfo),
  (RobotFeet, footOnGround),
  (RobotFeet, currentStepLeg),
);
#else
DEFINE_INPUT_CONNECTOR(MotionModule,
  (int, motionThreadPeriod),
  (vector<float>, jointStiffnessSensors),
  (vector<float>, touchSensors),
  #ifdef REALTIME_LOLA_AVAILABLE
  (vector<float>, jointPositionSensors),
  (vector<float>, handSensors),
  (vector<float>, inertialSensors),
  (vector<float>, fsrSensors),
  #endif
  (BallInfo<float>, ballInfo),
  (GoalInfo<float>, goalInfo),
  (RobotPose2D<float>, robotPose2D),
  (bool, landmarksFound),
);
DEFINE_OUTPUT_CONNECTOR(MotionModule,
  (int, motionThreadTimeTaken),
  #ifndef REALTIME_LOLA_AVAILABLE
  (vector<float>, jointPositionSensors),
  (vector<float>, handSensors),
  (vector<float>, inertialSensors),
  (vector<float>, fsrSensors),
  #endif
  (int, nFootsteps),
  (Matrix4f, upperCamInFeet),
  (Matrix4f, lowerCamInFeet),
  (Matrix4f, lFootOnGround),
  (Matrix4f, rFootOnGround),
  (PostureState, postureState),
  (bool, robotFallen),
  (cv::Point_<float>, kickTarget),
  (bool, robotInMotion),
  (BehaviorInfoMap, mBehaviorInfo),
  (RobotFeet, footOnGround),
  (RobotFeet, currentStepLeg),
);
#endif
#ifndef V6_CROSS_BUILD
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
      MotionModule::MotionModule(
        void* parent,
        const ALMemoryProxyPtr& memoryProxy,
        const ALDCMProxyPtr& dcmProxy,
        const ALMotionProxyPtr& motionProxy) :
        BaseModule(parent, TNSPLModules::motion, "MotionModule"),
        memoryProxy(memoryProxy),
        dcmProxy(dcmProxy),
        motionProxy(motionProxy)
      {
      }
  #else
    MotionModule::MotionModule(
      void* parent,
      const ALMemoryProxyPtr& memoryProxy,
      const ALDCMProxyPtr& dcmProxy) :
      BaseModule(parent, TNSPLModules::motion, "MotionModule"),
      memoryProxy(memoryProxy),
      dcmProxy(dcmProxy)
    {
    }
  #endif
#else
  #ifndef REALTIME_LOLA_AVAILABLE
    MotionModule::MotionModule(
      void* parent,
      const qi::AnyObject& memoryProxy,
      const qi::AnyObject& motionProxy) :
      BaseModule(parent, TNSPLModules::motion, "MotionModule"),
      memoryProxy(memoryProxy),
      motionProxy(motionProxy)
    {
    }
  #else
    MotionModule::MotionModule(void* parent) :
      BaseModule(parent, TNSPLModules::motion, "MotionModule")
    {
    }
  #endif
#endif

MotionModule::~MotionModule()
{
  delete inputConnector;
  delete outputConnector;
  kinematicsModule->cleanup();
}

void MotionModule::setThreadPeriod()
{
  setPeriodMinMS(MOTION_PERIOD_IN(MotionModule));
}

void MotionModule::setThreadTimeTaken()
{
  MOTION_TIME_TAKEN_OUT(MotionModule) = lastIterationTimeMS;
}

void MotionModule::initMemoryConn()
{
  inputConnector =
    new InputConnector(this, getModuleName() + "InputConnector");
  outputConnector =
    new OutputConnector(this, getModuleName() + "OutputConnector");
  inputConnector->initConnector();
  outputConnector->initConnector();
}

void MotionModule::init()
{
  ///< Reset output variables
  LOG_INFO("Initializing MotionModule Output Variables...")
  #ifndef V6_CROSS_BUILD
    JOINT_POSITIONS_OUT(MotionModule) = vector<float>(toUType(Joints::count), 0.f);
    HAND_SENSORS_OUT(MotionModule) = vector<float>(toUType(RobotHands::count), 0.f);
    INERTIAL_SENSORS_OUT(MotionModule) = vector<float>(toUType(InertialSensors::count), 0.f);
    INERTIAL_SENSORS_OUT(MotionModule)[toUType(InertialSensors::accelerometerX)] = 0.0;
    INERTIAL_SENSORS_OUT(MotionModule)[toUType(InertialSensors::accelerometerY)] = 0.0;
    INERTIAL_SENSORS_OUT(MotionModule)[toUType(InertialSensors::accelerometerZ)] = -Constants::gravity;
    FSR_SENSORS_OUT(MotionModule) = vector<float>(toUType(FsrSensors::count), 0.f);
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      JOINT_POSITIONS_OUT(MotionModule) = vector<float>(toUType(Joints::count), 0.f);
      HAND_SENSORS_OUT(MotionModule) = vector<float>(toUType(RobotHands::count), 0.f);
      INERTIAL_SENSORS_OUT(MotionModule) = vector<float>(toUType(InertialSensors::count), 0.f);
      INERTIAL_SENSORS_OUT(MotionModule)[toUType(InertialSensors::accelerometerX)] = 0.0;
      INERTIAL_SENSORS_OUT(MotionModule)[toUType(InertialSensors::accelerometerY)] = 0.0;
      INERTIAL_SENSORS_OUT(MotionModule)[toUType(InertialSensors::accelerometerZ)] = -Constants::gravity;
      FSR_SENSORS_OUT(MotionModule) = vector<float>(toUType(FsrSensors::count), 0.f);
    #endif
  #endif
  N_FOOTSTEPS_OUT(MotionModule) = 0;
  UPPER_CAM_TRANS_OUT(MotionModule) = Matrix4f::Identity();
  LOWER_CAM_TRANS_OUT(MotionModule) = Matrix4f::Identity();
  L_FOOT_TRANS_OUT(MotionModule) = Matrix4f::Identity();
  R_FOOT_TRANS_OUT(MotionModule) = Matrix4f::Identity();
  POSTURE_STATE_OUT(MotionModule) = PostureState::unknown;
  ROBOT_FALLEN_OUT(MotionModule) = false;
  KICK_TARGET_OUT(MotionModule) = Point2f(0.f, 0.f);
  ROBOT_IN_MOTION_OUT(MotionModule) = false;
  MB_INFO_OUT(MotionModule) = BehaviorInfoMap();
  FOOT_ON_GROUND_OUT(MotionModule) = RobotFeet::lFoot;
  STEP_LEG_OUT(MotionModule) = RobotFeet::unknown;

  #ifndef V6_CROSS_BUILD
    ///< Setup motion sensors
    LOG_INFO("Initializing motion module sensor layers...")
    setupSensors();
    ///< Setup motion actuators
    LOG_INFO("Initializing motion module actuator layers...")
    setupActuators();
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      ///< Setup motion sensors
      LOG_INFO("Initializing motion module sensor layers...")
      setupSensors();
    #endif
  #endif

  #ifndef V6_CROSS_BUILD
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
      ///< Disable NaoQi's fall manager
      LOG_INFO("Disabling Naoqi fall manager...")
      motionProxy->setFallManagerEnabled(false);
      LOG_INFO("Waking Robot Up...")
      //motionProxy->wakeUp();
      motionProxy->setMoveArmsEnabled(false, false);
    #endif
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      try {
        ///< Disable NaoQi's fall manager
        LOG_INFO("Setting setFallManagerEnabled()...")
        motionProxy.call<void>("setFallManagerEnabled", false);

        LOG_INFO("Setting wakeUp()...")
        //motionProxy.call<void>("wakeUp");

        LOG_INFO("Setting setMoveArmsEnabled()...")
        motionProxy.call<void>("setMoveArmsEnabled", false, false);
      } catch (exception& e) {
        LOG_EXCEPTION(e.what());
      }
    #endif
  #endif
  ///< Create kinematics module
  LOG_INFO("Initializing KinematicsModule...")
  kinematicsModule = boost::shared_ptr<KinematicsModule<MType>>(new KinematicsModule<MType>(this));
  kinematicsModule->init();
  ///< Create motion generator module
  LOG_INFO("Initializing MotionGenerator...")
  motionGenerator = MotionGeneratorPtr(new MotionGenerator<MType>(this));
  ///< Create fall detector
  LOG_INFO("Initializing FallDetector...")
  fallDetector = FallDetectorPtr(new FallDetector<MType>(this));
  ///< Create trajectory planner
  LOG_INFO("Initializing TrajectoryPlanner...")
  trajectoryPlanner =  TrajectoryPlannerPtr(new TrajectoryPlanner<MType>(this));
}

#ifndef V6_CROSS_BUILD
void MotionModule::setupSensors()
{
  sensorLayers.resize(toUType(MotionSensors::count));
  LOG_INFO("toUType(MotionSensors::count):"  << toUType(MotionSensors::count));
  sensorLayers[toUType(MotionSensors::jointPosition)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::joints) + toUType(JointSensorTypes::position),
      OVAR_PTR(vector<float>, MotionModule::Output::jointPositionSensors),
      memoryProxy);
  sensorLayers[toUType(MotionSensors::handSensors)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::handSensors),
      OVAR_PTR(vector<float>, MotionModule::Output::handSensors),
      memoryProxy);
  sensorLayers[toUType(MotionSensors::inertial)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::inertialSensors),
      OVAR_PTR(vector<float>, MotionModule::Output::inertialSensors),
      memoryProxy);
  sensorLayers[toUType(MotionSensors::fsr)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::fsrSensors),
      OVAR_PTR(vector<float>, MotionModule::Output::fsrSensors),
      memoryProxy);
  sensorsUpdate();
  LOG_INFO("Setting broker..");
  #ifndef MODULE_IS_REMOTE
    ((TeamNUSTSPL*) getParent())->getParentBroker()->getProxy("DCM")->getModule()->atPostProcess(boost::bind(&MotionModule::sensorsUpdate, this));
  #endif
  LOG_INFO("Setting broker..");
}
#else
  #ifndef REALTIME_LOLA_AVAILABLE
    void MotionModule::setupSensors()
    {
      sensorLayers.resize(toUType(MotionSensors::count));
      sensorLayers[toUType(MotionSensors::jointPosition)] =
        SensorLayer::makeSensorLayer(
          toUType(SensorTypes::joints) + toUType(JointSensorTypes::position),
          OVAR_PTR(vector<float>, MotionModule::Output::jointPositionSensors),
          memoryProxy);
      sensorLayers[toUType(MotionSensors::handSensors)] =
        SensorLayer::makeSensorLayer(
          toUType(SensorTypes::handSensors),
          OVAR_PTR(vector<float>, MotionModule::Output::handSensors),
          memoryProxy);
      sensorLayers[toUType(MotionSensors::inertial)] =
        SensorLayer::makeSensorLayer(
          toUType(SensorTypes::inertialSensors),
          OVAR_PTR(vector<float>, MotionModule::Output::inertialSensors),
          memoryProxy);
      sensorLayers[toUType(MotionSensors::fsr)] =
        SensorLayer::makeSensorLayer(
          toUType(SensorTypes::fsrSensors),
          OVAR_PTR(vector<float>, MotionModule::Output::fsrSensors),
          memoryProxy);
      sensorsUpdate();
    }
  #endif
#endif

#ifndef V6_CROSS_BUILD
void MotionModule::setupActuators()
{
  actuatorLayers.resize(toUType(MotionActuators::count));
    actuatorLayers[toUType(MotionActuators::jointActuators)] =
      ActuatorLayer::makeActuatorLayer(
        toUType(ActuatorTypes::jointActuators) + toUType(JointActuatorTypes::angles),
        dcmProxy);
    actuatorLayers[toUType(MotionActuators::handActuators)] =
      ActuatorLayer::makeActuatorLayer(
        toUType(ActuatorTypes::handActuators),
        dcmProxy);
  #ifndef MODULE_IS_REMOTE
    ((TeamNUSTSPL*) getParent())->getParentBroker()->getProxy("DCM")->getModule()->atPreProcess(boost::bind(&MotionModule::actuatorsUpdate, this));
  #endif
}
#else //! No actuators with/without in V6s
#endif

void MotionModule::handleRequests()
{
  //cout << "handling requests: " << inRequests.getSize() << endl;
  while(!inRequests.isEmpty()) {
    auto request = inRequests.queueFront();
    if (boost::static_pointer_cast <MotionRequest>(request)) {
      auto reqId = request->getRequestId();
      #ifndef V6_CROSS_BUILD
      //! Realtime actuator requests through MotionModule are only available in V5s
      if (reqId == toUType(MotionRequestIds::jointRequest)) {
        actuatorLayers[toUType(MotionActuators::jointActuators)]->addRequest(
          boost::static_pointer_cast<JointRequest>(request)
        );
      } else if (reqId == toUType(MotionRequestIds::handsRequest)) {
        actuatorLayers[toUType(MotionActuators::handActuators)]->addRequest(
          boost::static_pointer_cast<HandsRequest>(request)
        );
      } else if (reqId == toUType(MotionRequestIds::behaviorRequest)) {
      #else
      if (reqId == toUType(MotionRequestIds::behaviorRequest)) {
      #endif
        auto rmb =
          boost::static_pointer_cast<RequestMotionBehavior>(request);
        //cout << "handling requests id: " << rmb->getReqConfig()->id << endl;
        //cout << "handling requests type: " << rmb->getReqConfig()->type << endl;
        if (mbManagers.find(rmb->mbManagerId) != mbManagers.end()) {
          /*cout << "rmb->mbManagerIda already exists." << endl;
          auto configs = mbManagers[rmb->mbManagerId]->getBehaviorInfo().getConfigsTree();
          for (int i = 0; i < configs.size(); ++i) {
            cout << "config[" << i << "]:\n" << configs[i]->getJson() << endl;
          }*/
          mbManagers[rmb->mbManagerId]->manageRequest(rmb);
        } else {
          ///< Create motion behavior manager
          mbManagers.insert(
            pair<unsigned, MBManagerPtr>(
              rmb->mbManagerId,
              boost::shared_ptr<MBManager<MType> >(new MBManager<MType>(this))
            ));
          mbManagers[rmb->mbManagerId]->manageRequest(rmb);
        }
      } else if (reqId == toUType(MotionRequestIds::killBehavior)) {
        auto rmb =
          boost::static_pointer_cast<KillMotionBehavior>(request);
        if (mbManagers.find(rmb->mbManagerId) != mbManagers.end()) {
          mbManagers[rmb->mbManagerId]->killBehavior();
        }
      } else if (reqId == toUType(MotionRequestIds::killBehaviors)) {
        for (size_t i = 0; i < mbManagers.size(); ++i) {
          if (mbManagers[i])
            mbManagers[i]->killBehavior();
        }
      }
    }
    for (auto it = mbManagers.begin(); it != mbManagers.end(); )
    {
      it->second->update();
      MB_INFO_OUT(MotionModule).insert(
        pair<unsigned, BehaviorInfo>(it->first, it->second->getBehaviorInfo())
      );
      //auto configs = it->second->getBehaviorInfo().getConfigsTree();
      //for (int i = 0; i < configs.size(); ++i) {
      //  cout << "config[" << i << "]:\n" << configs[i]->getJson() << endl;
      //}
      if (it->second->getBehaviorInfo().isFinished()) {
        it = mbManagers.erase(it);
      } else {
        ++it;
      }
    }
    inRequests.popQueue();
  }
}

void MotionModule::mainRoutine()
{
  MB_INFO_OUT(MotionModule).clear();
  #ifndef V6_CROSS_BUILD
    #ifdef MODULE_IS_REMOTE
      sensorsUpdate();
    #endif
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      sensorsUpdate();
    #endif
  #endif
  motionGenerator->reset();
  #ifdef V6_CROSS_BUILD
  try {
    #ifndef REALTIME_LOLA_AVAILABLE
      JOINT_POSITIONS_OUT(MotionModule) = motionProxy.call<vector<float>>("getAngles", Constants::jointNames, true);
    #else
      JOINT_POSITIONS_IN(MotionModule) = motionProxy.call<vector<float>>("getAngles", Constants::jointNames, true);
    #endif
  } catch (exception& e) {
    LOG_EXCEPTION(e.what());
  }
  #endif
  kinematicsModule->update();
  fallDetector->update();
  for (auto it = mbManagers.begin(); it != mbManagers.end(); )
  {
    it->second->update();
    MB_INFO_OUT(MotionModule).insert(
      pair<unsigned, BehaviorInfo>(it->first, it->second->getBehaviorInfo())
    );
    //auto configs = it->second->getBehaviorInfo().getConfigsTree();
    //for (int i = 0; i < configs.size(); ++i) {
    //  cout << "config[" << i << "]:\n" << configs[i]->getJson() << endl;
    //}
    if (it->second->getBehaviorInfo().isFinished()) {
      it = mbManagers.erase(it);
    } else {
      ++it;
    }
  }
  motionGenerator->update();
  #ifndef V6_CROSS_BUILD
    #ifdef MODULE_IS_REMOTE
      actuatorsUpdate();
    #endif
  #else
    #ifndef REALTIME_LOLA_AVAILABLE //! No actuator in any case
    #endif
  #endif
}

#ifndef V6_CROSS_BUILD
  void MotionModule::sensorsUpdate()
  {
    for (const auto& sl : sensorLayers)
      if (sl) sl->update();
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    float steps = memoryProxy->getData("Motion/Walk/NbStep");
    N_FOOTSTEPS_OUT(MotionModule) = static_cast<int>(steps);
    #endif
  }

  void MotionModule::actuatorsUpdate()
  {
    for (const auto& al : actuatorLayers)
      if (al) al->update();
  }
#else
  #ifndef REALTIME_LOLA_AVAILABLE
    void MotionModule::sensorsUpdate()
    {
      for (const auto& sl : sensorLayers)
        if (sl) sl->update();
    }
  #endif
#endif

KinematicsModulePtr MotionModule::getKinematicsModule()
{
  return kinematicsModule;
}

MotionGeneratorPtr MotionModule::getMotionGenerator()
{
  return motionGenerator;
}

TrajectoryPlannerPtr MotionModule::getTrajectoryPlanner()
{
  return trajectoryPlanner;
}

#ifndef V6_CROSS_BUILD
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    ALMotionProxyPtr MotionModule::getSharedMotionProxy()
      { return motionProxy; }
  #endif
#else
  #ifndef REALTIME_LOLA_AVAILABLE
    qi::AnyObject MotionModule::getSharedMotionProxy()
      { return motionProxy; }
  #endif
#endif
