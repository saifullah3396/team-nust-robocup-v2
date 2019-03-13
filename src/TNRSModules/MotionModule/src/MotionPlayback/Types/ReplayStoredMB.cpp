/**
 * @file MotionModule/MotionPlayback/Types/ReplayStoredMB.h
 *
 * This file implements the class ReplayStoredMB
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MotionPlayback/Types/ReplayStoredMB.h"
#include "MotionModule/include/JointRequest.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/MBConfigs/MBMotionPlaybackConfig.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "Utils/include/JsonUtils.h"
#include "Utils/include/JsonLogger.h"
#include "Utils/include/Constants.h"

template <typename Scalar>
ReplayStoredMBConfigPtr ReplayStoredMB<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<ReplayStoredMBConfig> (this->config);
}

template <typename Scalar>
bool ReplayStoredMB<Scalar>::initiate()
{
  try {
    LOG_INFO("ReplayStoredMB.initiate() called...");
    string mbLogPath;
    mbLogPath = ConfigManager::getLogsDirPath() + getBehaviorCast()->pathToMB;
    LOG_INFO("Loading json log file: " << mbLogPath)
    ifstream logFile(mbLogPath, ifstream::binary);
    Json::Value json;
    logFile >> json;
    auto state = json[Constants::jointNames[0]]["state"];
    jointTimes.resize(state.size());
    jointCmds.resize(state.size(), toUType(Joints::count));
    for (int i = 0; i < state.size(); ++i) {
      jointTimes[i] = static_cast<Scalar>(state[i]["time"].asFloat());
    }
    for (int j = 0; j < toUType(Joints::count); ++j) {
      state = json[Constants::jointNames[j]]["state"];
      for (int i = 0; i < state.size(); ++i) {
        auto value = static_cast<Scalar>(state[i]["cmd"].asFloat());
        if (value == Json::nullValue)
          jointCmds(i, j) = NAN;
        else
          jointCmds(i, j) = value;
      }
    }
    jointTimes = (jointTimes.array() - jointTimes[0]).matrix();
    Matrix<Scalar, Dynamic, 1> filtTimes;
    filtTimes.resize(jointTimes.rows());
    filtTimes[0] = 0.0;
    for (int i = 1; i < jointTimes.rows(); ++i) {
      filtTimes[i] = jointTimes[i] - jointTimes[i-1] < this->cycleTime * 3 ? filtTimes[i-1] + this->cycleTime : jointTimes[i];
    }
    jointTimes = filtTimes;

    if (this->config->logData) {
      JSON_ASSIGN(this->dataLogger->getRoot(), "logFile", mbLogPath);
    }
    return true;
  } catch (const Json::Exception& e) {
    LOG_EXCEPTION(e.what());
    return false;
  }
}

template <typename Scalar>
void ReplayStoredMB<Scalar>::update()
{
  if (step >= jointTimes.size()) {
    finish();
  } else {
    Matrix<Scalar, Dynamic, 1> jr(toUType(Joints::count));
    auto aj = getBehaviorCast()->activeJoints;
    for (int i = 0; i < toUType(Joints::count); ++i) {
      if (aj[i]) jr[i] = jointCmds(step, i);
    }
    this->setJointCmds(jr);
    step++;
  }
}

template <typename Scalar>
void ReplayStoredMB<Scalar>::finish()
{
  LOG_INFO("ReplayStoredMB.finish() called...")
  POSTURE_STATE_OUT(MotionModule) = PostureState::unknown;
  this->inBehavior = false;
}

template class ReplayStoredMB<MType>;
