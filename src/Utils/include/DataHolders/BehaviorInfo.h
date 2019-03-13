/**
 * @file Utils/include/BehaviorInfo.h
 *
 * This file declares the base struct for the configuration of all the
 * behaviors.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include "BehaviorConfigs/include/BehaviorConfig.h"
#include "Utils/include/Exceptions/TNRSException.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/DataHolders/DataHolder.h"

/**
 * @struct BehaviorInfo
 * @brief A struct that holds information about the behavior
 */
struct BehaviorInfo : public DataHolder
{
  /**
   * Self-explanatory
   */
  void print() const final
  {
    PRINT_DATA(
      BehaviorInfo,
      (name, name),
      (fsmState, fsmState),
      (initiated, initiated),
      (running, running),
      (finished, finished),
    );
    Json::Value cfg;
    for (size_t i = 0; i < configs.size(); ++i) {
      auto cfgName = "config[" + DataUtils::varToString(i) + "]";
      JSON_ASSIGN(cfg, cfgName, configs[i]->getJson())
      cfg = cfg[cfgName];
    }
  }

  Json::Value getJson() const final
  {
    Json::Value cfg;
    for (size_t i = 0; i < configs.size(); ++i) {
      auto cfgName = "config[" + DataUtils::varToString(i) + "]";
      JSON_ASSIGN(cfg, cfgName, configs[i]->getJson())
      cfg = cfg[cfgName];
    }
    Json::Value val;
    val["ConfigTree"] = cfg;
    JSON_ASSIGN_(val, name, name);
    JSON_ASSIGN_(val, fsmState, fsmState);
    JSON_ASSIGN_(val, initiated, initiated);
    JSON_ASSIGN_(val, running, running);
    JSON_ASSIGN_(val, finished, finished);
    return val;
  }

  const bool& isInitiated() const
  {
    return initiated;
  }
  
  const bool& isRunning() const
  {
    return running;
  }

  const bool& isFinished() const
  {
    return finished;
  }
  
  const BehaviorConfigPtr& getConfig() const
  {
    return config;
  }

  void clearConfigs()
  {
    configs.clear();
  }

  void addConfig(const BehaviorConfigPtr& config)
  {
    configs.push_back(config);
  }

  const vector<BehaviorConfigPtr>& getConfigsTree() const
  {
    return configs;
  }

private:
  void setConfig(const BehaviorConfigPtr& config) {
    this->config = config;
  }
  
  void resetConfig() {
    config.reset();
  }

  void setName(const std::string& name) {
    this->name = name;
  }

  void setFsmState(const std::string& fsmState) {
    this->fsmState = fsmState;
  }

  void setInitiated(const bool& initiated) {
    this->initiated = initiated;
  }

  void setRunning(const bool& running) {
    this->running = running;
  }

  void setFinished(const bool& finished) {
    this->finished = finished;
  }

  std::string name = {"Unknown"};
  std::string fsmState = {"FSM not implemented for this behavior."};
  bool initiated = {false};
  bool running = {false};
  bool finished = {false};
  
  BehaviorConfigPtr config;
  vector<BehaviorConfigPtr> configs;
  
  //! Behavior manager can access its private members
  friend class BehaviorManager;
};

typedef boost::shared_ptr<BehaviorInfo> BehaviorInfoPtr;
