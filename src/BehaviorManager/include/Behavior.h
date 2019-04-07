/**
 * @file BehaviorManager/include/Behavior.h
 *
 * This file declares the classes Behavior and BehaviorException
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#pragma once
#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>
#include <json/json.h>
#include "Utils/include/Exceptions/TNRSException.h"

#define SPC(type, x) boost::static_pointer_cast<type>(x)

class Behavior;
typedef boost::shared_ptr<Behavior> BehaviorPtr;
namespace Utils {
  class JsonLogger;
}
typedef boost::shared_ptr<Utils::JsonLogger> JsonLoggerPtr;
struct BehaviorConfig;
typedef boost::shared_ptr<BehaviorConfig> BehaviorConfigPtr;
struct BehaviorRequest;
typedef boost::shared_ptr<BehaviorRequest> BehaviorRequestPtr;

/**
 * @class BehaviorException
 * @brief Behavior exception management class
 */
class BehaviorException : public TNRSException
{
public:
  /**
   * @brief BehaviorException Constructor
   *
   * @param behavior: In which the exception is raised
   * @param message: Explanatory message
   * @param bSysMsg: True if the system message (from strerror(errno))
   *   should be postfixed to the user provided message
   */
  BehaviorException(
    Behavior* behavior,
    const string& message,
    const bool& bSysMsg
  ) throw ();

  /**
   * @brief ~BehaviorException Destructor
   */
  ~BehaviorException() throw () {}

  /**
   * @brief getExcPrefix Returns a prefix for the exception message displayed
   * @return string
   */
  string getExcPrefix()
    { return "Exception caught in behavior \n\t" + name + ";\t"; }

private:
  string name; ///< Behavior name
};

/**
 * @class Behavior
 * @brief The base class for all types of behaviors
 */
class Behavior
{
public:
  /**
   * @brief Behavior Constructor
   * @param config Configuration for this behavior
   * @param name Name of the behavior
   */
  Behavior(
    const BehaviorConfigPtr& config,
    const string& name = "UnnamedBehavior");

  /**
   * @brief ~Behavior Destructor
   */
  virtual ~Behavior() {}

  /**
   * @brief manage Manages the behavior initiation and update cycle
   */
  void manage();

  /**
   * @brief reinitiate Reinitiates the behavior with the given configuration
   *
   * @param cfg The new behavior configuration
   */
  virtual void reinitiate(const BehaviorConfigPtr& cfg) {}

  /**
   * @brief setupChildRequest Sets up a child behavior request
   *
   * @param config Child behavior configuration
   * @param runInParallel To run the child alongside this behavior or not
   */
  void setupChildRequest(
    const BehaviorConfigPtr& config,
    const bool& runInParallel = false);

  /**
   * @brief onMaxTimeReached Finishes the behavior if it
   *   reaches its maximum time limit
   */
  void onMaxTimeReached();

  /**
   * @brief kill Kills the behavior with last child killed first
   */
  void kill();

  /**
   * @brief killChild Kills the child
   */
  void killChild();

  /**
   * @brief pause Pauses the behavior until next unpause call
   */
  void pause();

  /**
   * @brief un pause Unpauses the behavior
   */
  void unpause();

  /**
   * @brief pauseChild Pauses the child until next unpause call
   */
  void pauseChild();

  /**
   * @brief unPauseChild Unpauses the child
   */
  void unpauseChild();

  /**
   * @brief setLoggerFromParent Sets the data logger of this behavior
   *   from parent if it exists
   */
  void setLoggerFromParent();

  ///< Getters
  string getName() { return name; }
  virtual string getFsmState() {
    return "Finite state machine not implemented for this behavior."; }
  const bool& isInitiated() { return initiated; }
  const bool& isRunning() const { return inBehavior; }
  const bool& isPaused() const { return paused; }
  bool getChildInParallel() { return childInParallel; }
  JsonLoggerPtr getDataLogger() { return dataLogger; }
  const BehaviorConfigPtr& getBehaviorConfig() { return config; }
  BehaviorPtr& getChild() { return child; }
  BehaviorRequestPtr& getChildRequest() { return childReq; }
  unsigned& getLastChildReqId() { return lastChildReqId; }

  ///< Setters
  void setLastChildCfg(const BehaviorConfigPtr& cfg) {
    lastChildCfg = cfg;
  }
  bool setChildInParallel(const bool& childInParallel)
   { this->childInParallel = childInParallel; }
  void setParent(const BehaviorPtr& parent) { this->parent = parent; }

protected:
  /**
   * @brief initiate Initiates the behavior
   * @return True if successfully initiated
   */
  virtual bool initiate() = 0;

  /**
   * @brief update Behavior update cycle
   */
  virtual void update() = 0;

  /**
   * @brief finish Performs behavior cleanup operations
   */
  virtual void finish() = 0;

  /**
   * @brief loadExternalConfig Called once before initiation of the behavior
   */
  virtual void loadExternalConfig() = 0;

  ///< Name of the behavior
  string name = {"UnnamedBehavior"};

  ///< Time taken by behavior initiation
  float initTime = {0.0};

  ///< Total behavior runtime
  float runTime = {0.0};

  ///< Cycle time of this behavior
  float cycleTime = {0.0};

  ///< Whether the behavior is running
  bool inBehavior = {false};

  ///< Configuration of the behavior
  BehaviorConfigPtr config;

  ///< Config of last child behavior accepted
  BehaviorConfigPtr lastChildCfg;

  ///< Log directory path
  string logsDirPath;

  ///< Data logger for this behavior
  JsonLoggerPtr dataLogger;

private:
  /**
   * @brief setupLogsDir Creates a directory for storing logs.
   *
   * @return Returns false if unsuccessful
   */
  bool setupLogsDir();

  /**
   * @brief makeLogger Creates a json based logger for the behavior
   */
  virtual JsonLoggerPtr makeLogger();

  /**
   * @brief Can be derived in the child classes to log data after each
   *   update cycle as desired
   */
  virtual void updateDataLogger() {}

  ///< Whether behavior has been initiated
  bool initiated = {false};

  ///< Whether the behavior is currently paused
  bool paused = {false};

  ///< Whether the child should run in parallel or not
  bool childInParallel = {false};

  ///< Parent of this behavior
  BehaviorPtr parent;

  ///< Child of this behavior
  BehaviorPtr child;

  ///< Current child behavior request
  BehaviorRequestPtr childReq;

  ///< Previous child request id
  unsigned lastChildReqId;

  ///< Maximum time a behavior can have to perform setup
  //static constexpr float maxBehaviorSetupTime = 6.f;
};

typedef boost::shared_ptr<Behavior> BehaviorPtr;
