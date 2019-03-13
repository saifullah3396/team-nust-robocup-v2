/**
 * @file Utils/include/BehaviorConfig.h
 *
 * This file declares the base struct for the configuration of all the
 * behaviors.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <jsoncpp/json/json.h>
#include "Utils/include/Exceptions/TNRSException.h"
#include "BehaviorConfigs/include/BehaviorConfigMacros.h"

DEFINE_ENUM_WITH_STRING_CONVERSIONS(
	BConfigExceptionType, 
	(EXC_INVALID_BCONFIG_PARAMETERS)
  (EXC_INVALID_BCONFIG)
  (INVALID_JSON_INPUT)
)

struct BehaviorConfig;

/**
 * @class BConfigException
 * @brief BehaviorConfig exception management class
 */
class BConfigException : public TNRSException
{
public:
  /**
   * Constructor
   * 
   * @param behaviorConfig in which the exception is raised
   * @param message explanatory message
   * @param bSysMsg true if the system message (from strerror(errno))
   *   should be postfixed to the user provided message
   * @param type Argument parser exception type
   */
  BConfigException(
    BehaviorConfig* config, 
		const string& message,
		const bool& bSysMsg, 
		const BConfigExceptionType& type) throw ();

  /**
   * Destructor
   */
  ~BConfigException() throw () {}
  
  string getExcPrefix();

private:
  BehaviorConfig* config;
	BConfigExceptionType type;
};

/**
 * Enumeration for all types of behavior base types
 *
 * @enum BaseBehaviorType
 */
enum class BaseBehaviorType : unsigned int
{
  motion,
  general,
  planning,
  count
};

/**
 * @struct BehaviorConfig
 * @brief A base struct for all types of behavior configurations
 */
struct BehaviorConfig
{
  /**
   * Constructor
   * 
   * @param id: Behavior id
   * @param baseType: Behavior baseType -> MOTION, STATIC, PLANNING
   * @param maxRuntime: Maximum runtime for behavior after which it should
   *   be killed
   * @param type: Sub type of the behavior
   */ 
  BehaviorConfig(
    const unsigned& id, 
    const BaseBehaviorType& baseType,
    const float& maxRuntime,
    const int& type = -1) :
    id(id),
    baseType(baseType),
    maxRuntime(maxRuntime),
    type(type),
    logData(false)
  {
  }
  
  /**
   * Destructor
   */ 
  virtual
  ~BehaviorConfig()
  {
  }

  /**
   * Throws an exception if the configuration is invalid
   * 
   * @return void
   */ 
  virtual void validate() {
    throw
      BConfigException(
        this,
        "Requested behavior configuration is invalid or undefined for use.",
        false,
        EXC_INVALID_BCONFIG
      );
  }
  
  /**
   * Makes an object of type this and returns it if valid
   */ 
  static boost::shared_ptr<BehaviorConfig> 
    makeFromJson(const Json::Value& obj);
  
  /**
   * Assigns the configuration parameters using json object
   * 
   * @param obj: Input json object with info regarding the configuration
   * 
   * @return false if an exception is raised
   */
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * Makes a json object from the configuration parameters
   * 
   * @return Json::Value
   */
  virtual Json::Value getJson();
  
  //! Behavior id as defined for the given base type
  unsigned id;
  
  //! Behavior sub type
  int type;
  
  //! Behavior base type
  BaseBehaviorType baseType;
  
  //! Maximum run time of this behavior
  float maxRuntime;

  //! Whether to log data for the behavior
  bool logData;
};

typedef boost::shared_ptr<BehaviorConfig> BehaviorConfigPtr;
