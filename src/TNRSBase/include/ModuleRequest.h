/**
 * @file TNRSBase/include/ModuleRequest.h
 *
 * This file declares the struct ModuleRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <boost/shared_ptr.hpp>

namespace Json { class Value; }
enum class TNSPLModules : unsigned int;

/**
 * @class ModuleRequest
 * @brief A base class for defining a request. The module that receives
 *   the request performs some operation based on the id.
 */
class ModuleRequest
{
public:
  /**
   * @brief ModuleRequest Constructor
   * @param moduleId Module id
   * @param requestId Request id
   */
  ModuleRequest(
    const TNSPLModules& moduleId,
    const unsigned& requestId);

  ///< Getters
  TNSPLModules getModuleId() { return moduleId; }
  unsigned getRequestId() { return requestId; }

  ///< Setters
  void setRequestId(const unsigned& requestId)
    { this->requestId = requestId; }

  /**
   * Makes an object of type this and returns it if valid
   */
  static boost::shared_ptr<ModuleRequest> makeFromJson(const Json::Value& obj);

  /**
   * Assigns the request parameters using json object
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

private:
  TNSPLModules moduleId; ///< Module id
  unsigned requestId; ///< Request id
};
typedef boost::shared_ptr<ModuleRequest> ModuleRequestPtr;
