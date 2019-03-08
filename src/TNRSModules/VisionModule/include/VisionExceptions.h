/**
 * @file VisionModule/include/VisionException.h
 *
 * This file declares the class VisionException
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#pragma once
#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>
#include "Utils/include/Exceptions/TNRSException.h"

/**
 * Enumeration for possible types of vision exceptions
 *
 * @enum VisionExceptionType
 */
DEFINE_ENUM_WITH_STRING_CONVERSIONS(
  VisionExceptionType,
  (EXC_BALL_CLASSIFIER_NOT_FOUND)
  (EXC_ROBOT_CLASSIFIER_NOT_FOUND)
)

/**
 * @class VisionException
 * @brief Vision exception management class
 */
class VisionException : public TNRSException
{
public:
  /**
   * Constructor
   *
   * @param moduleName: Module in which the exception is raised
   * @param message: Explanatory message
   * @param bSysMsg: True if the system message (from strerror(errno))
   *   should be postfixed to the user provided message
   * @param type: Vision exception type
   */
  VisionException(
    const string& moduleName,
    const string& message,
    const bool& bSysMsg,
    const VisionExceptionType& type) throw () :
    TNRSException(message, bSysMsg),
    name(moduleName),
    type(type)
  {
  }

  /**
   * Destructor
   */
  ~VisionException() throw () {}

  string getExcName() { return "VisionException"; }
  string getExcPrefix()
    { return "Exception caught in " + name + ";\n\t"; }

private:
  string name;
  VisionExceptionType type;
};
