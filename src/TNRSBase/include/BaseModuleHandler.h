/**
 * @file TNRSBase/include/BaseModuleHandler.h
 *
 * This file declares and implements the class BaseModuleHandler
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 07 Nov 2017
 */

#pragma once

#include "TNRSBase/include/BaseModule.h"
#include "Utils/include/PrintUtils.h"

/**
 * @class BaseModuleHandler
 * @brief A class that provides child BaseModule vector and its handling
 *   utilities
 */
class BaseModuleHandler
{
public:
  /**
   * @brief ~BaseModuleHandler Constructor
   */
  BaseModuleHandler() = default;

  /**
   * @brief ~BaseModuleHandler Destructor
   */
  virtual ~BaseModuleHandler() {}

  /**
   * Returns the vector of child BaseModules
   *
   * @return a vector of BaseModulePtr
   */
  vector<BaseModulePtr>& getChildModules() { return childModules; }

protected:
  /**
   * @brief start Starts all child threads
   */
  void start()
  {
    for (const auto& cm : childModules) {
      if (cm) cm->start();
    }
  }

  /**
   * @brief suspend Suspends all child threads
   */
  void suspend()
  {
    for (const auto& cm : childModules) {
      if (cm) cm->suspend();
    }
  }

  /**
   * @brief resume Resumes all child threads
   */
  void resume()
  {
    for (const auto& cm : childModules) {
      if (cm) cm->resume();
    }
  }

  /**
   * @brief resume Resumes all child threads
   */
  void join()
  {
    for (const auto& cm : childModules) {
      if (cm) cm->join();
    }
    LOG_INFO("All BaseModules successfully joined...");
  }

  /**
   * @brief terminate Terminates all child threads
   */
  void terminate()
  {
    for (const auto& cm : childModules) {
      if (cm) cm->terminate();
    }
  }

  vector<BaseModulePtr> childModules; //! A vector of child BaseModules.
};
