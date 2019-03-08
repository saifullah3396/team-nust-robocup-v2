/**
 * @file Utils/include/PrintUtils.h
 *
 * This file defines the macro for handling debugging.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 07 Feb 2017
 */

#pragma once

#include <assert.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include "Utils/include/ConfigManager.h"
#include "Utils/include/VariadicMacros.h"

using namespace std;

class PrintUtils
{
public:
  /**
   * @brief PrintUtils Constructor
   */
  PrintUtils() {}

  /**
   * @brief ~PrintUtils Destructor
   */
  ~PrintUtils() { mainLog.close(); }
  static fstream mainLog; //! Output log file
};

#ifndef MODULE_IS_REMOTE
  #define LOG_INFO(msg) \
    if (PrintUtils::mainLog.is_open()) \
      PrintUtils::mainLog << "[Info] " << msg << endl;\
    else\
      PrintUtils::mainLog.open(\
        ConfigManager::getLogsDirPath() + "Output.txt",\
        std::ofstream::out | std::ofstream::trunc\
      );

  #define LOG_ERROR(msg) \
    if (PrintUtils::mainLog.is_open()) \
      PrintUtils::mainLog << "[Error] " << msg << endl;\
    else\
      PrintUtils::mainLog.open(\
        ConfigManager::getLogsDirPath() + "Output.txt",\
        std::ofstream::out | std::ofstream::trunc\
      );

  #define LOG_EXCEPTION(msg) \
    if (PrintUtils::mainLog.is_open()) \
      PrintUtils::mainLog << "[Exception] " << msg;\
    else\
      PrintUtils::mainLog.open(\
        ConfigManager::getLogsDirPath() + "Output.txt",\
        std::ofstream::out | std::ofstream::trunc\
      );
#else
  #define LOG_INFO(msg) \
    cout << "[Info] " << msg << endl;

  #define LOG_ERROR(msg) \
    cout << "[Error] " << msg << endl;

  #define LOG_EXCEPTION(msg) \
    cout << "[Exception] " << msg << endl;

#endif
