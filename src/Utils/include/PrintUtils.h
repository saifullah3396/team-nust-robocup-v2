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
#include <sstream>
#include <ostream>
#include <streambuf>
#include "Utils/include/ConfigManager.h"
#include "Utils/include/ThreadSafeQueue.h"
#include "Utils/include/VariadicMacros.h"

using namespace std;
#define OSS(VALUES) \
    static_cast<std::ostringstream&&>(std::ostringstream() << VALUES)

class PrintUtils
{
public:
  /**
   * @brief PrintUtils Constructor
   */
  PrintUtils();

  /**
   * @brief ~PrintUtils Destructor
   */
  ~PrintUtils();
  static void startSendingOutputOverNetwork();
  static void addMessage(const std::stringstream& stream);
  static ThreadSafeQueue<string>& getMessagesQueue() { return messagesQueue; }

  static std::stringstream stream; ///< Output log stream
  static pthread_mutex_t printMutex;
private:
  static fstream mainLog; ///< Output log file
  ///< Only used if sending messages over the network
  static ThreadSafeQueue<string> messagesQueue;
  static string fileName;
  static bool sendOutputOverNetwork;
};

#ifndef MODULE_IS_REMOTE
  #define ADD_LOG_MESSAGE(msg) \
    pthread_mutex_lock(&PrintUtils::printMutex); \
    PrintUtils::stream << msg; \
    PrintUtils::addMessage(PrintUtils::stream); \
    pthread_mutex_unlock(&PrintUtils::printMutex);

  #define LOG_INFO(msg) \
    ADD_LOG_MESSAGE("[Info] " << msg << endl)

  #define LOG_ERROR(msg) \
    ADD_LOG_MESSAGE("[Error] " << msg << endl)

  #define LOG_EXCEPTION(msg) \
    ADD_LOG_MESSAGE("[Exception] " << msg << endl)
#else
  #define LOG_INFO(msg) \
    cout << "[Info] " << msg << endl;

  #define LOG_ERROR(msg) \
    cout << "[Error] " << msg << endl;

  #define LOG_EXCEPTION(msg) \
    cout << "[Exception] " << msg << endl;
#endif
