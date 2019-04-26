/**
 * @file Utils/src/PrintUtils.cpp
 *
 * This file defines the macro for handling debugging.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 07 Feb 2017
 */

#include "Utils/include/PrintUtils.h"
#include "Utils/include/ConfigManager.h"

string PrintUtils::fileName;
fstream PrintUtils::mainLog;
std::stringstream PrintUtils::stream;
pthread_mutex_t PrintUtils::printMutex = PTHREAD_MUTEX_INITIALIZER;
ThreadSafeQueue<string> PrintUtils::messagesQueue;
bool PrintUtils::sendOutputOverNetwork = false;

/**
 * @brief PrintUtils Constructor
 */
PrintUtils::PrintUtils() {}

/**
 * @brief ~PrintUtils Destructor
 */
PrintUtils::~PrintUtils() { mainLog.close(); }

void PrintUtils::startSendingOutputOverNetwork() {
  sendOutputOverNetwork = true;
  std::ifstream output(fileName);
  std::string message((std::istreambuf_iterator<char>(output)),
                   std::istreambuf_iterator<char>());
  messagesQueue.pushToQueue(message);
}

void PrintUtils::addMessage(const std::stringstream& stream) {
  if (!sendOutputOverNetwork) {
    //#ifndef MODULE_IS_LOCAL_SIMULATED
    cout << stream.str();
    //#endif
    if (mainLog.is_open()) {
      mainLog << stream.str();
    } else {
      fileName = ConfigManager::getLogsDirPath() + "Output.txt";
      mainLog.open(fileName, std::ofstream::out | std::ofstream::trunc);
      mainLog << stream.str();
    }
  } else {
    messagesQueue.pushToQueue(stream.str());
  }
  PrintUtils::stream.str("");
}
