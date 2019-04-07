/**
 * @file UserCommModule/include/UserCommModule.h
 *
 * This file declares the class UserCommModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <type_traits>
#include "TNRSBase/include/BaseIncludes.h"

class TcpServer;
class CommMessage;

/**
 * @class UserCommModule
 * @brief This class defines all the functions and algorithms for data
 *   that are necessary for communication between the robots and the game
 *   controller
 */
class UserCommModule : public BaseModule
{
  DECLARE_INPUT_CONNECTOR(
    userCommThreadPeriod
  );
  DECLARE_OUTPUT_CONNECTOR(
    heartBeat
  );
public:
  /**
   * @brief UserCommModule Constructor
   * @param teamNUSTSPL pointer to base class which is in this case
   *   the TeamNUSTSPL class
   */
  UserCommModule(void* teamNUSTSPL);

  /**
   * @brief ~UserCommModule Destructor
   */
  ~UserCommModule()
  {
    delete inputConnector;
    delete outputConnector;
  }

  /**
   * See BaseModule::init()
   */
  void init() final;

  /**
   * See BaseModule::mainRoutine()
   */
  void mainRoutine() final;

  /**
   * See BaseModule::handleRequests()
   */
  void handleRequests() final;

  /**
   * See BaseModule::initMemoryConn()
   */
  void initMemoryConn() final;

  /**
   * See BaseModule::setThreadPeriod()
   */
  void setThreadPeriod() final;

  /**
   * @brief join Calls join on child threads along with its own thread
   */
  void join() override {
    BaseModule::join();
    for (auto& thread : ioThreads) {
      thread.join();
    }
  }

private:
  /**
   * @brief runServers Runs the boost::io_service handling the server in background
   * @param ioService Service to run
   */
  static void runServers(const boost::shared_ptr<boost::asio::io_service>& ioService);

  /**
   * @brief addCommMessageToQueue Adds the communication message to out message queue
   * @param cMsg CommMessage
   */
  void addCommMessageToQueue(const CommMessage& cMsg);

  /**
   * @brief addImageToQueue Adds an image to out image queue
   * @param image Image
   */
  void addImageToQueue(
    const cv::Mat& image);

  ///< Boost Asio based Tcp server that asynchronously handles read/write from clients
  vector<boost::shared_ptr<TcpServer> > servers;

  ///< Connection ports for each connection type
  vector<unsigned short> connPorts;

  ///< Boost io services for each connection type
  vector<boost::shared_ptr<boost::asio::io_service > > ioServices;

  ///< Boost io threads for each connection type
  vector<boost::thread> ioThreads;

  ///< Fstream for log file
  fstream logsFile;
};
