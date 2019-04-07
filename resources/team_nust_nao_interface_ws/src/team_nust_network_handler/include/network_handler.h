/**
 * @file NetworkHandler/NetworkHandler.h
 *
 * This file declares the class NetworkHandler
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <string>
#include <type_traits>
#include <vector>

class TcpClient;
class CommMessage;

/**
 * @class NetworkHandler
 * @brief Handles communication between robot and user for Team-NUST
 *   Robocup SPL software
 */
class NetworkHandler
{
public:
  /**
   * @brief NetworkHandler Constructor
   */
  NetworkHandler(
    ros::NodeHandle& nh, const std::string& robotIp);

  /**
   * @brief ~NetworkHandler Destructor
   */
  ~NetworkHandler()
  {
  }

  /**
   * @brief update Updates the network communication
   */ 
  void update();

  /**
   * @brief join Calls join on child threads along with its own thread
   */
  void join() {
    for (auto& thread : ioThreads) {
      thread.join();
    }
  }

private:
  /**
   * @brief runClients Runs the boost::io_service handling the clients in background
   * @param ioService Service to run
   */
  static void runClients(const boost::shared_ptr<boost::asio::io_service>& ioService);
  
  ///< Boost Asio based Tcp client that asynchronously handles read/write with robot
  std::vector<boost::shared_ptr<TcpClient> > clients;

  ///< Connection ports for each connection type
  std::vector<std::string> connPorts;

  ///< Boost io services for each connection type
  std::vector<boost::shared_ptr<boost::asio::io_service > > ioServices;

  ///< Boost io threads for each connection type
  std::vector<boost::thread> ioThreads;
};
