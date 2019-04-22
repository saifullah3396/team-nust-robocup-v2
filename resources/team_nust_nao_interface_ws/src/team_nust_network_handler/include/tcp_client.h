/**
 * @file UserCommModule/include/TcpClient.h
 *
 * This file defines the class TcpClient
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/serialization/vector.hpp>
#include <iostream>
#include <std_msgs/String.h>
#include <vector>
#include "UserCommModule/include/BoostConnection.h"
#include "Utils/include/JsonUtils.h"

template <typename T>
class ThreadSafeQueue;
class CommMessage;

/**
 * @class TcpClient
 * @brief Defines a tcp client with asynchronous communication capability
 *   based on boost::asio
 */
class TcpClient
{
public:
  /**
   * @brief DataClient Constructor
   * @param ioService Boost::asio io service
   * @param host Server ip address
   * @param port Connection port
   */
  TcpClient(
    boost::shared_ptr<boost::asio::io_service>& ioService,
    const std::string& host,
    const std::string& port);

  /**
   * @brief ~TcpClient Destructor
   */
  virtual ~TcpClient() {}

  /**
   * @brief handleConnect Called when a new connection is made with
   *   the server. Sends a response to the server.
   * @param e Boost system error code
   * @param endpoint_iterator Connection endpoint iterator
   */
  void handleConnect(const boost::system::error_code& e,
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator);

  /**
   * @brief handleWrite Called when some data is written to connection
   * @param e Boost system error code
   * @param conn Associated connection
   */
  virtual void handleWrite(const boost::system::error_code& e, const bool& readAfter) = 0;

  /**
   * @brief handleRead Called when some data is read from connection. The write
   *   function is called here immediately after async_read call.
   * @param e Boost system error code
   * @param conn Associated connection
   */
  virtual void handleRead(const boost::system::error_code& e) = 0;

  /**
   * @brief update Updates the server read/write cycle
   */
  virtual void update() = 0;

  /**
   * @brief onConnection Callback for successful connection to server
   */
  virtual void onConnection() {}

protected:
  /**
   * @brief reconnect Starts reconnection routine if connection is lost
   */
  void reconnect();

  ///< Boost io service for this client
  boost::shared_ptr<boost::asio::io_service> ioService;

  ///< A vector of connections that are alive
  Connection conn;

  ///< Robot server ip
  string host;

  ///< Robot server port
  string port;

  ///< Whether we are currently reconnecting
  bool reconnecting = {false};
};

/**
 * @class DataClient
 * @brief A TcpClient that handles received data
 */
class DataClient : public TcpClient
{
public:
  /**
   * @brief DataClient Constructor
   * @param ioService Boost::asio io service
   * @param host Server ip address
   * @param port Connection port
   * @param nh ROS node handle
   * @param publisher_name ROS Data publisher name
   */
  DataClient(
    boost::shared_ptr<boost::asio::io_service>& ioService,
    const std::string& host,
    const std::string& port,
    ros::NodeHandle& nh,
    const std::string& publisher_name);

  /**
   * @brief handleWrite See TcpClient::handleWrite
   */
  void handleWrite(const boost::system::error_code& e, const bool& readAfter) final;

  /**
   * @brief handleRead See TcpClient::handleRead
   */
  void handleRead(const boost::system::error_code& e) final;

  /**
   * @brief update See TcpClient::update
   */
  void update() final;

  /**
   * @brief onConnection See TcpClient::onConnection
   */
  void onConnection() final;

  /**
   * @brief commandToMessage Converts a user command to an outMessage
   * @param msg User command topic
   */
  void commandToMessage(const std_msgs::String::ConstPtr& msg);

private:
  void parseJSONMessage(const Json::Value& jsonMessage);

  ///< Message that is to be sent to all clients
  std::string outMessage;

  ///< Message that is read from all the clients
  std::string inMessage;

  ///< Json parser for recieved messages
  Json::Reader reader;

  ///< Network data publisher
  ros::Publisher dataPublisher;

  ///< Network data subscriber
  ros::Subscriber dataSubscriber;

  ///< Ros node handler
  ros::NodeHandle nh;
};

/**
 * @class ImageClient
 * @brief A TcpClient that handles received image data
 */
class ImageClient : public TcpClient
{
public:
  /**
   * @brief ImageClient Constructor
   * @param ioService Boost::asio io service
   * @param host Server ip address
   * @param port Connection port
   * @param nh ROS node handle
   * @param publisher_name ROS Data publisher name
   */
  ImageClient(
    boost::shared_ptr<boost::asio::io_service>& ioService,
    const std::string& host,
    const std::string& port,
    ros::NodeHandle& nh,
    const std::string& publisher_name);

  /**
   * @brief ~ImageClient Destructor
   */
  ~ImageClient() final {}

  /**
   * @brief handleWrite See TcpClient::handleWrite
   */
  void handleWrite(const boost::system::error_code& e, const bool& readAfter) final;

  /**
   * @brief handleRead See TcpClient::handleRead
   */
  void handleRead(const boost::system::error_code& e) final;

  /**
   * @brief update See TcpClient::update
   */
  void update() final;

private:
  ///< Message that is read from all the clients
  std::string inMessage;

  ///< Network data publisher
  ros::Publisher dataPublisher;

  ///< Ros node handler
  ros::NodeHandle nh;
};
