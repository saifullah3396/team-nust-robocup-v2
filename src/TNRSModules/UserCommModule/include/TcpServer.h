/**
 * @file UserCommModule/include/TcpServer.h
 *
 * This file defines the class TcpServer
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <string>
#include <opencv2/core/core.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <vector>
#include <boost/serialization/vector.hpp>
#include "UserCommModule/include/BoostConnection.h"
#include "Utils/include/JsonUtils.h"

template <typename T>
class ThreadSafeQueue;
class CommMessage;

/**
 * @brief The TcpServer class defines a boost::asio based async tcp connection
 *   for communication with clients
 */
class TcpServer
{
public:
  /**
   * @brief TcpServer Constructor
   * @param io_service Boost::asio io service
   * @param port Connection port
   */
  TcpServer(boost::shared_ptr<boost::asio::io_service>& io_service, unsigned short port);

  /**
   * @brief ~TcpServer Destructor
   */
  virtual ~TcpServer() {}

  /**
   * @brief handleAccept Called when a new connection is accepted. Stores the accepted
   *   connection.
   * @param e Boost system error code
   * @param conn Accepted connection
   */
  void handleAccept(const boost::system::error_code& e, const ConnectionPtr& conn);

  /**
   * @brief update Updates the server read/write cycle
   */
  virtual void update() = 0;

  /**
   * @brief handleWrite Called when some data is written to connection
   * @param e Boost system error code
   * @param conn Associated connection
   */
  virtual void handleWrite(const boost::system::error_code& e, const ConnectionPtr& conn) = 0;

  /**
   * @brief handleRead Called when some data is read from connection. The write
   *   function is called here immediately after async_read call.
   * @param e Boost system error code
   * @param conn Associated connection
   */
  virtual void handleRead(const boost::system::error_code& e, ConnectionPtr& conn) = 0;

protected:
  ///< The acceptor object used to accept incoming socket connections.
  boost::asio::ip::tcp::acceptor acceptor;

  ///< A vector of connections that are alive
  std::vector<ConnectionPtr> conns;
};

/**
 * @class DataServer
 * @brief A TcpServer that streams data
 */
class DataServer : public TcpServer
{
public:
  /**
   * @brief DataServer Constructor
   * @param io_service Boost::asio io service
   * @param port Connection port
   */
  DataServer(boost::shared_ptr<boost::asio::io_service>& io_service, unsigned short port);

  /**
   * @brief ~DataServer Destructor
   */
  virtual ~DataServer() final;

  /**
   * @brief update See TcpServer::update
   */
  void update() final;

  /**
   * @brief handleWrite See TcpServer::handleWrite
   */
  void handleWrite(const boost::system::error_code& e, const ConnectionPtr& conn) final;

  /**
   * @brief handleRead See TcpServer::handleRead
   */
  void handleRead(const boost::system::error_code& e, ConnectionPtr& conn) final;

  /**
   * @brief addMessage Pushes a communication message to queue
   * @param msg Message
   */
  void addMessage(const CommMessage& msg);

private:
  ///< Queue containing all the messages to be sent to the connected clients
  ThreadSafeQueue<CommMessage>* outDataQueue;

  ///< Json parser for recieved messages
  Json::Reader reader;

  ///< Message that is to be sent to all clients
  std::string outMessage;

  ///< Message that is read from all the clients
  std::string inMessage;
};

/**
 * @class ImageServer
 * @brief A TcpServer that streams images
 */
class ImageServer : public TcpServer
{
public:
  /**
   * @brief ImageServer Constructor
   * @param io_service Boost::asio io service
   * @param port Connection port
   */
  ImageServer(boost::shared_ptr<boost::asio::io_service>& io_service, unsigned short port);

  /**
   * @brief ~ImageServer Destructor
   */
  virtual ~ImageServer() final;

  /**
   * @brief handleWrite See TcpServer::handleWrite
   */
  void handleWrite(const boost::system::error_code& e, const ConnectionPtr& conn) final;

  /**
   * @brief handleRead See TcpServer::handleRead
   */
  void handleRead(const boost::system::error_code& e, ConnectionPtr& conn) final;

  /**
   * @brief update See TcpServer::update
   */
  void update() final;

  /**
   * @brief addImage Pushes an image to queue
   * @param image Imaged
   */
  void addImage(const cv::Mat& image);

private:
  ///< Queue containing all the messages to be sent to the connected clients
  ThreadSafeQueue<cv::Mat>* outImageQueue;

  ///< Out image
  string outMessage;
};
