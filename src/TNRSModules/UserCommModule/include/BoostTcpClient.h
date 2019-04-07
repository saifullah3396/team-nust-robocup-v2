/**
 * @file UserCommModule/include/BoostTcpClient.h
 *
 * This file defines the class TcpClient
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include "TNRSBase/include/DebugBase.h"
#include "UserCommModule/include/BoostConnection.h"
#include "UserCommModule/include/CommMessage.h"
#include "Utils/include/JsonUtils.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/ThreadSafeQueue.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <vector>
#include <boost/serialization/vector.hpp>

/**
 * @brief The TcpClient class defines a boost::asio based async tcp connection
 *   for communication with clients
 */
class TcpClient
{
public:
  /**
   * @brief TcpClient Constructor
   * @param io_service Boost::asio io service
   * @param port Connection port
   */
  TcpClient(boost::shared_ptr<boost::asio::io_service>& io_service, unsigned short port)
    : acceptor(*io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
  {
    //! Setup a new connection
    connection_ptr conn(new connection(acceptor.get_io_service()));

    //! Start listening on the connection
    acceptor.async_accept(
      conn->socket(),
      boost::bind(
        &TcpClient::handleAccept, this, boost::asio::placeholders::error, conn));
  }

  /**
   * @brief handleAccept Called when a new connection is accepted. Stores the accepted
   *   connection.
   * @param e Boost system error code
   * @param conn Accepted connection
   */
  void handleAccept(const boost::system::error_code& e, const connection_ptr& conn)
  {
    if (!e)
    {
      conns.push_back(conn);
      connection_ptr newConn(new connection(acceptor.get_io_service()));
      acceptor.async_accept(newConn->socket(),
          boost::bind(&TcpClient::handleAccept, this,
            boost::asio::placeholders::error, newConn));
    } else {
      std::cerr << e.message() << std::endl;
    }
  }

  /**
   * @brief handleWrite Called when some data is written to connection
   * @param e Boost system error code
   * @param conn Associated connection
   */
  void handleWrite(const boost::system::error_code& e, const connection_ptr& conn)
  {
    if (e) {
      std::cerr << e.message() << std::endl;
    }
  }

  /**
   * @brief handleRead Called when some data is read from connection. The write
   *   function is called here immediately after async_read call.
   * @param e Boost system error code
   * @param conn Associated connection
   */
  void handleRead(const boost::system::error_code& e, connection_ptr& conn)
  {
    if (!e) {
      std::cout << "Message received:" << readMsg.c_str() << endl;
      Json::Value root;
      Json::Reader reader;
      bool parsingSuccessful = reader.parse(readMsg.c_str(), root);
      if (!parsingSuccessful)
      {
        std::cout  << "Failed to parse" << reader.getFormattedErrorMessages();
      } else {
        //! If the input string is a json string
        //! Find if a user command is received
        //auto userCmdKey = DataUtils::varToString(static_cast<int>(CommMsgTypes::USER_CMD));
        //if (root[userCmdKey] != Json::nullValue) {
        //  Json::Value reply = DebugBase::processDebugMsg(root[userCmdKey].asString());
        //  //! Generate a reply
        //  addMessage(CommMessage(reply, CommMsgTypes::LOG_TEXT));
        //}
      }

      //! Write all the data to connection
      conn->async_write(
        sendMsg,
        boost::bind(&TcpClient::handleWrite, this, boost::asio::placeholders::error, conn));
    } else {
      if (
        (boost::asio::error::eof == e) ||
        (boost::asio::error::connection_reset == e))
      {
        //! Disconnected from the client. Remove the associated connection
        std::cerr << e.message() << std::endl;
        for (auto it = conns.begin(); it != conns.end(); ) {
          if ((*it) == conn) {
            cout << conn.get() << endl;
            it = conns.erase(it);
          } else {
            ++it;
          }
        }
      }
    }
  }

  /**
   * @brief update Updates the server read/write cycle
   */
  void update() {
    if (!conns.empty()) {
      Json::Value msg;
      while (!sendDataQueue.isEmpty()) {
        auto tcpMsg = sendDataQueue.queueFront();
        auto key = DataUtils::varToString(static_cast<int>(tcpMsg.type));
        msg[key] = tcpMsg.json;
        sendDataQueue.popQueue();
      }
      cout << "Sending message:" << sendMsg << endl;
      sendMsg = JsonUtils::jsonToMinimalString(msg);
    }

    for (auto& conn : conns) {
      conn->async_read(
        readMsg, boost::bind(&TcpClient::handleRead, this, boost::asio::placeholders::error, conn));
    }
  }

  /**
   * @brief addMessage Pushes a communication message to queue
   * @param msg Message
   */
  void addMessage(const CommMessage& msg) {
    sendDataQueue.pushToQueue(msg);
  }

private:
  //! The acceptor object used to accept incoming socket connections.
  boost::asio::ip::tcp::acceptor acceptor;

  //! Queue containing all the messages to be sent to the connected clients
  ThreadSafeQueue<CommMessage> sendDataQueue;

  //! A vector of connections that are alive
  vector<connection_ptr> conns;

  //! Message that is to be sent to all clients
  string sendMsg;

  //! Message that is read from all the clients
  string readMsg;

  //! Json parser for recieved messages
  Json::Reader reader;
};
