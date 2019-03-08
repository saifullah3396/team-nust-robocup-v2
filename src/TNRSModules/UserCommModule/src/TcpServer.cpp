/**
 * @file UserCommModule/src/TcpServer.cpp
 *
 * This file implements the class TcpServer and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "TNRSBase/include/DebugBase.h"
#include "UserCommModule/include/CommMessage.h"
#include "UserCommModule/include/TcpServer.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/ThreadSafeQueue.h"
#include "Utils/include/VisionUtils.h"
#include "Utils/include/ZLIBCompression.h"

TcpServer::TcpServer(
  boost::shared_ptr<boost::asio::io_service>& io_service,
  unsigned short port) :
  acceptor(*io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
{
  //! Setup a new connection
  connection_ptr conn(new connection(acceptor.get_io_service()));

  //! Start listening on the connection
  acceptor.async_accept(
    conn->socket(),
    boost::bind(
      &TcpServer::handleAccept, this, boost::asio::placeholders::error, conn));
}

void TcpServer::handleAccept(const boost::system::error_code& e, const connection_ptr& conn)
{
  if (!e)
  {
    conns.push_back(conn);
    acceptor.async_accept(conn->socket(),
        boost::bind(&TcpServer::handleWrite, this,
          boost::asio::placeholders::error, conn));

    connection_ptr newConn(new connection(acceptor.get_io_service()));
    acceptor.async_accept(newConn->socket(),
        boost::bind(&TcpServer::handleAccept, this,
          boost::asio::placeholders::error, newConn));
  } else {
    LOG_ERROR("Error while accepting connection to client:\n\t" << e.message());
  }
}

DataServer::DataServer(
  boost::shared_ptr<boost::asio::io_service>& io_service,
  unsigned short port) :
  TcpServer(io_service, port)
{
  outDataQueue = new ThreadSafeQueue<CommMessage>();
}

DataServer::~DataServer()
{
  delete outDataQueue;
}

void DataServer::update() {
  if (!conns.empty()) {
    Json::Value msg;
    while (!outDataQueue->isEmpty()) {
      auto tcpMsg = outDataQueue->queueFront();
      auto key = DataUtils::varToString(static_cast<int>(tcpMsg.type));
      msg[key] = tcpMsg.json;
      outDataQueue->popQueue();
    }
    outMessage.clear();
    Utils::compress(JsonUtils::jsonToMinimalString(msg), outMessage);
  }

  for (auto& conn : conns) {
    //! Write all the data to connection
    conn->async_write(
      outMessage,
      boost::bind(&TcpServer::handleWrite, this, boost::asio::placeholders::error, conn));
  }
}

void DataServer::addMessage(const CommMessage& msg) {
  outDataQueue->pushToQueue(msg);
}

void DataServer::handleWrite(const boost::system::error_code& e, const connection_ptr& conn)
{
  if (!e) {
    conn->async_read(
      inMessage, boost::bind(&TcpServer::handleRead, this, boost::asio::placeholders::error, conn));
  } else {
    if (
      (boost::asio::error::eof == e) ||
      (boost::asio::error::connection_reset == e) ||
      (boost::asio::error::broken_pipe == e))
    {
      //! Disconnected from the client. Remove the associated connection
      LOG_ERROR("Connection to client lost with message:\n\t" << e.message());
      for (auto it = conns.begin(); it != conns.end(); ) {
        if ((*it) == conn) {
          it = conns.erase(it);
        } else {
          ++it;
        }
      }
    }
  }
}

void DataServer::handleRead(const boost::system::error_code& e, connection_ptr& conn)
{
  if (!e) {
    if (!inMessage.empty()) {
      Json::Value root;
      Json::Reader reader;
      std::string decompressed;
      Utils::compress(inMessage, decompressed);
      bool parsingSuccessful = reader.parse(decompressed.c_str(), root);
      if (!parsingSuccessful)
      {
        //! Not a command
        LOG_INFO("Message received: " << decompressed.c_str());
      } else {
        //! If the input string is a json string
        //! Find if a user command is received
        static const auto userCmdKey =
          DataUtils::varToString(static_cast<int>(CommMsgTypes::userCmd));
        if (root[userCmdKey] != Json::nullValue) {
          Json::Value reply = DebugBase::processDebugMsg(root[userCmdKey]);
          //! Generate a reply
          addMessage(CommMessage(reply, CommMsgTypes::logText));
        }
      }
    }
  } else {
    if (
      (boost::asio::error::eof == e) ||
      (boost::asio::error::connection_reset == e))
    {
      //! Disconnected from the client. Remove the associated connection
      LOG_ERROR("Connection to client lost with message:\n\t" << e.message());
      for (auto it = conns.begin(); it != conns.end(); ) {
        if ((*it) == conn) {
          it = conns.erase(it);
        } else {
          ++it;
        }
      }
    }
  }
}

ImageServer::ImageServer(
  boost::shared_ptr<boost::asio::io_service>& io_service,
  unsigned short port) :
  TcpServer(io_service, port)
{
  outImageQueue = new ThreadSafeQueue<cv::Mat >();
}

ImageServer::~ImageServer()
{
  delete outImageQueue;
}

void ImageServer::update() {
  if (!conns.empty()) {
    cout << "Image connections exist..." << endl;
    cout << "outImageQueue->size(): " << outImageQueue->getSize() << endl;
    cv::Mat outImage;
    while (!outImageQueue->isEmpty()) {
      outImage = outImageQueue->queueFront();
      outImageQueue->popQueue();
    }
    outMessage.clear();
    Utils::compress(VisionUtils::cvMatToString(outImage), outMessage);
  }

  for (auto& conn : conns) {
    //! Write all the data to connection
    conn->async_write(
      outMessage,
      boost::bind(&TcpServer::handleWrite, this, boost::asio::placeholders::error, conn));
  }
}

void ImageServer::handleWrite(const boost::system::error_code& e, const connection_ptr& conn)
{
  if (!e) {
    cout << "Wrote image..." << endl;
  } else {
    if (
      (boost::asio::error::eof == e) ||
      (boost::asio::error::connection_reset == e) ||
      (boost::asio::error::broken_pipe == e))
    {
      //! Disconnected from the client. Remove the associated connection
      LOG_ERROR("Connection to client lost with message:\n\t" << e.message());
      for (auto it = conns.begin(); it != conns.end(); ) {
        if ((*it) == conn) {
          it = conns.erase(it);
        } else {
          ++it;
        }
      }
    }
  }
}

void ImageServer::handleRead(const boost::system::error_code& e, connection_ptr& conn)
{
}

void ImageServer::addImage(const cv::Mat& image) {
  outImageQueue->pushToQueue(image);
}
