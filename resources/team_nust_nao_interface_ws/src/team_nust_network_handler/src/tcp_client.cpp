/**
 * @file team_nust_network_handler/src/boost_tcp_client.cpp
 *
 * This file implements the class TcpClient and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "tcp_client.h"
#include "TNRSBase/include/DebugBase.h"
#include "UserCommModule/include/BoostConnection.h"
#include "Utils/include/DataHolders/CommMessage.h"
#include "Utils/include/DataHolders/CommMsgTypes.h"
#include "Utils/include/JsonUtils.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/ZLIBCompression.h"

TcpClient::TcpClient(
  boost::shared_ptr<boost::asio::io_service>& ioService,
  const std::string& host,
  const std::string& port)
  : conn(*ioService), ioService(ioService), host(host), port(port)
{
  ROS_INFO("Starting connection with server at ip %s and port %s", host.c_str(), port.c_str());
  boost::asio::ip::tcp::resolver resolver(*ioService) ;
  boost::asio::ip::tcp::resolver::query query(host, port);
  boost::asio::ip::tcp::resolver::iterator endpoint_iterator =
    resolver.resolve(query);
  boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;

  conn.socket().async_connect(endpoint,
    boost::bind(&TcpClient::handleConnect, this,
      boost::asio::placeholders::error, ++endpoint_iterator));
}

void TcpClient::handleConnect(const boost::system::error_code& e,
  boost::asio::ip::tcp::resolver::iterator endpoint_iterator)
{
  if (!e)
  {
    onConnection();
  } else if (boost::asio::error::already_connected != e) {
    if (!reconnecting)
      ROS_WARN("%s", e.message().c_str());
    reconnect();
  }
}

void TcpClient::reconnect()
{
  if (!reconnecting)
    ROS_INFO("Trying to reconnect with server at ip %s and port %s", host.c_str(), port.c_str());
  reconnecting = true;
  boost::asio::ip::tcp::resolver resolver(*ioService);
  boost::asio::ip::tcp::resolver::query query(host, port);
  boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
  boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;
  conn.socket().async_connect(endpoint,
    boost::bind(&TcpClient::handleConnect, this,
      boost::asio::placeholders::error, ++endpoint_iterator));
}

DataClient::DataClient(
  boost::shared_ptr<boost::asio::io_service>& ioService,
  const std::string& host,
  const std::string& port,
  ros::NodeHandle& nh,
  const std::string& publisher_name) :
  TcpClient(ioService, host, port)
{
  dataSubscriber =
    nh.subscribe(
      "team_nust_user_cmds", 1, &DataClient::commandToMessage, this);
  dataPublisher =
    nh.advertise<std_msgs::String>(publisher_name, 1);
}

void DataClient::handleWrite(
  const boost::system::error_code& e, const bool& readAfter)
{
  if (e) {
    if (
      (boost::asio::error::eof == e) ||
      (boost::asio::error::connection_reset == e) ||
      (boost::asio::error::broken_pipe == e))
    {
      ///< Disconnected from the client. Remove the associated connection
      if (!reconnecting)
        ROS_FATAL("Connection lost to server with message: %s", e.message().c_str());
      conn.socket().close();
      reconnect();
    } else {
      ROS_WARN("Error in asyn_write(): %s", e.message().c_str());
    }
  }
  outMessage.clear();
  if (readAfter) {
    conn.async_read(
      inMessage,
      boost::bind(
        &TcpClient::handleRead, this, boost::asio::placeholders::error));
  }
}

void DataClient::handleRead(const boost::system::error_code& e)
{
  if (!e) {
    Json::Value root;
    Json::Reader reader;
    std::string decompressed;
    if (Utils::decompress(inMessage, decompressed)) {
      bool parsed = reader.parse(decompressed, root);
      if (!parsed)
      {
        ROS_WARN("Failed to parse %s", reader.getFormattedErrorMessages().c_str());
      } else {
        ///< If the input string is a json string
        ///< Find if a user command is received
        std_msgs::String msg;
        msg.data = decompressed;
        dataPublisher.publish(msg);
      }
    }
  } else {
    if (
      (boost::asio::error::eof == e) ||
      (boost::asio::error::connection_reset == e) ||
      (boost::asio::error::broken_pipe == e))
    {
      ///< Disconnected from the client. Remove the associated connection
      ROS_FATAL("Connection lost to server with message: %s", e.message().c_str());
      conn.socket().close();
      reconnect();
    } else {
      ROS_WARN("Error in asyn_read(): %s", e.message().c_str());
    }
  }
  conn.async_write(
    outMessage,
    boost::bind(
      &TcpClient::handleWrite, this, boost::asio::placeholders::error, true));
}

void DataClient::commandToMessage(const std_msgs::String::ConstPtr& msg)
{
  auto messageStr = msg->data;
  messageStr.erase(std::remove(messageStr.begin(), messageStr.end(), '\n'), messageStr.end());
  messageStr =
    "{\"" +
    DataUtils::varToString(static_cast<int>(CommMsgTypes::userCmd)) +
    "\":" +
    messageStr +
    "}";
  if (!Utils::compress(messageStr, outMessage))
    outMessage.clear();
}

/**
 * @brief update Updates the server read/write cycle
 */
void DataClient::update() {
  conn.async_read(
    inMessage,
    boost::bind(
      &TcpClient::handleRead, this, boost::asio::placeholders::error));
}

void DataClient::onConnection()
{
  ROS_INFO("Data client connected to server.");
  reconnecting = false;
  if (Utils::compress("DataClient connected.", outMessage)) {
    conn.async_write(
      outMessage,
      boost::bind(
        &TcpClient::handleWrite, this, boost::asio::placeholders::error, false));
  }
}

ImageClient::ImageClient(
  boost::shared_ptr<boost::asio::io_service>& ioService,
  const std::string& host,
  const std::string& port,
  ros::NodeHandle& nh,
  const std::string& publisher_name) :
  TcpClient(ioService, host, port)
{
  dataPublisher = nh.advertise<std_msgs::String>(publisher_name, 1);
}

void ImageClient::handleWrite(
  const boost::system::error_code& e, const bool& readAfter)
{
}

void ImageClient::handleRead(const boost::system::error_code& e)
{
  if (!e) {
    std::string decompressed;
    if (Utils::decompress(inMessage, decompressed)) {
      ///< If the input string is a json string
      ///< Find if a user command is received
      std_msgs::String msg;
      msg.data = decompressed;
      dataPublisher.publish(msg);
    }
  } else {
    if (
      (boost::asio::error::eof == e) ||
      (boost::asio::error::connection_reset == e) ||
      (boost::asio::error::broken_pipe == e))
    {
      ///< Disconnected from the client. Remove the associated connection
      ROS_FATAL("Connection lost to server with message: %s", e.message().c_str());
      conn.socket().close();
      reconnect();
    } else {
      ROS_WARN("Error in asyn_read(): %s", e.message().c_str());
    }
  }
  conn.async_read(
    inMessage,
    boost::bind(
      &TcpClient::handleRead, this, boost::asio::placeholders::error));
}

/**
 * @brief update Updates the server read/write cycle
 */
void ImageClient::update() {
  conn.async_read(
    inMessage,
    boost::bind(
      &TcpClient::handleRead, this, boost::asio::placeholders::error)
  );
}
