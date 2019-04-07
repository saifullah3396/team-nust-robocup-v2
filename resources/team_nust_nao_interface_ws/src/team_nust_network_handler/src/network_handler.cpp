/**
 * @file NetworkHandler/NetworkHandler.h
 *
 * This file implements the class NetworkHandler
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include <boost/make_shared.hpp>
#include "network_handler.h"
#include "tcp_client.h"
#include "UserCommModule/include/UserCommTypes.h"
#include "Utils/include/DataHolders/CommMsgTypes.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/JsonUtils.h"
#include "Utils/include/PrintUtils.h"

NetworkHandler::NetworkHandler(
  ros::NodeHandle& nh, const string& robotIp)
{
  connPorts.resize(toUType(UserCommTypes::count));
  GET_CONFIG("UserComm",
    (string, dataPort, connPorts[0]),
    (string, imagePort, connPorts[1]),
  )
  vector<string> publisherNames(toUType(UserCommTypes::count));
  publisherNames[toUType(UserCommTypes::dataConn)] = "team_nust_nao_data";
  publisherNames[toUType(UserCommTypes::imageConn)] = "team_nust_nao_image";
  ioThreads.resize(toUType(UserCommTypes::count));
  for (size_t i = 0; i < toUType(UserCommTypes::count); ++i) {
    ioServices.push_back(
      boost::shared_ptr<boost::asio::io_service> (
        new boost::asio::io_service));
    if (i == toUType(UserCommTypes::dataConn))
      clients.push_back(
        boost::make_shared<DataClient>(
          ioServices[i], robotIp, connPorts[i], nh, publisherNames[i]));
    else if (i == toUType(UserCommTypes::imageConn))
      clients.push_back(
        boost::make_shared<ImageClient>(
          ioServices[i], robotIp, connPorts[i], nh, publisherNames[i]));
    ioThreads[i] = 
      boost::thread(
        boost::bind(&NetworkHandler::runClients, ioServices[i]));
  }
}

void NetworkHandler::runClients(
  const boost::shared_ptr<boost::asio::io_service>& ioService)
{
  auto work = boost::make_shared<boost::asio::io_service::work>(*ioService);
  ioService->run();
}

void NetworkHandler::update()
{
  for (const auto& client : clients) {
    client->update();
  }
}
