/**
 * @file UserCommModule/UserCommModule.h
 *
 * This file implements the class UserCommModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include <boost/make_shared.hpp>
#include "TNRSBase/include/MemoryIOMacros.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "UserCommModule/include/UserCommModule.h"
#include "UserCommModule/include/UserCommRequest.h"
#include "UserCommModule/include/UserCommTypes.h"
#include "UserCommModule/include/TcpServer.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/DataHolders/CommMsgTypes.h"
#include "Utils/include/EnumUtils.h"
#include "Utils/include/JsonUtils.h"

DEFINE_INPUT_CONNECTOR(UserCommModule,
  (int, userCommThreadPeriod),
)

DEFINE_OUTPUT_CONNECTOR(UserCommModule,
  (int, heartBeat),
)

UserCommModule::UserCommModule(void* teamNUSTSPL) :
  BaseModule(teamNUSTSPL, TNSPLModules::userComm, "UserCommModule")
{
  connPorts.resize(toUType(UserCommTypes::count));
  GET_CONFIG("UserComm",
    (int, dataPort, connPorts[0]),
    (int, imagePort, connPorts[1]),
  )
}

void UserCommModule::setThreadPeriod()
{
  setPeriodMinMS(USER_COMM_PERIOD_IN(UserCommModule));
}

void UserCommModule::initMemoryConn()
{
  inputConnector =
    new InputConnector(this, getModuleName() + "InputConnector");
  outputConnector =
    new OutputConnector(this, getModuleName() + "OutputConnector");
  inputConnector->initConnector();
  outputConnector->initConnector();
}

void UserCommModule::runServers(const boost::shared_ptr<boost::asio::io_service>& ioService)
{
  auto work = boost::make_shared<boost::asio::io_service::work>(*ioService);
  ioService->run();
}

void UserCommModule::init()
{
  LOG_INFO("Initiating UserCommModule...")
  ioThreads.resize(toUType(UserCommTypes::count));
  for (size_t i = 0; i < toUType(UserCommTypes::count); ++i) {
    ioServices.push_back(boost::shared_ptr<boost::asio::io_service> (new boost::asio::io_service));
    if (i == toUType(UserCommTypes::dataConn))
      servers.push_back(
        boost::make_shared<DataServer>(ioServices[i], connPorts[i]));
    else if (i == toUType(UserCommTypes::imageConn))
      servers.push_back(
        boost::make_shared<ImageServer>(ioServices[i], connPorts[i]));
    ioThreads[i] = boost::thread(boost::bind(&UserCommModule::runServers, ioServices[i]));
  }
}

void UserCommModule::handleRequests()
{
  if (inRequests.isEmpty())
    return;
  auto request = inRequests.queueFront();
  if (boost::static_pointer_cast <UserCommRequest>(request)) {
    auto reqId = request->getRequestId();
    if (reqId == toUType(UserCommRequestIds::sendMsgRequest)) {
      auto smr = boost::static_pointer_cast<SendMsgRequest>(request);
      addCommMessageToQueue(smr->cMsg);
    } else if (reqId == toUType(UserCommRequestIds::sendImageRequest)) {
      auto smr = boost::static_pointer_cast<SendImageRequest>(request);
      addImageToQueue(smr->image);
    }
  }
  inRequests.popQueue();
}

void UserCommModule::mainRoutine()
{
  CommMessage cMsg(getLocalSharedMemory()->getJson(), CommMsgTypes::memory);
  addCommMessageToQueue(cMsg);
  for (const auto& server : servers) {
    server->update();
  }
  HEART_BEAT_OUT(UserCommModule)++;
}

void UserCommModule::addCommMessageToQueue(
  const CommMessage& cMsg)
{
  boost::static_pointer_cast<DataServer>(servers[toUType(UserCommTypes::dataConn)])->addMessage(cMsg);
}

void UserCommModule::addImageToQueue(
  const cv::Mat& image)
{
  boost::static_pointer_cast<ImageServer>(servers[toUType(UserCommTypes::imageConn)])->addImage(image);
}
