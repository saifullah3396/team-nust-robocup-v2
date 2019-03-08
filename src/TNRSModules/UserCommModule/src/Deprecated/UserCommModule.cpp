/**
 * @file UserCommModule/UserCommModule.h
 *
 * This file implements the class UserCommModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "Utils/include/ConfigMacros.h"
#include "UserCommModule/include/UserCommModule.h"
#include "UserCommModule/include/CommRequest.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"

UserCommModule::UserCommModule(void* teamNUSTSPL) :
  BaseModule(teamNUSTSPL, (unsigned) TNSPLModules::USER_COMM, "UserCommModule"),
  DebugBase("UserCommModule", this)
{
  auto size = static_cast<int>(Connections::COUNT);
  connPorts.resize(size);
  servers.resize(size);
  ioServices.resize(size);
  ioThreads.resize(size);
  GET_CONFIG("UserCommModule",
    (bool, memoryDataSwitch, memoryDataSwitch),
    (unsigned short, dataPort, connPorts[Connections::DATA_CONN]),
    (unsigned short, imagePort, connPorts[Connections::IMAGE_CONN]),
  )
}

void UserCommModule::setThreadPeriod()
{
  setPeriodMinMS(IVAR(int, UserCommModule::userCommThreadPeriod));
}

void UserCommModule::initMemoryConn()
{
  ASSERT_MSG(sharedMemory, "Shared Memory not found.");
  inputConnector =
    new InputConnector(this, getModuleName() + "InputConnector");
  outputConnector =
    new OutputConnector(this, getModuleName() + "OutputConnector");
  inputConnector->initConnector();
  outputConnector->initConnector();
}

void UserCommModule::init()
{
  for (size_t i = 0; i < static_cast<size_t>(Connections::COUNT); ++i) {
    servers =
      boost::make_shared<BoostServer>(ioServices[i], connPorts[i]);
    threads[i] = thread([&]{ioServices[i].run();});
  }
}

void UserCommModule::handleRequests()
{
  if (inRequests.isEmpty())
    return;
  auto request = inRequests.queueFront();
  if (boost::static_pointer_cast <CommRequest>(request)) {
    auto reqId = request->getId();
    if (reqId == (unsigned)CommRequestIds::SEND_MSG_REQUEST) {
      auto smr = boost::static_pointer_cast<SendMsgRequest>(request);
      if (smr->cMsg.getType() > CommMsgTypes::REQUEST_MSGS) {
        if (smr->cMsg.getType() == CommMsgTypes::TOP_IMAGE ||
            smr->cMsg.getType() == CommMsgTypes::BOTTOM_IMAGE)
          addCommMessageToQueue(smr->cMsg, IMAGE, -1);
        else
          addCommMessageToQueue(smr->cMsg, BASIC, -1);
      }
    }
  }
  inRequests.popQueue();
}

void UserCommModule::mainRoutine()
{
  cout << "UserCommModule::mainRoutine() called..." << endl;
  /*
  if (memoryDataSwitch) {
    sendHeartBeat();
    sendMemoryData();
  }
  sendReceive();
  updateIncomingMessageQueue();
  OVAR(int, UserCommModule::heartBeat)++;*/
}

/*
void
UserCommModule::sendMemoryHeader()
{
  //! Send memory header
  string memoryHeader;
  getLocalSharedMemory()->getStringHeader(memoryHeader);
  CommMessage cMsg(memoryHeader, CommMsgTypes::MEMORY_HEADER);
  addCommMessageToQueue(cMsg, BASIC, -1);  
}

void
UserCommModule::sendHeartBeat()
{
  //! Send heart beat signal
  CommMessage cMsg("", CommMsgTypes::HEART_BEAT);
  addCommMessageToQueue(cMsg, BASIC, -1);  
}

void
UserCommModule::sendMemoryData()
{
  //! Send memory data
  string memoryData;
  getLocalSharedMemory()->getString(memoryData);
  CommMessage cMsg(memoryData, CommMsgTypes::MEMORY_DATA);
  addCommMessageToQueue(cMsg, BASIC, -1);
}

void 
UserCommModule::addCommMessageToQueue(
  const CommMessage& cMsg,
  const TcpClientType& targetClientType, 
  const int& relatedClientId)
{
  auto msg = TcpMessage(
    cMsg.getMessage(), 
    cMsg.getType(),
    targetClientType,
    relatedClientId
  );
  sendDataQueue.push(msg);
}

void
UserCommModule::sendReceive()
{
  //!Data sending
  while (!sendDataQueue.empty()) {
    auto tcpMsg = sendDataQueue.front();
    string dataToSend = 
      DataUtils::varToString((unsigned)tcpMsg.getType()) + "@" + tcpMsg.getMessage() + "\n";
    for (size_t i = 0; i < clients.size(); ++i)
    {
      if (clients[i]->connected() &&
          tcpMsg.targetClientType == clients[i]->getType() &&
          tcpMsg.relatedClientId == -1 || 
          tcpMsg.relatedClientId == clients[i]->getId()) 
      {
        if (!clients[i]->send(dataToSend.c_str(), dataToSend.length())) {
          LOG_ERROR("Failed to send data to client " << i)
        }
      }
    }
    sendDataQueue.pop();
  }
  //!Data recieval
  for (size_t i = 0; i < clients.size(); ++i) {
    if (clients[i]->connected()) {
      int bytesRead = 0;
      do {
        bytesRead = receive(i);
        if (bytesRead < 0) continue;
      } while (bytesRead > 0);
    } else {
      clients[i]->setDeleteClient(true);
    }
  }
}

int
UserCommModule::receive(const int& clientIndex)
{
  if (!dataRead) {
    //ParentThread->DisplayToLog(1, "ASSERT:"  "receive:Error creating recv buffer");
    return 0;
  }
  int sizeReceived = 0;
  bool noMoreData = false;
  bool success = clients[clientIndex]->receive(
    recvDataQueue,
    dataRead,
    sizeReceived,
    noMoreData,
    TEMP_DATA_BUFFER_SIZE);
  if (!noMoreData) {
    if (!success) {
      return -1;
    } else {
      return sizeReceived;
    }
  } else {
    return 0;
  }
}

void
UserCommModule::processIncomingMessage(const TcpMessage& tcpMsg)
{
  string reply = DebugBase::processDebugMsg(tcpMsg.getMessage());
  auto msg = CommMessage(reply, CommMsgTypes::LOG_TEXT);
  addCommMessageToQueue(msg, BASIC, tcpMsg.relatedClientId);
}

void
UserCommModule::updateIncomingMessageQueue()
{
  while (!recvDataQueue.empty()) {
    processIncomingMessage(recvDataQueue.front());
    recvDataQueue.pop();
  }
}
*/
