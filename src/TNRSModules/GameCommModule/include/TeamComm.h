/**
 * @file TeamComm/TeamComm.h
 *
 * This file declares the class TeamComm
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Jan 2017
 */

#pragma once

#include "GameCommModule/include/GameCommModule.h"
#include "GameCommModule/include/UdpComm.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/DataHolders/SPLStandardMessage.h"
#include "Utils/include/ThreadSafeQueue.h"
#include <boost/make_shared.hpp>

/**
 * @class TeamComm
 * @brief This class deals with inter-robot communication using udp
 *   broadcast and messsage receival
 */
class TeamComm : public MemoryBase
{
public:
  /**
   * @brief TeamComm Constructor
   *
   * @param commModule pointer to parent CommModule
   * @param port port used for udp communication
   * @param subnet broadcast ip address of the system
   */
  TeamComm(
    GameCommModule* gameCommModule,
    const unsigned short& port,
    const char* subnet);

  /**
   * @brief ~TeamComm Destructor
   */
  ~TeamComm() {}

  /**
   * @brief send Broadcasts the SPLStandardMessage obtained
   *   from the sendQueue front
   */
  void send();

  /**
   * @brief receive Receives the SPLStandardMessage and adds
   *   it to the receiveQueue
   *
   * @return unsigned Size of recieved message
   */
  unsigned receive();

  /**
   * @brief processReceivedMsgs Processes messages stored
   *   in receiveQueue
   */
  void processReceivedMsgs();

private:
  /**
   * @brief updateMessage Updates the SPLStandardMsg to be broadcasted based on
   *   the current robot state
   *
   * @param msg message to be updated
   */
  void updateMessage(SPLStandardMessage& msg);

  /**
   * @brief updateTeamData Processes an incoming StandarSPLMessage message
   *   and updates the team robots data according to the message
   */
  void updateTeamData(const SPLStandardMessage& msg);

  //! Port to be used for udp communication. Team ports are setup as
  //! 10000 + teamNumber whereas teamNumber is defined in
  //! Config/TeamSettings.
  unsigned short port;

  //! UdpComm class object for udp connection management.
  UdpComm udpComm;

  //! GameCommModule pointer
  GameCommModule* gameCommModule;

  //! The receive queus for SPLStandardMessage.
  ThreadSafeQueue<SPLStandardMessage> receiveQueue;
};

typedef boost::shared_ptr<TeamComm> TeamCommPtr;
