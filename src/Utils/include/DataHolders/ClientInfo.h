/**
 * @file Utils/include/DataHolders/ClientInfo.h
 *
 * This file defines the struct ClientInfo
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <cstring>
#include <net/if.h>
#include <ifaddrs.h>
#include <string>
#include "Utils/include/DataHolders/DataHolder.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/JsonUtils.h"

using namespace std;

/**
 * Enumeration that defines the possible types of clients
 *
 * @enum TcpClientType
 */
enum TcpClientType
{
  BASIC,
  IMAGE
};

/**
 * @struct ClientInfo
 * @brief Holds information about a connected client
 */
struct ClientInfo : public DataHolder
{
  /**
   * @brief ClientInfo Constructor
   * @param address Socket address at which the client is connected
   * @param type Type of client
   */
  ClientInfo(const string& address, const TcpClientType& type) :
    address(address), type(type)
  {
  }

  /**
   * @brief ClientInfo Constructor
   * @param address Socket address at which the client is connected
   * @param type Type of client
   */
  ClientInfo(const sockaddr_in& address, const TcpClientType& type) :
    type(type), address(inet_ntoa(address.sin_addr))
  {
  }

  /**
   * @brief print Self-explanatory
   */
  void print() const final {
    PRINT_DATA(
      ClientInfo,
      (type, static_cast<unsigned>(this->type)),
      (address, address)
    );
  }

  Json::Value getJson() const final
  {
    Json::Value val;
    JSON_ASSIGN_(val, address, address);
    JSON_ASSIGN_(val, type, (int)type);
    return val;
  }

  string address; ///< The socket address
  TcpClientType type; ///< Type of TcpClient
};
