/**
 * @file UserCommModule/include/CommMessage.h
 *
 * This file defines the class CommMessage
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <string>
#include "Utils/include/DataHolders/CommMsgTypes.h"
#include "Utils/include/JsonUtils.h"

using namespace std;

/**
 * @class CommMessage
 * @brief Defines a message that can be sent through a CommRequest
 */
class CommMessage
{
public:
  CommMessage() = default;

  /**
   * Constructor
   *
   * @param msg: Message string
   * @param type: Message type
   */
  CommMessage(
    const Json::Value& json,
    const CommMsgTypes& type) :
    json(json),
    type(type)
  {
  }

  /**
   * Constructor
   *
   * @param msg: Message string
   * @param type: Message type
   */
  CommMessage(
    const string& msg,
    const CommMsgTypes& type) :
    json(msg),
    type(type)
  {
  }

  static CommMessage jsonToType(const Json::Value& json) {
    CommMessage msg;
    msg.type = static_cast<CommMsgTypes>(json["type"].asUInt());
    msg.json = json["json"];
    return msg;
  }

  static CommMessage getJson(const Json::Value& json) {
    CommMessage msg;
    msg.type = static_cast<CommMsgTypes>(json["type"].asUInt());
    msg.json = json["json"];
    return msg;
  }

  //! Message in json format
  Json::Value json;

  //! Message type
  CommMsgTypes type;
};
