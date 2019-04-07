/**
 * @file Utils/include/DataHolders/CommMessage.h
 *
 * This file defines the class CommMessage
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <string>
#include "Utils/include/DataHolders/CommMsgTypes.h"
#include "Utils/include/DataHolders/DataHolder.h"
#ifndef VISUALIZER_BUILD
#include "Utils/include/JsonUtils.h"
#endif

/**
 * @class CommMessage
 * @brief Defines a message that can be sent through a CommRequest
 */
struct CommMessage : public DataHolder
{
  CommMessage() = default;
  CommMessage(const CommMessage&) = default;
  virtual ~CommMessage() {}

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

  #ifndef VISUALIZER_BUILD
  /**
   * Self-explanatory
   */
  void print() const final
  {
    PRINT_DATA(
      CommMessage,
      (CommMsgTypes, toUType(type)),
      (Json::Value, json),
    );
  }

  Json::Value getJson() const final
  {
    Json::Value val;
    JSON_ASSIGN_(val, type, toUType(type));
    JSON_ASSIGN_(val, json, json);
    return val;
  }

  static CommMessage jsonToType(const Json::Value& json) {
    CommMessage msg;
    msg.type = static_cast<CommMsgTypes>(json["type"].asUInt());
    msg.json = json["json"];
    return msg;
  }
  #endif

  ///< Message in json format
  Json::Value json;

  ///< Message type
  CommMsgTypes type;
};
