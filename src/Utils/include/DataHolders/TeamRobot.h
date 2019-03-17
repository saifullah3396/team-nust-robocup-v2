/**
 * @file Utils/include/DataHolders/TeamRobot.h
 *
 * This file declares and implements the struct TeamRobot
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 15 Jan 2018
 */

#include "Utils/include/PrintUtils.h"
#include "Utils/include/DataHolders/DataHolder.h"
#include "Utils/include/DataHolders/RobotPose2D.h"

#pragma once

/**
 * @struct TeamRobot
 * @brief Defines a single teammate robot
 */
template <typename T = float>
struct TeamRobot : public DataHolder
{
  /**
   * @brief print Self-explanatory
   */
  void print() const final
  {
    PRINT_DATA(
      TeamRobot,
      (id, id),
      (dataReceived, dataReceived),
      (fallen, static_cast<int>(fallen)),
      (intention, static_cast<int>(intention)),
      (suggestionToMe, static_cast<int>(suggestionToMe)),
      (positionConfidence, static_cast<int>(positionConfidence)),
      (sideConfidence, static_cast<int>(sideConfidence)),
      (ballAge, ballAge),
      (ballPos, ballPos),
      (ballVel, ballVel),
      (pose, pose.get().transpose()),
      (walkingTo, walkingTo),
      (shootingTo, shootingTo),
    );
  }

  Json::Value getJson() const final
  {
    Json::Value val;
    JSON_ASSIGN_(val, id, id);
    JSON_ASSIGN_(val, dataReceived, dataReceived);
    JSON_ASSIGN_(val, fallen, static_cast<int>(fallen));
    JSON_ASSIGN_(val, intention, static_cast<int>(intention));
    JSON_ASSIGN_(val, suggestionToMe, static_cast<int>(suggestionToMe));
    JSON_ASSIGN_(val, positionConfidence, static_cast<int>(positionConfidence));
    JSON_ASSIGN_(val, sideConfidence, static_cast<int>(sideConfidence));
    JSON_ASSIGN_(val, ballAge, ballAge);
    JSON_ASSIGN_(val, ballPos, JsonUtils::getJson(ballPos));
    JSON_ASSIGN_(val, ballVel, JsonUtils::getJson(ballVel));
    JSON_ASSIGN_(val, pose, JsonUtils::matrixToJson(pose.get().transpose()));
    JSON_ASSIGN_(val, walkingTo, JsonUtils::getJson(walkingTo));
    JSON_ASSIGN_(val, shootingTo, JsonUtils::getJson(shootingTo));
    return val;
  }

  int id = {-1}; //! Robot id
  bool dataReceived = {false}; //! Whether the data for this robot is recieved
  int8_t fallen = {0}; //! Robot is fallen or not
  int8_t intention = {4}; //! Current robot intention
  int8_t suggestionToMe = {0}; //! Suggestion of this robot to me
  int8_t positionConfidence = {0}; //! Position confidence
  int8_t sideConfidence = {0}; //! Side confidence
  T ballAge = {-1}; //! Time until ball last in seconds
  cv::Point_<T> ballPos; //! Ball position in robot frame
  cv::Point_<T> ballVel; //! Ball velocity in robot frame
  cv::Point_<T> walkingTo; //! Current walking target in world
  cv::Point_<T> shootingTo; //! Current shooting target in world
  RobotPose2D<T> pose; //! Robot pose in world
};
