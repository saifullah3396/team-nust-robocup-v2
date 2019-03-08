/**
 * @file Utils/include/DataHolders/WorldBallInfo.h
 *
 * This file defines the struct WorldBallInfo
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "Utils/include/DataHolders/DataHolder.h"
#include "Utils/include/PrintUtils.h"

/**
 * @struct WorldBallInfo
 * @brief Holds information about the latest state of the ball in world extracted
 *   from teammates' data.
 */
template <typename T = float>
struct WorldBallInfo : public DataHolder
{
  /**
   * @brief print Self-explanatory
   */
  void print() const final
  {
    PRINT_DATA(
      WorldBallInfo,
      (found, found),
      (posWorld, posWorld),
      (velWorld, velWorld),
    );
  }

  Json::Value getJson() const final
  {
    Json::Value val;
    JSON_ASSIGN_(val, found, found);
    JSON_ASSIGN_(val, posWorld, JsonUtils::getJson(posWorld));
    JSON_ASSIGN_(val, velWorld, JsonUtils::getJson(velWorld));
    return val;
  }

  bool found = bool{false}; //! Whether the ball is found
  cv::Point_<T> posWorld; //! Ball position in world frame
  cv::Point_<T> velWorld; //! Ball velocity in world frame
};
