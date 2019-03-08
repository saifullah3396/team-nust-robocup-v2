/**
 * @file Utils/include/DataHolders/Obstacle.h
 *
 * This file defines the structs Obstacle and ObsObstacles
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 6 Jan 2018
 */

#pragma once

#include "Utils/include/DataHolders/DataHolder.h"
#include "Utils/include/DataHolders/DataHeader.h"
#include "Utils/include/DataHolders/ObstacleType.h"
#include "Utils/include/TNRSLine.h"
#include "Utils/include/JsonUtils.h"

/**
 * @struct Obstacle
 * @brief Holds information about a single obstacle
 */
template <typename T = float>
struct Obstacle : public DataHolder
{
  /**
   * @brief Obstacle Constructor
   * @param type Obstacle type
   */
  Obstacle(const ObstacleType& type) : type(type)
  {
    if (type == ObstacleType::goalPost) {
      depth = goalPoseObsDepth;
    } else if (type == ObstacleType::opponent || type == ObstacleType::teammate) {
      depth = robotObsDepth;
    } else if (type == ObstacleType::opponentFallen || type == ObstacleType::teammateFallen) {
      depth = fallenRobotObsDepth; //Depth of the robot.
    }
  }

  /**
   * @brief print Self-explanatory
   */
  void print() const final {
    PRINT_DATA(
      Obstacle,
      (type, static_cast<unsigned>(this->type)),
      (center, center),
      (depth, depth),
    );
  }

  Json::Value getJson() const final
  {
    Json::Value val;
    JSON_ASSIGN_(val, type, static_cast<unsigned>(type));
    JSON_ASSIGN_(val, center, JsonUtils::getJson(center));
    JSON_ASSIGN_(val, depth, depth);
    return val;
  }

  TNRSLine<T> front; //! Obstacle front line in image
  TNRSLine<T> back; //! Obstacle back line in image
  TNRSLine<T> frontT; //! Obstacle front line in world
  TNRSLine<T> backT; //! Obstacle front line in world
  ObstacleType type = {ObstacleType::unknown}; //! Obstacle type
  cv::Point_<T> center; //! Obstacle center
  T depth = {0}; //! Obstacle depth

  static constexpr float goalPoseObsDepth = 0.1f; //! Goal post depth
  static constexpr float robotObsDepth = 0.2f; //! Standing robot depth
  static constexpr float fallenRobotObsDepth = 0.2f; //! Fallen robot depth
};

/**
 * @struct ObsObstacles
 * @brief A struct that holds the information about all the latest 
 *   observed obstacles.
 */
template <typename T = float>
struct ObsObstacles : public DataHolder, public DataHeader
{
  vector<Obstacle<T> > data; //! The array of observed obstacles
};
