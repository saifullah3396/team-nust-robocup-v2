/**
 * @file Utils/include/DataHolders/GoalInfo.h
 *
 * This file defines the struct GoalInfo
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "Utils/include/DataHolders/DataHolder.h"
#include "Utils/include/DataHolders/DataHeader.h"
#include "Utils/include/DataHolders/RobotPose2D.h"

using namespace cv;

enum class GoalPostType : unsigned int {
  ourTeam,
  opponentTeam,
  unknown
};

/**
 * @struct GoalInfo
 * @brief Holds information about the goal
 */
template <typename T = float>
struct GoalInfo : public DataHolder, public DataHeader
{
  void print() const final {
    PRINT_DATA(
      GoalInfo,
      (found, found),
      (type, static_cast<unsigned>(type)),
      (leftPost, leftPost),
      (rightPost, rightPost),
      (mid, mid),
      (poseFromGoal, poseFromGoal.get().transpose()),
    );
  }

  Json::Value getJson() const final
  {
    Json::Value val;
    JSON_ASSIGN_(val, found, found);
    JSON_ASSIGN_(val, type, static_cast<unsigned>(type));
    JSON_ASSIGN_(val, leftPost, JsonUtils::getJson(leftPost));
    JSON_ASSIGN_(val, rightPost, JsonUtils::getJson(rightPost));
    JSON_ASSIGN_(val, mid, JsonUtils::getJson(mid));
    JSON_ASSIGN_(val, poseFromGoal, JsonUtils::matrixToJson(poseFromGoal.get().transpose()));
    return val;
  }

  bool found = {false}; ///< Whether the goal post is found
  GoalPostType type = {GoalPostType::unknown}; ///< Whether the goal post is type or opponents
  cv::Point_<T> leftPost; ///< Left post position
  cv::Point_<T> rightPost; ///< Right post position
  cv::Point_<T> mid; ///< Goal middle position
  RobotPose2D<T> poseFromGoal; ///< Pose of robot from the goal
};
