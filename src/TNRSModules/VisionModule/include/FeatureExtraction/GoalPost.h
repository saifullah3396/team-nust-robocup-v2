/**
 * @file VisionModule/include/FeatureExtraction/GoalPost.h
 *
 * This file defines the struct GoalPost.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include "Utils/include/VisionUtils.h"

/**
 * @struct GoalPost
 * @brief Holds information about a detected goalpost
 */
struct GoalPost
{
  GoalPost(const GoalPost&) = default;
  GoalPost(GoalPost&&) = default;
  GoalPost& operator=(const GoalPost&) & = default;
  GoalPost& operator=(GoalPost&&) & = default;
  virtual ~GoalPost() {}

  /**
   * @brief GoalPost Constructor
   * @param world Position in world
   * @param world Position in image
   */
  GoalPost(const cv::Point2f& world, const cv::Point2f& image, const float& timeDetected = 0.f) :
    world(world), image(image), timeDetected(timeDetected), refresh(true)
  {
  }

  /**
   * @brief checkDuplicate Checks if the detected goal post is similar to
   *   another one
   * @param other Other post
   * @return true if a duplicate is found
   */
  bool checkDuplicate(const boost::shared_ptr<GoalPost>& other)
  {
    if (cv::norm(other->world - this->world) < minDistValidation) {
      *this = *other;
      refresh = true;
      return true;
    } else {
      refresh = false;
      return false;
    }
  }

  cv::Point2f world; ///< Position in world
  cv::Point2f image; ///< Position in image
  bool refresh = {true}; ///< To refresh the data
  float timeDetected = {0.f}; ///< Time at first detection

  ///< Distance between two posts to consider them duplicates
  static constexpr float minDistValidation = 0.3f;
};
typedef boost::shared_ptr<GoalPost> GoalPostPtr;
