/**
 * @file Utils/include/DataHolders/Landmark.h
 *
 * This file defines the structs Landmark and ObsLandmarks
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017 
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "Utils/include/DataHolders/RobotPose2D.h"

using namespace cv;

/**
 * @struct Landmark
 * @brief The struct for defining a field landmark
 */
template <typename T = float>
struct Landmark : public DataHolder
{
  /**
   * @brief Landmark Constructor
   * @param type Obstacle type
   * @param pos Obstacle position
   */
  Landmark(const unsigned& type, const cv::Point_<T>& pos) :
    type(type), pos(pos)
  {
  }

  /**
   * @brief print Self-explanatory
   */
  void print() const override {
    PRINT_DATA(
      Landmark,
      (id, id),
      (type, static_cast<unsigned>(type)),
      (pos, pos),
    );
  }

  Json::Value getJson() const override
  {
    Json::Value val;
    JSON_ASSIGN_(val, id, id);
    JSON_ASSIGN_(val, type, static_cast<unsigned>(type));
    JSON_ASSIGN_(val, pos, JsonUtils::getJson(pos));
    return val;
  }

  int id = {-1}; //! Id of this landmark
  unsigned type = {0}; //! Type of this landmark
  cv::Point_<T> pos;//! Position of the landmark in field
};

template struct Landmark<float>;
template struct Landmark<double>;

/**
 * @struct UnknownLandmark
 * @brief The struct for defining the landmarks for which a match in 
 *   the actual field is not known
 */
template <typename T = float>
struct UnknownLandmark : Landmark<T>
{
  /**
   * @brief UnknownLandmark Constructor
   * @param pos Obstacle position
   */
  UnknownLandmark(const cv::Point_<T>& pos) :
    Landmark<T>(0, pos) // Change 0 to LandmarkTypes::UNKNOWN
  {
  }

  /**
   * @brief print Self-explanatory
   */
  void print() const final {
    PRINT_DATA(
      UnknownLandmark,
      (id, this->id),
      (type, static_cast<unsigned>(this->type)),
      (pos, this->pos),
    );
  }
};

template struct UnknownLandmark<float>;
template struct UnknownLandmark<double>;

/**
 * @struct UnknownLandmark
 * @brief The struct for defining the landmarks for which a match in 
 *   the actual field is known such as T, L corners, or center circle
 */
template <typename T = float>
struct KnownLandmark : Landmark<T>
{
  /**
   * @brief KnownLandmark Constructor
   * @param type Obstacle type
   * @param pos Obstacle position
   * @param poseFromLandmark Robot pose from landmark
   */
  KnownLandmark(
    const unsigned& type,
    const cv::Point_<T>& pos,
    const RobotPose2D<T>& poseFromLandmark) :
    Landmark<T>(type, pos),
    poseFromLandmark(poseFromLandmark)
  {
  }

  /**
   * @brief print Self-explanatory
   */
  void print() const final {
    PRINT_DATA(
      KnownLandmark,
      (id, this->id),
      (type, static_cast<unsigned>(this->type)),
      (pos, this->pos),
      (poseFromLandmark, poseFromLandmark.get().transpose()),
    );
  }

  Json::Value getJson() const final
  {
    Json::Value val;
    JSON_ASSIGN_(val, id, this->id);
    JSON_ASSIGN_(val, type, static_cast<unsigned>(this->type));
    JSON_ASSIGN_(val, pos, JsonUtils::getJson(this->pos));
    JSON_ASSIGN_(val, poseFromLandmark, JsonUtils::MatrixToJson(this->poseFromLandmark.get().transpose()));
    return val;
  }

  RobotPose2D<T> poseFromLandmark; //! Pose of the landmark from robot
};
