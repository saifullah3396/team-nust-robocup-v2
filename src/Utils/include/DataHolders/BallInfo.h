/**
 * @file Utils/include/DataHolders/BallInfo.h
 *
 * This file defines the struct BallInfo
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "Utils/include/DataHolders/DataHolder.h"
#include "Utils/include/DataHolders/DataHeader.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/JsonUtils.h"

/**
 * @struct BallInfo
 * @brief Holds information about the latest state of the ball
 */
template <typename T = float>
struct BallInfo : public DataHolder, DataHeader
{
  /**
   * Self-explanatory
   */
  void print() const final
  {
    PRINT_DATA(
      BallInfo,
      (camera, static_cast<unsigned>(camera)),
      (cameraNext, static_cast<unsigned>(cameraNext)),
      (bboxWidth, bboxWidth),
      (bboxHeight, bboxHeight),
      (found, found),
      (ballAge, ballAge),
      (radius, radius),
      (posImage, posImage),
      (posRel, posRel),
      (velRel, velRel),
      (accRel, accRel)
    );
  }

  Json::Value getJson() const final
  {
    Json::Value val;
    JSON_ASSIGN_(val, camera, static_cast<unsigned>(camera));
    JSON_ASSIGN_(val, cameraNext, static_cast<unsigned>(cameraNext));
    JSON_ASSIGN_(val, bboxWidth, bboxWidth);
    JSON_ASSIGN_(val, bboxHeight, bboxHeight);
    JSON_ASSIGN_(val, found, found);
    JSON_ASSIGN_(val, ballAge, ballAge);
    JSON_ASSIGN_(val, radius, radius);
    JSON_ASSIGN_(val, posImage, JsonUtils::getJson(posImage));
    JSON_ASSIGN_(val, posRel, JsonUtils::getJson(posRel));
    JSON_ASSIGN_(val, velRel, JsonUtils::getJson(velRel));
    JSON_ASSIGN_(val, accRel, JsonUtils::getJson(accRel));
    return val;
  }

  CameraId camera = {CameraId::headTop}; ///< Camera in which ball is observed
  CameraId cameraNext = {CameraId::headBottom};///< Camera in which ball would be observed in next frame
  unsigned bboxWidth = {0}; ///< Ball bounding box width in image
  unsigned bboxHeight = {0}; ///< Ball bounding box height in image
  bool found = {false}; ///< Ball is found or not
  T ballAge = {-1.0}; ///< Time during which ball is observed
  T radius = {0.0}; ///< Ball radius
  cv::Point_<T> posImage; ///< Position of the ball in image
  cv::Point_<T> posRel; ///< Position of ball in robot frame
  cv::Point_<T> velRel; ///< Velocity of ball in robot frame
  cv::Point_<T> accRel; ///< Acceleration of ball in robot frame
};
