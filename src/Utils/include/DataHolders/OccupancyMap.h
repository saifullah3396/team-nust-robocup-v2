/**
 * @file Utils/include/DataHolders/OccupancyMap.h
 *
 * This file defines the struct OccupancyMap
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "Utils/include/DataHolders/DataHolder.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/JsonUtils.h"

/**
 * @struct OccupancyMap
 * @brief Holds the environment's occupancy map information.
 */
template <typename T = float>
struct OccupancyMap : public DataHolder
{
  void print() const  final
  {
    PRINT_DATA(
      OccupancyMap,
      (resolution, resolution),
      (originPose, originPose),
    );
  }

  Json::Value getJson() const final
  {
    Json::Value val;
    JSON_ASSIGN_(val, resolution, resolution);
    JSON_ASSIGN_(val, originPose, JsonUtils::getJson(originPose));
    return val;
  }

  T resolution = {0}; //! Map resolution
  cv::Point3_<T> originPose; //! Map origin x-y-theta
  cv::Mat  data; //! Map data stored in the form of a matrix
};
