/**
 * @file VisionModule/include/FeatureExtraction/RobotRegion.h
 *
 * This file defines the struct RobotRegion.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include "Utils/include/VisionUtils.h"
#include "Utils/include/DataHolders/ObstacleType.h"
#include "VisionModule/include/FeatureExtraction/ScannedRegion.h"

/**
 * @struct RobotRegion
 * @brief Holds information about the region that defines a robot.
 */
struct RobotRegion
{
  RobotRegion(const RobotRegion&) = default;
  RobotRegion(RobotRegion&&) = default;
  RobotRegion& operator=(const RobotRegion&) & = default;
  RobotRegion& operator=(RobotRegion&&) & = default;
  virtual ~RobotRegion() {}

  /**
   * @brief RobotRegion Constructor
   * @param sr Associated scanned region
   * @param ourTeam Whether its a teammate or opponent
   */
  RobotRegion(
    const boost::shared_ptr<ScannedRegion>& sr,
    const ObstacleType& obstacleType) :
    obstacleType(obstacleType), sr(sr)
  {
  }

  cv::Point2f world; ///< Position in world
  cv::Point2f frontLeft; ///< Position of left front in world
  cv::Point2f frontRight; ///< Position of right front in world
  ObstacleType obstacleType; ///< Type of obstacle
  boost::shared_ptr<ScannedRegion> sr;
  boost::shared_ptr<ScannedRegion> bodySr;
};
typedef boost::shared_ptr<RobotRegion> RobotRegionPtr;
