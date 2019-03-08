/**
 * @file FeatureExtraction/ScannedLine.h
 *
 * This file defines the struct ScannedLine.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include "Utils/include/VisionUtils.h"

/**
 * @struct ScannedLine
 * @brief Holds information about the line segments resulting from
 *   scanning the image.
 */
struct ScannedLine
{
  ScannedLine(const ScannedLine&) = default;
  ScannedLine(ScannedLine&&) = default;
  ScannedLine& operator=(const ScannedLine&) & = default;
  ScannedLine& operator=(ScannedLine&&) & = default;
  virtual ~ScannedLine() {}

  /**
   * Constructor
   * 
   * @param p1: One of the two end points of the line segment
   * @param p2: One of the two end points of the line segment
   */
  ScannedLine(
    const cv::Point& p1, const cv::Point& p2) :
    p1(p1), p2(p2)
  {
  }

  cv::Point p1; //! One of the two end points of the line segment
  cv::Point p2; //! One of the two end points of the line segment
  float closestDist = {1e6}; //! Distance to other closest scanned line
  int chainLength = {0}; //! Length of the chain of which this line is the parent
  bool searched = {false}; //! Whether this line segment has already been searched
  boost::shared_ptr<ScannedLine> pred; //! Preceding ScannedLine in search
  boost::shared_ptr<ScannedLine> bestNeighbor; //! Best neighbouring ScannedLine
};
typedef boost::shared_ptr<ScannedLine> ScannedLinePtr;
