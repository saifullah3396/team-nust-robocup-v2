/**
 * @file VisionModule/include/FeatureExtraction/ScannedLine.h
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

  cv::Point p1; ///< One of the two end points of the line segment
  cv::Point p2; ///< One of the two end points of the line segment
  float closestDist = {1e6}; ///< Distance to other closest scanned line
  int chainLength = {0}; ///< Length of the chain of which this line is the parent
  bool searched = {false}; ///< Whether this line segment has already been searched
  boost::shared_ptr<ScannedLine> pred; ///< Preceding ScannedLine in search
  boost::shared_ptr<ScannedLine> bestNeighbor; ///< Best neighbouring ScannedLine
};
typedef boost::shared_ptr<ScannedLine> ScannedLinePtr;

/**
 * @struct LinearScannedLine
 * @brief Holds information about a line segments in one direction x-y
 */
struct LinearScannedLine
{
  LinearScannedLine(const LinearScannedLine&) = default;
  LinearScannedLine(LinearScannedLine&&) = default;
  LinearScannedLine& operator=(const LinearScannedLine&) & = default;
  LinearScannedLine& operator=(LinearScannedLine&&) & = default;
  virtual ~LinearScannedLine() {}

  /**
   * Constructor
   *
   * @param start: Start of scanned line
   * @param end: End of scanned line
   */
  LinearScannedLine(
    const int& start, const int& end, const int& baseIndex, const int& len, const bool& direction) :
    start(start), end(end), baseIndex(baseIndex), len(len), direction(direction)
  {
  }

  cv::Point getStartPoint() {
    if (!direction)
      return cv::Point(start, baseIndex);
    else
      return cv::Point(baseIndex, start);
  }

  cv::Point getEndPoint() {
    if (!direction)
      return cv::Point(end, baseIndex);
    else
      return cv::Point(baseIndex, end);
  }

  void draw(cv::Mat& image, const cv::Scalar& color = cv::Scalar(0, 0, 0)) {
    if (!direction) {
      cv::line(
        image,
        cv::Point(start, baseIndex),
        cv::Point(end, baseIndex),
        color, 1
      );
    } else {
      cv::line(
        image,
        cv::Point(baseIndex, start),
        cv::Point(baseIndex, end),
        color, 1
      );
    }
  }

  bool direction = {false}; ///< False for horizontal, true for vertical
  int len; ///< Length of line
  int start; ///< Start of the line segment
  int end; ///< End of the line segment
  int baseIndex; ///< Other index
  float closestDist = {1e6}; ///< Distance to other closest scanned line
  int chainLength = {0}; ///< Length of the chain of which this line is the parent
  bool searched = {false}; ///< Whether this line segment has already been searched
  boost::shared_ptr<LinearScannedLine> pred; ///< Preceding ScannedLine in search
  boost::shared_ptr<LinearScannedLine> bestNeighbor; ///< Best neighbouring ScannedLine
};
typedef boost::shared_ptr<LinearScannedLine> LinearScannedLinePtr;
