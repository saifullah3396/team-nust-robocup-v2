/**
 * @file VisionModule/include/FeatureExtraction/FittedLine.h
 *
 * This file defines the struct FittedLine.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include "Utils/include/VisionUtils.h"
#include "Utils/include/TNRSLine.h"

/**
 * @struct FittedLine
 * @brief Holds information about a line
 */
struct FittedLine : TNRSLine<float>
{
  FittedLine() = default;
  FittedLine(const FittedLine&) = default;
  FittedLine(FittedLine&&) = default;
  FittedLine& operator=(const FittedLine&) & = default;
  FittedLine& operator=(FittedLine&&) & = default;
  virtual ~FittedLine() {}

  FittedLine(const cv::Point2f& p1, const cv::Point2f& p2) :
    TNRSLine<float>(p1, p2) {}

  bool circleLine = {false};
};

typedef boost::shared_ptr<FittedLine> FittedLinePtr;
