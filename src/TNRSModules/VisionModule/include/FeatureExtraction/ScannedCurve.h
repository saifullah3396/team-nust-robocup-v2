/**
 * @file VisionModule/include/FeatureExtraction/ScannedCurve.h
 *
 * This file defines the struct ScannedCurve.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include "Utils/include/VisionUtils.h"

/**
 * @struct ScannedCurve
 * @brief Holds information about a curve resulting from scanning the
 *   image.
 */
struct ScannedCurve
{
  ScannedCurve() = default;
  ScannedCurve(const ScannedCurve&) = default;
  ScannedCurve(ScannedCurve&&) = default;
  ScannedCurve& operator=(const ScannedCurve&) & = default;
  ScannedCurve& operator=(ScannedCurve&&) & = default;
  virtual ~ScannedCurve() {}

  vector<cv::Point> upper; ///< Upper contour of the curve
  vector<cv::Point> lower; ///< Lower contour of the curve
  boost::shared_ptr<ScannedCurve> succ; ///< Successor curve
  boost::shared_ptr<ScannedCurve> pred; ///< Preceding curve
};

typedef boost::shared_ptr<ScannedCurve> ScannedCurvePtr;
