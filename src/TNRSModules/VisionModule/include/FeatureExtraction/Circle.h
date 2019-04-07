/**
 * @file VisionModule/include/FeatureExtraction/Circle.h
 *
 * This file defines the struct Circle.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include <vector>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;
/**
 * @struct Circle
 * @brief Holds information about a Circle
 */
struct Circle
{
  /**
   * Constructor from two points and radius
   */
  Circle(const Point2f& center, const float& radius) :
    center(center), radius(radius)
  {}

  Circle()
  {
    center.x = 0;
    center.y = 0;
    radius = 0;
  }

  static inline vector<Circle> pointsToCircle(
    const Point2f& p1, const Point2f& p2, const float& r)
  {
    vector<Circle> cs;
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float t = sqrt(r * r / (dx * dx + dy * dy) - 1 / 4);
    float cx = (p1.x + p2.x) / 2;
    float cy = (p1.y + p2.y) / 2;
    auto c1 = Point2f(cx - t * dy, cy + t * dx);
    auto c2 = Point2f(cx + t * dy, cy - t * dx);
    cs.push_back(Circle(c1, r));
    cs.push_back(Circle(c2, r));
    return cs;
  }

  Point2f center; ///< Circle center
  float radius; ///< Circle radius
};
