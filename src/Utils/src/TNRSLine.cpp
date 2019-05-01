/**
 * @file Utils/src/TNRSLine.cpp
 *
 * This file implements the struct TNRSLine
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include "Utils/include/TNRSLine.h"
#include "Utils/include/MathsUtils.h"

using namespace MathsUtils;

template<typename T>
TNRSLine<T>::TNRSLine(const cv::Point_<T>& p1, const cv::Point_<T>& p2) :
  p1(p1), p2(p2)
{
  diff = p2 - p1;
  d = norm(diff);
  unit = diff / d;
  perp.x = -unit.y;
  perp.y = unit.x;
  angle = atan2(unit.y, unit.x);
  perpDist = perp.x * p1.x + perp.y * p1.y;
}

template<typename T>
void TNRSLine<T>::setup() {
  diff = p2 - p1;
  d = norm(diff);
  unit = diff / d;
  perp.x = unit.y;
  perp.y = -unit.x;
  angle = atan2(unit.y, unit.x);
  perpDist = perp.x * p1.x + perp.y * p1.y;
}

template<typename T>
template <typename U>
bool TNRSLine<T>::findIntersection(const TNRSLine<U>& l, cv::Point_<T>& inter)
{
  auto p3 = l.p1;
  auto p4 = l.p2;
  auto p1p2 = p1 - p2;
  auto p3p4 = p3 - p4;
  auto det = p1p2.x * p3p4.y - p1p2.y * p3p4.x;
  if (det != 0.0) {
    ///< det == 0 -> TNRSLines are parallel
    auto c1 = p1.x * p2.y - p1.y * p2.x;
    auto c2 = p3.x * p4.y - p3.y * p4.x;
    inter.x = (c1 * p3p4.x - p1p2.x * c2) / det;
    inter.y = (c1 * p3p4.y - p1p2.y * c2) / det;
    return true;
  } else {
    return false;
  }
}
template bool TNRSLine<float>::findIntersection(const TNRSLine<float>& other, cv::Point_<float>& inter);
template bool TNRSLine<float>::findIntersection(const TNRSLine<double>& other, cv::Point_<float>& inter);
template bool TNRSLine<double>::findIntersection(const TNRSLine<float>& other, cv::Point_<double>& inter);
template bool TNRSLine<double>::findIntersection(const TNRSLine<double>& other, cv::Point_<double>& inter);


template<typename T>
template <typename U>
T TNRSLine<T>::findShortest(const TNRSLine<U>& other)
{
  return perp.x * other.p1.x + perp.y * other.p1.y - perpDist;
}
template float TNRSLine<float>::findShortest(const TNRSLine<float>& other);
template float TNRSLine<float>::findShortest(const TNRSLine<double>& other);
template double TNRSLine<double>::findShortest(const TNRSLine<float>& other);
template double TNRSLine<double>::findShortest(const TNRSLine<double>& other);

template<typename T>
template <typename U>
T TNRSLine<T>::findShortest(const boost::shared_ptr<TNRSLine<U> >& other) {
  return perp.x * other->p1.x + perp.y * other->p1.y - perpDist;
}

template<typename T>
bool TNRSLine<T>::findIntersectionWithCircle(const cv::Point_<T>& circleCenter, const T& radius)
{
  auto p1c = p1 - circleCenter;
  auto p2c = p2 - circleCenter;
  auto diff = p2c - p1c;
  auto sqrdDiff = diff.x * diff.x + diff.y * diff.y;
  auto D = p1c.x*p2c.y - p2c.x*p1c.y;
  return radius * radius * sqrdDiff > D * D;
}

template bool TNRSLine<float>::findIntersectionWithCircle(const cv::Point_<float>& circleCenter, const float& radius);
template bool TNRSLine<double>::findIntersectionWithCircle(const cv::Point_<double>& circleCenter, const double& radius);

template class TNRSLine<float>;
template class TNRSLine<double>;
