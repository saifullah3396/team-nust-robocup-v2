/**
 * @file Utils/include/TNRSLine.h
 *
 * This file defines the struct TNRSLine
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

/**
 * @struct TNRSLine
 * @brief Holds information about a line
 */
template<typename T>
struct TNRSLine
{
  TNRSLine() = default;
  TNRSLine(const TNRSLine&) = default;
  TNRSLine(TNRSLine&&) = default;
  TNRSLine& operator=(const TNRSLine&) & = default;
  TNRSLine& operator=(TNRSLine&&) & = default;
  virtual ~TNRSLine() {}
  TNRSLine(const cv::Point_<T>& p1, const cv::Point_<T>& p2) ;

  void setup();

  template <typename U>
  bool findIntersection(const TNRSLine<U>& l, cv::Point_<T>& inter);

  template <typename U>
  T findShortest(const TNRSLine<U>& other);

  template <typename U>
  T findShortest(const boost::shared_ptr<TNRSLine<U> > & other);

  cv::Point_<T> p1; //! One of the two end points of the TNRSLine
  cv::Point_<T> p2; //! One of the two end points of the TNRSLine
  cv::Point_<T> diff; //! Difference of points
  cv::Point_<T> unit; //! Unit vector defining the TNRSLine orientation
  cv::Point_<T> perp; //! Perpendicular to TNRSLine
  T perpDist = {0.0}; //! Perp distance to another TNRSLine
  T angle = {0.0}; //! Angle
  T d = {0.0}; //! Length
  boost::shared_ptr<std::vector<cv::Point_<T> > > points; //! Vector of points
};
