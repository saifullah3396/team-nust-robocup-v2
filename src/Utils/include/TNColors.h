/**
 * @file Utils/include/TNColors.h
 *
 * This file defines the struct TNColor and enumeration TNColors
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include "Utils/include/EnumUtils.h"

/**
 * @enum TNColors
 * @brief Enumeration for Colors
 */
enum class TNColors : unsigned int
{
  white = 0,
  black,
  green,
  blue,
  red,
  yellow,
  count
};

static std::string colorNames[toUType(TNColors::count)]
{
  "White",
  "Black",
  "Green",
  "Blue",
  "Red",
  "Yellow"
};

static cv::Scalar colorToBgr[toUType(TNColors::count)]
{
  cv::Scalar(255, 255, 255),
  cv::Scalar(0, 0, 0),
  cv::Scalar(0, 255, 0),
  cv::Scalar(255, 0, 0),
  cv::Scalar(0, 0, 255),
  cv::Scalar(0, 255, 255),
};

/**
 * @struct TNColor
 * @brief Defines a color
 */
struct TNColor
{
  TNColor() = default;
  TNColor(const TNColor&) = default;
  TNColor(TNColor&&) = default;
  TNColor& operator=(const TNColor&) & = default;
  TNColor& operator=(TNColor&&) & = default;
  virtual ~TNColor() {}

  TNColor(const uint8_t& y, const uint8_t& u, const uint8_t& v)
  {
    this->y() = y;
    this->u() = u;
    this->v() = v;
  }

  bool operator<(const TNColor& rhs) const
  {
    return (this->yuv.array() < rhs.yuv.array()).all();
  }

  bool operator>(const TNColor& rhs) const
  {
    return (this->yuv.array() > rhs.yuv.array()).all();
  }

  bool operator<=(const TNColor& rhs) const
  {
    return (this->yuv.array() <= rhs.yuv.array()).all();
  }

  bool operator>=(const TNColor& rhs) const
  {
    return (this->yuv.array() >= rhs.yuv.array()).all();
  }

  unsigned& y() { return this->yuv[0]; }
  unsigned& u() { return this->yuv[1]; }
  unsigned& v() { return this->yuv[2]; }
  const unsigned& getY() const { return this->yuv[0]; }
  const unsigned& getU() const { return this->yuv[1]; }
  const unsigned& getV() const { return this->yuv[2]; }
  Eigen::Matrix<unsigned, 3, 1> yuv;
};
