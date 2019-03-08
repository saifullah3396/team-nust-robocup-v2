/**
 * @file Utils/include/TNRSPoint.h
 *
 * This file defines the struct TNRSPoint
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include <Eigen/Dense>

/**
 * @struct TNRSPoint
 * @brief Defines a scalable point based on Eigen::Matrix
 */
template <typename Scalar, int U>
class TNRSPoint
{
public:
  TNRSPoint() = default;
  TNRSPoint(const TNRSPoint&) = default;
  TNRSPoint(TNRSPoint&&) = default;
  TNRSPoint& operator=(const TNRSPoint&) & = default;
  TNRSPoint& operator=(TNRSPoint&&) & = default;
  TNRSPoint(const Eigen::Matrix<Scalar, U, 1>& p) : p(p) {}
  virtual ~TNRSPoint() {}

  void clip (const Scalar& min, const Scalar& max) {
    p = p.array().max(min).min(max).matrix();
  }

  Scalar norm() { return p.norm(); }
  Eigen::Matrix<Scalar, U, 1> get() const { return this->p; }
protected:
  Eigen::Matrix<Scalar, U, 1> p = {Eigen::Matrix<Scalar, U, 1>::Zero()};
};

/**
 * @struct TNRSPoint
 * @brief Defines a 2D point based on Eigen::Matrix
 */
template <typename Scalar>
class TNRSPoint2 : public TNRSPoint<Scalar, 2>
{
public:
  /**
   * @brief Constructor.
   * @param x X-coordinate
   * @param y Y-coordinate
   */
  TNRSPoint2(const Scalar& x, const Scalar& y) :
    TNRSPoint<Scalar, 2>(Eigen::Matrix<Scalar, 2, 1>(x, y))
  {
  }

  /**
   * @brief Constructor.
   */
  TNRSPoint2() = default;
  TNRSPoint2(const TNRSPoint2&) = default;
  TNRSPoint2(TNRSPoint2&&) = default;
  TNRSPoint2& operator=(const TNRSPoint2&) & = default;
  TNRSPoint2& operator=(TNRSPoint2&&) & = default;
  TNRSPoint2(const Eigen::Matrix<Scalar, 2, 1>& p) : TNRSPoint<Scalar, 2>(p) {}

  /**
   * @brief toCV Returns a cv::Point for given TNRSPoint2
   * @return cv::Point_<Scalar>
   */
  cv::Point_<Scalar> toCV() {
    return cv::Point_<Scalar>(this->p[0], this->p[1]);
  }

  template <typename OtherScalar>
  TNRSPoint2<Scalar>& operator+=(const TNRSPoint2<OtherScalar>& other)
  {
    this->p += other.p;
    return (*this);
  }

  template <typename OtherScalar>
  TNRSPoint2<Scalar>& operator-=(const TNRSPoint2<OtherScalar>& other)
  {
    this->p -= other.p;
    return (*this);
  }

  template <typename OtherScalar>
  TNRSPoint2<Scalar>& operator*=(const OtherScalar& constant)
  {
    this->p *= constant;
    return (*this);
  }

  template <typename OtherScalar>
  TNRSPoint2<Scalar>& operator/=(const OtherScalar& constant)
  {
    this->p /= constant;
    return (*this);
  }

  template <typename OtherScalar>
  const TNRSPoint2<Scalar> operator+(const TNRSPoint2<OtherScalar>& other) const
  {
    return TNRSPoint2<Scalar>(*this) += other;
  }

  template <typename OtherScalar>
  const TNRSPoint2<Scalar> operator-(const TNRSPoint2<OtherScalar>& other) const
  {
    return TNRSPoint2<Scalar>(*this) -= other;
  }

  template <typename OtherScalar>
  const TNRSPoint2<Scalar>& operator/(const OtherScalar& constant) const
  {
    return TNRSPoint2<Scalar>(*this) /= constant;
  }

  template <typename OtherScalar>
  const TNRSPoint2<Scalar>& operator*(const OtherScalar& constant) const
  {
    return TNRSPoint2<Scalar>(*this) *= constant;
  }

  Scalar& x() { return this->p[0]; }
  Scalar& y() { return this->p[1]; }
  Scalar getX() { return this->p[0]; }
  Scalar getY() { return this->p[1]; }
};

/**
 * @struct TNRSPoint
 * @brief Defines a 3D point based on Eigen::Matrix
 */
template <typename Scalar>
class TNRSPoint3 : public TNRSPoint<Scalar, 3>
{
public:
  /**
   * @brief Constructor.
   * @param x X-coordinate
   * @param y Y-coordinate
   * @param z Z-coordinate
   */
  TNRSPoint3(const Scalar& x, const Scalar& y, const Scalar& z) :
    TNRSPoint<Scalar, 3>(Eigen::Matrix<Scalar, 3, 1>(x, y, z))
  {
  }

  /**
   * @brief Constructor.
   */
  TNRSPoint3() = default;
  TNRSPoint3(const TNRSPoint3&) = default;
  TNRSPoint3(TNRSPoint3&&) = default;
  TNRSPoint3& operator=(const TNRSPoint3&) & = default;
  TNRSPoint3& operator=(TNRSPoint3&&) & = default;
  TNRSPoint3(const Eigen::Matrix<Scalar, 3, 1>& p) : TNRSPoint<Scalar, 2>(p) {}

  Scalar& x() { return this->p[0]; }
  Scalar& y() { return this->p[1]; }
  Scalar& z() { return this->p[2]; }
  Scalar getX() const { return this->p[0]; }
  Scalar getY() const { return this->p[1]; }
  Scalar getZ() const { return this->p[2]; }
};
