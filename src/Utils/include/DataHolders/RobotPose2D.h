/**
 * @file Utils/include/DataHolders/RobotPose2D.h
 *
 * This file declares the structs RobotPose2D, RobotPose2D, and VelocityInput.
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 18 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <vector>
#include "Utils/include/DataHolders/DataHolder.h"
#include "Utils/include/TNRSPoint.h"

using namespace std;
using namespace Eigen;

/**
 * @struct RobotPose2D
 * @brief A struct that defines the pose of the robot in x-y-theta.
 */
template<typename Scalar = float>
struct RobotPose2D : public TNRSPoint<Scalar, 3>, public DataHolder
{
public:
  RobotPose2D(const RobotPose2D&) = default;
  RobotPose2D(RobotPose2D&&) = default;
  RobotPose2D& operator=(const RobotPose2D&) & = default;
  RobotPose2D& operator=(RobotPose2D&&) & = default;
  virtual ~RobotPose2D() {}

  /**
   * @brief print Self-explanatory
   */
  void print() const override;

  /**
   * @brief getJson Returns the json object
   * @return Json::Value
   */
  Json::Value getJson() const override;

  /**
   * @brief Constructor.
   * @param x X-coordinate of the pose.
   * @param y Y-coordinate of the pose.
   * @param theta Theta-coordinate of the pose.
   */
  RobotPose2D(const Scalar& x = 0, const Scalar& y = 0, const Scalar& theta = 0);

  /**
   * @brief transform Transforms the input pose with respect to this pose
   * @param p Input pose
   * @return Transformed pose
   */
  template <typename OtherScalar>
  RobotPose2D<OtherScalar> transform(const RobotPose2D<OtherScalar>& p) const;

  /**
   * @brief transform Transforms the input pose with respect to this pose
   * @param p Input pose
   * @return Transformed pose
   */
  template <typename OtherScalar>
  Matrix<OtherScalar, 3, 1> transform(const Matrix<OtherScalar, 3, 1>& p) const;

  /**
   * @brief transform Transforms the input point with respect to this pose
   * @param p Input point
   * @return Transformed point
   */
  template <typename OtherScalar>
  Matrix<OtherScalar, 2, 1> transform(const Matrix<OtherScalar, 2, 1>& p) const;

  /**
   * @brief transform Transforms the input point with respect to this pose
   * @param p Input point
   * @return Transformed point
   */
  template <typename OtherScalar>
  cv::Point_<OtherScalar> transform(const cv::Point_<OtherScalar>& p) const;

  /**
   * @brief rotate Rotates the input point with respect to this pose
   * @param p Input point
   * @return Rotated point
   */
  template <typename OtherScalar>
  cv::Point_<OtherScalar> rotate(const cv::Point_<OtherScalar>& p) const;

  /**
   * @brief transform Transforms the input point with respect to this pose
   * @param p Input point
   * @return Transformed point
   */
  //cv::Point2f transform(const cv::Point2f& p);

  /**
   * @brief getInverse Returns the inverse pose
   * @return RobotPose2D<Scalar>
   */
  RobotPose2D<Scalar> getInverse() const;

  template <typename OtherScalar>
  RobotPose2D<Scalar>& operator+=(const RobotPose2D<OtherScalar>& other)
  {
    this->p += other.p;
    return (*this);
  }

  template <typename OtherScalar>
  RobotPose2D<Scalar>& operator-=(const RobotPose2D<OtherScalar>& other)
  {
    this->p -= other.p;
    return (*this);
  }

  template <typename OtherScalar>
  RobotPose2D<Scalar>& operator*=(const OtherScalar& constant)
  {
    this->p *= constant;
    return (*this);
  }

  template <typename OtherScalar>
  RobotPose2D<Scalar>& operator/=(const OtherScalar& constant)
  {
    this->p /= constant;
    return (*this);
  }

  template <typename OtherScalar>
  const RobotPose2D<Scalar> operator+(const RobotPose2D<OtherScalar>& other) const
  {
    return RobotPose2D<Scalar>(*this) += other;
  }

  template <typename OtherScalar>
  const RobotPose2D<Scalar> operator-(const RobotPose2D<OtherScalar>& other) const
  {
    return RobotPose2D<Scalar>(*this) -= other;
  }

  template <typename OtherScalar>
  const RobotPose2D<Scalar>& operator/(const OtherScalar& constant) const
  {
    return RobotPose2D<Scalar>(*this) /= constant;
  }

  template <typename OtherScalar>
  const RobotPose2D<Scalar>& operator*(const OtherScalar& constant) const
  {
    return RobotPose2D<Scalar>(*this) *= constant;
  }

  Scalar& x() { return this->p[0]; }
  Scalar& y() { return this->p[1]; }
  Scalar& theta() { return this->p[2]; }
  const Scalar& getX() const { return this->p[0]; }
  const Scalar& getY() const { return this->p[1]; }
  const Scalar& getTheta() const { return this->p[2]; }
  const Scalar& getCTheta() const { return ct; }
  const Scalar& getSTheta() const { return st; }

  Scalar ct = {0}; ///< Cosine of angle
  Scalar st = {0}; ///< Sine of angle
};
