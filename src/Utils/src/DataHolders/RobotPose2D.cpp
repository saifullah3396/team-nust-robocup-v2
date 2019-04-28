/**
 * @file Utils/src/DataHolders/RobotPose2D.cpp
 *
 * This file implements the struct RobotPose2D
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 18 Feb 2017
 */

#include <iostream>
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/JsonUtils.h"
#include "Utils/include/PrintUtils.h"

using namespace std;

template <typename Scalar>
RobotPose2D<Scalar>::RobotPose2D(const Scalar& x, const Scalar& y, const Scalar& theta) :
  TNRSPoint<Scalar, 3>(Matrix<Scalar, 3, 1>(x, y, theta)), ct(cos(theta)), st(sin(theta))
{
}

template <typename Scalar>
void RobotPose2D<Scalar>::print() const
{
  PRINT_DATA(
    RobotPose2D,
    (pose2D, this->get().transpose()),
    ("", "")
  );
}

template <typename Scalar>
Json::Value RobotPose2D<Scalar>::getJson() const
{
  Json::Value val;
  JSON_ASSIGN_(val, pose2D, JsonUtils::matrixToJson(this->get().transpose()));
  return val;
}

template <typename Scalar>
template <typename OtherScalar>
RobotPose2D<OtherScalar> RobotPose2D<Scalar>::transform(const RobotPose2D<OtherScalar>& p) const
{
  return
    RobotPose2D<OtherScalar>(
      getX() + p.getX() * ct - p.getY() * st,
      getY() + p.getX() * st + p.getY() * ct,
      MathsUtils::addAngles(static_cast<Scalar>(p.getTheta()), getTheta())
    );
}
template RobotPose2D<float> RobotPose2D<float>::transform(const RobotPose2D<float>& p) const;
template RobotPose2D<float> RobotPose2D<double>::transform(const RobotPose2D<float>& p) const;
template RobotPose2D<double> RobotPose2D<float>::transform(const RobotPose2D<double>& p) const;
template RobotPose2D<double> RobotPose2D<double>::transform(const RobotPose2D<double>& p) const;

template <typename Scalar>
template <typename OtherScalar>
Matrix<OtherScalar, 3, 1> RobotPose2D<Scalar>::transform(const Matrix<OtherScalar, 3, 1>& p) const
{
  return
    Matrix<OtherScalar, 3, 1>(
      getX() + p[0] * ct - p[1] * st,
      getY() + p[0] * st + p[1] * ct,
      MathsUtils::addAngles(static_cast<Scalar>(p[2]), getTheta())
    );
}
template Matrix<float, 3, 1> RobotPose2D<float>::transform(const Matrix<float, 3, 1>& p) const;
template Matrix<float, 3, 1> RobotPose2D<double>::transform(const Matrix<float, 3, 1>& p) const;
template Matrix<double, 3, 1> RobotPose2D<float>::transform(const Matrix<double, 3, 1>& p) const;
template Matrix<double, 3, 1> RobotPose2D<double>::transform(const Matrix<double, 3, 1>& p) const;

template <typename Scalar>
template <typename OtherScalar>
Matrix<OtherScalar, 2, 1> RobotPose2D<Scalar>::transform(const Matrix<OtherScalar, 2, 1>& p) const
{
 return Matrix<OtherScalar, 2, 1>(
     getX() + p[0] * ct - p[1] * st,
     getY() + p[0] * st + p[1] * ct
 );
}
template Matrix<float, 2, 1> RobotPose2D<float>::transform(const Matrix<float, 2, 1>& p) const;
template Matrix<float, 2, 1> RobotPose2D<double>::transform(const Matrix<float, 2, 1>& p) const;
template Matrix<double, 2, 1> RobotPose2D<float>::transform(const Matrix<double, 2, 1>& p) const;
template Matrix<double, 2, 1> RobotPose2D<double>::transform(const Matrix<double, 2, 1>& p) const;

template <typename Scalar>
template <typename OtherScalar>
cv::Point_<OtherScalar> RobotPose2D<Scalar>::transform(const cv::Point_<OtherScalar>& p) const
{
  return
    cv::Point_<OtherScalar>(
      getX() + p.x * ct - p.y * st,
      getY() + p.x * st + p.y * ct
    );
}
template cv::Point_<int> RobotPose2D<float>::transform(const cv::Point_<int>& p) const;
template cv::Point_<int> RobotPose2D<double>::transform(const cv::Point_<int>& p) const;
template cv::Point_<float> RobotPose2D<float>::transform(const cv::Point_<float>& p) const;
template cv::Point_<float> RobotPose2D<double>::transform(const cv::Point_<float>& p) const;
template cv::Point_<double> RobotPose2D<float>::transform(const cv::Point_<double>& p) const;
template cv::Point_<double> RobotPose2D<double>::transform(const cv::Point_<double>& p) const;

template <typename Scalar>
template <typename OtherScalar>
cv::Point_<OtherScalar> RobotPose2D<Scalar>::rotate(const cv::Point_<OtherScalar>& p) const
{
  return
    cv::Point_<OtherScalar>(
      p.x * ct - p.y * st,
      p.x * st + p.y * ct
    );
}
template cv::Point_<int> RobotPose2D<float>::rotate(const cv::Point_<int>& p) const;
template cv::Point_<int> RobotPose2D<double>::rotate(const cv::Point_<int>& p) const;
template cv::Point_<float> RobotPose2D<float>::rotate(const cv::Point_<float>& p) const;
template cv::Point_<float> RobotPose2D<double>::rotate(const cv::Point_<float>& p) const;
template cv::Point_<double> RobotPose2D<float>::rotate(const cv::Point_<double>& p) const;
template cv::Point_<double> RobotPose2D<double>::rotate(const cv::Point_<double>& p) const;

/*template <typename Scalar>
cv::Point2f RobotPose2D<Scalar>::transform(const cv::Point2f& p)
{
  return
    cv::Point2f(
      getX() + p.x * ct - p.y * st,
      getY() + p.x * st + p.y * ct
    );
}
template cv::Point2f RobotPose2D<float>::transform(const cv::Point2f& p);
template cv::Point2f RobotPose2D<double>::transform(const cv::Point2f& p);
*/

template <typename Scalar>
RobotPose2D<Scalar> RobotPose2D<Scalar>::getInverse() const
{
  return
    RobotPose2D<Scalar>(
      -ct * getX() - st * getY(),//-(cos(-getTheta()) * x - sin(-getTheta()) * y),
      st * getX() - ct * getY(),//-(sin(-getTheta()) * x + cos(-getTheta()) * y),
      -getTheta()
    );
}

template RobotPose2D<float> RobotPose2D<float>::getInverse() const;
template RobotPose2D<double> RobotPose2D<double>::getInverse() const;

template struct RobotPose2D<float>;
template struct RobotPose2D<double>;
