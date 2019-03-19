/**
 * @file MotionModule/src/MovementModule/TNRSFootstep.cpp
 *
 * This file implements the struct TNRSFootstep
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 06 Oct 2017
 */

#include "Utils/include/Constants.h"
#include "Utils/include/DataHolders/TNRSFootstep.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/JsonUtils.h"

template <typename Scalar>
TNRSFootstep<Scalar>::TNRSFootstep(
  const RobotPose2D<Scalar>& pose2D,
  const RobotFeet& foot,
  const Matrix<Scalar, 4, 4>& trans,
  const Scalar& timeAtFinish
  ) :
  pose2D(pose2D),
  foot(foot),
  trans(trans),
  timeAtFinish(timeAtFinish)
{
}

template <typename Scalar>
void TNRSFootstep<Scalar>::print() const
{
  LOG_INFO("Pose2D: " << pose2D.get().transpose());
  LOG_INFO("Foot: " << toUType(foot));
  LOG_INFO("Transformation:\n" << trans);
}

template <typename Scalar>
Json::Value TNRSFootstep<Scalar>::getJson() const
{
  Json::Value val;
  val["foot"] = toUType(foot);
  val["pose2D"] = pose2D.getJson();
  val["trans"] = JsonUtils::getJson(trans);
  val["timeAtFinish"] = timeAtFinish;
  return val;
}

template <typename Scalar>
cv::RotatedRect TNRSFootstep<Scalar>::getFootRect(
  const cv::Point2f& offset, const Scalar& multiplier)
{
  auto center = cv::Point2f(pose2D.getX() * multiplier + offset.x, pose2D.getY() * multiplier + offset.y);
  auto size = cv::Size2f(Constants::footSizeX * multiplier, Constants::footSizeY * multiplier);
  return cv::RotatedRect(center, size, pose2D.getTheta() * MathsUtils::RAD_TO_DEG);
}

template class TNRSFootstep<float>;
template class TNRSFootstep<double>;
