/**
 * @file Utils/include/DataHolders/TNRSFootstep.h
 *
 * This file defines the struct TNRSFootstep
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 06 Oct 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include "Utils/include/DataHolders/RobotPose2D.h"

template <typename Scalar>
struct RobotPose2D;
enum class RobotFeet : unsigned int;

namespace PathPlannerSpace {
  class State;
}

template <typename Scalar>
struct TNRSFootstep : public DataHolder
{
  TNRSFootstep() = default;
  TNRSFootstep(const TNRSFootstep&) = default;
  TNRSFootstep(TNRSFootstep&&) = default;
  TNRSFootstep& operator=(const TNRSFootstep&) & = default;
  TNRSFootstep& operator=(TNRSFootstep&&) & = default;
  virtual ~TNRSFootstep() {}
  TNRSFootstep(
    const RobotPose2D<Scalar>& pose2D,
    const RobotFeet& foot,
    const Eigen::Matrix<Scalar, 4, 4>& trans,
    const Scalar& timeAtFinish = 0.0
  );

  /**
   * @brief print Prints step data
   */
  void print() const final;

  Json::Value getJson() const final;

  /**
   * @brief getFootRect Makes a rotated rectangle for footstep
   * @param offset Center offset
   * @param multiplier Size multiplier
   * @return cv::RotatedRect
   */
  cv::RotatedRect getFootRect(
    const cv::Point2f& offset, const Scalar& multiplier);

  static TNRSFootstep<Scalar> fromPathPlannerState(const PathPlannerSpace::State& state);
  static vector<TNRSFootstep<Scalar>> fromPathPlannerStates(const std::vector<PathPlannerSpace::State>& states);

  RobotPose2D<Scalar> pose2D;
  RobotFeet foot;
  Eigen::Matrix<Scalar, 4, 4> trans;
  Scalar timeAtFinish;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
