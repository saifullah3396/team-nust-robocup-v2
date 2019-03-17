/**
 * @file BehaviorConfigs/include/MBConfigs/MBKickConfig.h
 *
 * This file defines the structs MBKickConfig, JSKickConfig, 
 * JSE2DImpKickConfig and JSOImpKickConfig
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <chrono>
#include "MBConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"

using namespace chrono;

//! Since commas in macros cause problems
#define POINT_DEFINITION cv::Point2f(-1.0, -1.0)

struct MBPostureConfig;
struct MBBalanceConfig;
struct MBHeadControlConfig;
typedef boost::shared_ptr<MBPostureConfig> MBPostureConfigPtr;
typedef boost::shared_ptr<MBBalanceConfig> MBBalanceConfigPtr;
typedef boost::shared_ptr<MBHeadControlConfig> MBHeadControlConfigPtr;

/**
 * @struct MBKickConfig
 * @brief Kick behavior base configuration
 * @param target Target position in robot foot center frame
 * @param reqVel Required ball velocity on hit
 * @param ball Position of the ball in robot foot center frame
 * @param targetDistAngle Target in terms of distance and angle
 * @param postureConfig If passed the robot first goes to the desired
 *   posture
 * @param balanceConfig If passed the balancer config is used for
 *   shifting balance of the robot to the support leg
 */
DECLARE_BEHAVIOR_CONFIG_WITH_VARS(
  MBKickConfig,
  MBConfig,
  MBKickConfigPtr,
  MBIds::kick,
  15.0,
  MBKickTypes,
  (cv::Point2f, target, POINT_DEFINITION),
  (cv::Point2f, reqVel, POINT_DEFINITION),
  (cv::Point2f, ball, POINT_DEFINITION),
  (Eigen::Vector2f, targetDistAngle, Eigen::Vector2f::Zero()),
  (MBPostureConfigPtr, postureConfig, InterpToPostureConfigPtr()),
  (MBBalanceConfigPtr, balanceConfig, MBBalanceConfigPtr()),
)

/**
 * @struct JSKickConfig
 * @brief Joint space base kick behavior configuration
 * @param minTimeToKick: Minimum possible time for overall kick
 *   trajectory. This is important if we don't want the kick to be
 *   too fast even if we're using time optimization
 */
DECLARE_BEHAVIOR_CONFIG_BASE_TYPE_WITH_VARS(
  JSKickConfig,
  MBKickConfig,
  MBKickTypes,
  JSKickConfigPtr,
  (float, minTimeToKick, 1.f),
  (bool, inKickBalance, true),
)

/**
 * @struct JSOImpKickConfig
 * @brief Joint space optimized impulse kick behavior configuration
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  JSOImpKickConfig,
  JSKickConfig,
  MBKickTypes::jsoImpKick,
  JSOImpKickConfigPtr,
  (float, minTimeToKick, 1.f),
  (bool, inKickBalance, true),
)

/**
 * @struct JSE2DImpKickConfig
 * @brief Joint space estimated 2D impulse kick behavior configuration
 * @param ballVel: Estimated initial ball velocity which ball reaches
 *   the initial position
 * @param timeUntilImpact: Estimated time until ball reaches the
 *   initial position
 * @param timeAtEstimation: Time at which the observed ball state is
 *   used to get an estimate
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  JSE2DImpKickConfig,
  JSKickConfig,
  MBKickTypes::jse2DImpKick,
  JSE2DImpKickConfigPtr,
  (cv::Point2f, ballVel, cv::Point2f()),
  (double, timeUntilImpact, 0.0),
  (double, timeAtEstimation, 0.0),
)

/**
 * @struct CSpaceBSplineKick
 * @brief B-Spline based kick in Cartesian space planning
 */
DECLARE_BEHAVIOR_CONFIG_TYPE(
  CSpaceBSplineKickConfig,
  MBKickConfig,
  MBKickTypes::cSpaceBSplineKick,
  CSpaceBSplineKickConfigPtr
)
