/**
 * @file Utils/include/Behaviors/MBConfigs/MBKickConfig.h
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

using namespace chrono;

struct MBPostureConfig;
struct MBBalanceConfig;
struct MBHeadControlConfig;
typedef boost::shared_ptr<MBPostureConfig> MBPostureConfigPtr;
typedef boost::shared_ptr<MBBalanceConfig> MBBalanceConfigPtr;
typedef boost::shared_ptr<MBHeadControlConfig> MBHeadControlConfigPtr;

/**
 * @struct MBKickConfig
 * @brief Kick behavior configuration
 */
struct MBKickConfig : MBConfig
{
  /**
   * Constructor
   * 
   * @param type: Type of the kick behavior
   * @param ball: Initial ball position
   * @param postureConfig: If passed the robot first goes to the desired
   *   posture
   */ 
  MBKickConfig(
    const MBKickTypes& type,
    const cv::Point2f& ball,
    const MBPostureConfigPtr& postureConfig);
  
  /**
   * Constructor
   *
   * @param type: Type of the kick behavior
   * @param ball: Initial ball position
   * @param postureConfig: If passed the robot first goes to the desired
   *   posture
   * @param balanceConfig: If passed the balancer config is used for
   *   shifting balance of the robot to the support leg
   */
  MBKickConfig(
    const MBKickTypes& type,
    const cv::Point2f& ball,
    const MBPostureConfigPtr& postureConfig,
    const MBBalanceConfigPtr& balanceConfig);

  /**
   * Constructor
   *
   * @param type: Type of the kick behavior
   * @param ball: Initial ball position
   * @param balanceConfig: If passed the balancer config is used for
   *   shifting balance of the robot to the support leg
   */
  MBKickConfig(
    const MBKickTypes& type,
    const cv::Point2f& ball,
    const MBBalanceConfigPtr& balanceConfig);

  /**
   * Constructor
   * 
   * @param type: Type of the kick behavior
   * @param ball: Initial ball position
   */ 
  MBKickConfig(
    const MBKickTypes& type,
    const cv::Point2f& ball);

  /**
   * Constructor
   *
   * @param type: Type of the kick behavior
   */
  MBKickConfig(const MBKickTypes& type);
  
  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */
  virtual Json::Value getJson();
  
  /**
   * Makes an object of type this and returns it if valid
   */ 
  static boost::shared_ptr<MBKickConfig> 
    makeFromJson(const Json::Value& obj);
  
  //! Target of the kick in robot frame
  cv::Point2f target;

  //! Target of the kick in robot frame
  Matrix<float, 2, 1> targetDistAngle;
  
  //! Required velocity of the end-effector
  cv::Point2f reqVel;
  
  //! Initial position of the ball
  cv::Point2f ball;

  //! Posture configuration
  MBPostureConfigPtr postureConfig;

  //! Balance configuration
  MBBalanceConfigPtr balanceConfig;
};
typedef boost::shared_ptr<MBKickConfig> MBKickConfigPtr;

/**
 * @struct JSImpKickConfig
 * @brief Joint space base kick behavior configuration
 */
struct JSKickConfig : MBKickConfig
{
  /**
   * Constructor
   * 
   * @param type: Type of the kick behavior
   * @param ball: Initial ball position
   * @param postureConfig: If passed the robot first goes to the desired
   *   posture
   * @param balanceConfig: If passed the balancer config is used for 
   *   shifting balance of the robot to the support leg
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSKickConfig(
    const MBKickTypes& type,
    const cv::Point2f& ball,
    const boost::shared_ptr<MBPostureConfig>& postureConfig,
	const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
	const float& minTimeToKick);
  
  /**
   * Constructor
   * 
   * @param type: Type of the kick behavior
   * @param ball: Initial ball position
   * @param balanceConfig: If passed the balancer config is used for 
   *   shifting balance of the robot to the support leg
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSKickConfig(
	const MBKickTypes& type,
    const cv::Point2f& ball,
	const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
	const float& minTimeToKick);
  
  /**
   * Constructor
   * 
   * @param type: Type of the kick behavior
   * @param ball: Initial ball position
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSKickConfig(
    const MBKickTypes& type,
    const cv::Point2f& ball,
    const float& minTimeToKick);
  
  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);

  virtual Json::Value getJson();
  
  //! Minimum possible time for overall kick trajectory.
  float minTimeToKick = {1.f};

  //! Whether to perform real-time balance during kicking
  bool inKickBalance = {true};
};
typedef boost::shared_ptr<JSKickConfig> JSKickConfigPtr;

/**
 * @struct JSOImpKickConfig
 * @brief Joint space optimized impulse kick behavior configuration
 */
struct JSOImpKickConfig : JSKickConfig
{
  /**
   * Constructor
   * 
   * @param ball: Initial ball position
   * @param postureConfig: If passed the robot first goes to the desired
   *   posture
   * @param balanceConfig: If passed the balancer config is used for 
   *   shifting balance of the robot to the support leg
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSOImpKickConfig(
    const cv::Point2f& ball,
    const boost::shared_ptr<MBPostureConfig>& postureConfig,
    const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
    const float& minTimeToKick = 1.f);
  
  /**
   * Constructor
   * 
   * @param ball: Initial ball position
   * @param balanceConfig: If passed the balancer config is used for 
   *   shifting balance of the robot to the support leg
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSOImpKickConfig(
    const cv::Point2f& ball,
    const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
    const float& minTimeToKick = 1.f);
  
  /**
   * Constructor
   * 
   * @param ball: Initial ball position
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSOImpKickConfig(
    const cv::Point2f& ball = cv::Point2f(0.f, 0.f),
    const float& minTimeToKick = 1.f);
  
  /**
   * @derived
   */  
  void validate();
  
  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */ 
  virtual Json::Value getJson();
};
typedef boost::shared_ptr<JSOImpKickConfig> JSOImpKickConfigPtr;

/**
 * @struct JSE2DImpKickConfig
 * @brief Joint space estimated 2D impulse kick behavior configuration
 */
struct JSE2DImpKickConfig : JSKickConfig
{
  /**
   * Constructor
   * 
   * @param ball: Initial ball position
   * @param ballVel: Estimated initial ball velocity which ball reaches
   *   the initial position
   * @param timeUntilImpact: Estimated time until ball reaches the 
   *   initial position
   * @param timeAtEstimation: Time at which the observed ball state is 
   *   used to get an estimate
   * @param postureConfig: If passed the robot first goes to the desired
   *   posture
   * @param balanceConfig: If passed the balancer config is used for 
   *   shifting balance of the robot to the support leg
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSE2DImpKickConfig(
    const cv::Point2f& ball,
    const cv::Point2f& ballVel,
    const double& timeUntilImpact,
    const high_resolution_clock::time_point& timeAtEstimation,
    const boost::shared_ptr<MBPostureConfig>& postureConfig,
    const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
    const float& minTimeToKick = 1.f);
  
  /**
   * Constructor
   * 
   * @param ball: Initial ball position
   * @param ballVel: Estimated initial ball velocity which ball reaches
   *   the initial position
   * @param timeUntilImpact: Estimated time until ball reaches the 
   *   initial position
   * @param balanceConfig: If passed the balancer config is used for 
   *   shifting balance of the robot to the support leg
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSE2DImpKickConfig(
    const cv::Point2f& ball,
    const cv::Point2f& ballVel,
    const double& timeUntilImpact,
    const high_resolution_clock::time_point& timeAtEstimation,
    const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
    const float& minTimeToKick = 1.f);
  
  /**
   * Constructor
   * 
   * @param ball: Initial ball position
   * @param ballVel: Estimated initial ball velocity which ball reaches
   *   the initial position
   * @param timeUntilImpact: Estimated time until ball reaches the 
   *   initial position
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSE2DImpKickConfig(
    const cv::Point2f& ball,
    const cv::Point2f& ballVel,
    const double& timeUntilImpact,
    const high_resolution_clock::time_point& timeAtEstimation,
    const float& minTimeToKick = 1.f);
  
/**
   * Constructor
   * 
   * @param ball: Initial ball position
   * @param ballVel: Estimated initial ball velocity which ball reaches
   *   the initial position
   * @param timeUntilImpact: Estimated time until ball reaches the 
   *   initial position
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSE2DImpKickConfig(
    const cv::Point2f& ball = cv::Point2f(0.f, 0.f),
    const cv::Point2f& ballVel = cv::Point2f(0.f, 0.f),
    const double& timeUntilImpact = 0.0,
    const float& minTimeToKick = 1.f);
  
  /**
   * @derived
   */ 
  void validate();
  
  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */ 
  virtual Json::Value getJson();
  
  //! Estimated initial ball velocity which ball reaches the initial position
  cv::Point2f ballVel;
  
  //! Time at which the observed ball state is used to get an estimate
  high_resolution_clock::time_point timeAtEstimation;
  
  //! Estimated time until ball reaches the initial position
  double timeUntilImpact;
};
typedef boost::shared_ptr<JSE2DImpKickConfig> JSE2DImpKickConfigPtr;

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
