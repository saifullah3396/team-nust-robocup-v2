/**
 * @file Utils/include/JsonUtils.h
 *
 * This file declares the helper functions associated with JSON
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <iostream>
#include <jsoncpp/json/json.h>
#include <boost/shared_ptr.hpp>
#include "Eigen/Dense"
#include <opencv2/core/core.hpp>

#define JSON_ASSIGN(logRoot, key, value) logRoot[key] = value;
#define JSON_ASSIGN_(logRoot, key, value) logRoot[#key] = value;
#define JSON_APPEND(logRoot, key, value) logRoot[key].append(value);

using namespace Eigen;
using namespace std;

struct BehaviorConfig;
struct MBBalanceConfig;
struct MBBallThrowConfig;
struct MBConfig;
struct MBDiveConfig;
struct MBGetupConfig;
struct MBHeadControlConfig;
struct MBKickConfig;
struct MBMovementConfig;
struct MBPostureConfig;
struct PBExternalInterfaceConfig;
struct PBKickSequenceConfig;
struct PBNavigationConfig;
struct PBRobocupConfig;
struct PBStartupConfig;
struct PBConfig;
struct TestSuiteConfig;
struct GBLedsConfig;
struct GBStiffnessConfig;
struct GBWDConfig;

enum class LinkChains : unsigned int;
enum class RobotFeet : unsigned int;
enum class BaseBehaviorType : unsigned int;
enum class ObstacleType : unsigned int;
enum class PostureState : unsigned int;
enum class PlanningState : unsigned int;
enum class StiffnessState : unsigned int;
enum class CameraId : unsigned int;
template <typename T>
struct Point_;
template <typename T>
struct Point3_;
template <typename T>
struct RobotPose2D;
template <typename T>
struct VelocityInput;
template <typename T>
struct BallInfo;
struct BehaviorInfo;
template <typename T>
struct Camera;
template <typename T>
struct GoalInfo;
template <typename T>
struct Obstacle;
template <typename T>
struct ObsObstacles;
template <typename T>
struct OccupancyMap;
struct RoboCupGameControlData;
template <typename T>
struct WorldBallInfo;
template <typename T>
struct TeamRobot;
typedef map<unsigned, BehaviorInfo> BehaviorInfoMap;

#define DECLARE_JSON_TO_TYPE_CONFIG(NAME) \
  /** \
   * @brief jsonToType Converts a json object to behavior config \
   * @param var Output behavior config \
   * @param val JSON object \
   * @param def Default value for the behavior config \
   */ \
  void jsonToType( \
    boost::shared_ptr<NAME>& var, \
    Json::Value val, \
    const boost::shared_ptr<NAME>& def); \
  \
  /** \
   * @brief getJson Creates a JSON object from behavior config \
   * @param var Input behavior config \
   * @return JSON object \
   */ \
  Json::Value getJson( \
    const boost::shared_ptr<NAME>& var);

#define DECLARE_JSON_TO_TYPE(TYPE) \
  void jsonToType(TYPE& var, Json::Value val, const TYPE& def);

namespace JsonUtils
{
  /**
   * @brief getJson Creates a JSON object from given input variable
   * @param var Input variable
   * @return JSON object
   */
  template<typename T>
  Json::Value getJson(const T& var);

  template<typename T>
  Json::Value getJson(const vector<T>& var);

  Json::Value getJson(const BaseBehaviorType& type);

  Json::Value getJson(const ObstacleType& type);

  Json::Value getJson(const PostureState& state);

  Json::Value getJson(const PlanningState& state);

  Json::Value getJson(const StiffnessState& state);

  Json::Value getJson(const LinkChains& state);

  Json::Value getJson(const RobotFeet& state);

  Json::Value getJson(const CameraId& id);

  template<typename T>
  Json::Value getJson(const BallInfo<T>& ballInfo);

  Json::Value getJson(const BehaviorInfo& behaviorInfo);

  Json::Value getJson(const BehaviorInfoMap& behaviorInfo);

  template<typename T>
  Json::Value getJson(const Camera<T>& camera);

  template<typename T>
  Json::Value getJson(const GoalInfo<T>& goalInfo);

  template<typename T>
  Json::Value getJson(const Obstacle<T>& obstacle);

  template<typename T>
  Json::Value getJson(const ObsObstacles<T>& obsObstacles);

  template<typename T>
  Json::Value getJson(const OccupancyMap<T>& occupancyMap);

  template<typename T>
  Json::Value getJson(const cv::Point_<T>& p2);

  template<typename T>
  Json::Value getJson(const cv::Point3_<T>& p3);

  template<typename T>
  Json::Value getJson(const RobotPose2D<T>& robotPose2D);

  template<typename T>
  Json::Value getJson(const VelocityInput<T>& velocityInput);

  Json::Value getJson(const RoboCupGameControlData& data);

  template <typename T>
  Json::Value getJson(const WorldBallInfo<T>& worldBallInfo);

  template<typename T>
  Json::Value getJson(const TeamRobot<T>& teamRobot);

  /**
   * @brief MatrixToJson Creates a JSON object from Eigen matrix
   * @param mat Eigen matrix
   * @return JSON object
   */
  template<typename Derived>
  Json::Value MatrixToJson(const MatrixBase<Derived>& mat)
  {
    Json::Value jsonMat;
    for (int i = 0; i < mat.rows(); ++i) {
      jsonMat.append(Json::Value::null);
      for (int j = 0; j < mat.cols(); ++j) {
        jsonMat[i].append(mat(i ,j));
      }
    }
    return jsonMat;
  }

  /**
   * @brief getJson Wrapper for MatrixToJson()
   * @param mat Eigen matrix
   * @return JSON object
   */
  template<typename Derived>
  Json::Value getJson(const MatrixBase<Derived>& mat);

  template<typename Scalar, size_t Rows, size_t Cols>
  Json::Value getJson(const Matrix<Scalar, Rows, Cols>& mat);

  Json::Value getJson(const Matrix4f& mat);

  /**
   * @brief jsonToType Converts a JSON object to integer type
   * @param var Typed output
   * @param val JSON object
   * @param def Default value for typed output
   */
  void jsonToType(int& var, Json::Value val, const int& def);

  /**
   * @brief jsonToType Converts a JSON object to unsigned type
   * @param var Typed output
   * @param val JSON object
   * @param def Default value for typed output
   */
  void jsonToType(unsigned& var, Json::Value val, const unsigned& def);

  /**
   * @brief jsonToType Converts a JSON object to float type
   * @param var Typed output
   * @param val JSON object
   * @param def Default value for typed output
   */
  void jsonToType(float& var, Json::Value val, const float& def);

  /**
   * @brief jsonToType Converts a JSON object to double type
   * @param var Typed output
   * @param val JSON object
   * @param def Default value for typed output
   */
  void jsonToType(double& var, Json::Value val, const double& def);

  /**
   * @brief jsonToType Converts a JSON object to bool type
   * @param var Typed output
   * @param val JSON object
   * @param def Default value for typed output
   */
  void jsonToType(bool& var, Json::Value val, const bool& def);

  /**
   * @brief jsonToType Converts a JSON object to bool type
   * @param var Typed output
   * @param val JSON object
   * @param def Default value for typed output
   */
  template <typename T>
  void jsonToType(vector<T>& var, Json::Value val, const vector<T>& def);

  /**
   * @brief jsonToType Converts a JSON object to string type
   * @param var Typed output
   * @param val JSON object
   * @param def Default value for typed output
   */
  void jsonToType(string& var, Json::Value val, const string& def);

  /**
   * @brief jsonToType Converts a JSON object to RobotPose2D<Scalar> type
   * @param var Typed output
   * @param val JSON object
   * @param def Default value for typed output
   */
  template <typename Scalar>
  void jsonToType(RobotPose2D<Scalar>& var, Json::Value val, const RobotPose2D<Scalar>& def);

  /**
   * @brief jsonToType Converts a JSON object to LinkChains type
   * @param var Typed output
   * @param val JSON object
   * @param def Default value for typed output
   */
  void jsonToType(LinkChains& var, Json::Value val, const LinkChains& def);

  string jsonToMinimalString(const Json::Value& root);

  DECLARE_JSON_TO_TYPE_CONFIG(MBBalanceConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(MBBallThrowConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(MBDiveConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(MBGetupConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(MBHeadControlConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(MBKickConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(MBMovementConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(MBPostureConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(PBExternalInterfaceConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(PBKickSequenceConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(PBNavigationConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(PBRobocupConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(PBStartupConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(PBConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(TestSuiteConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(GBLedsConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(GBStiffnessConfig)
  DECLARE_JSON_TO_TYPE_CONFIG(GBWDConfig)

  //! Unimplemented conversions for memory variables. Not needed atm
  DECLARE_JSON_TO_TYPE(BallInfo<float>);
  DECLARE_JSON_TO_TYPE(WorldBallInfo<float>);
  DECLARE_JSON_TO_TYPE(GoalInfo<float>);
  DECLARE_JSON_TO_TYPE(cv::Point_<float>);
  DECLARE_JSON_TO_TYPE(RobotFeet);
  DECLARE_JSON_TO_TYPE(Matrix4f);
  DECLARE_JSON_TO_TYPE(ObsObstacles<float>);
  DECLARE_JSON_TO_TYPE(OccupancyMap<float>);
  DECLARE_JSON_TO_TYPE(PostureState);
  DECLARE_JSON_TO_TYPE(StiffnessState);
  DECLARE_JSON_TO_TYPE(PlanningState);
  DECLARE_JSON_TO_TYPE(RoboCupGameControlData);
  DECLARE_JSON_TO_TYPE(BehaviorInfo);
  typedef std::map<unsigned, BehaviorInfo> BehaviorInfoMap;
  DECLARE_JSON_TO_TYPE(BehaviorInfoMap);
  DECLARE_JSON_TO_TYPE(TeamRobot<float>);
}
