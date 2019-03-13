/**
 * @file Utils/include/JsonUtils.cpp
 *
 * This file defines the class JsonUtils
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include "Utils/include/JsonUtils.h"
#include "BehaviorConfigs/include/BehaviorConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBBallThrowConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBDiveConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBGetupConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBKickConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBExternalInterfaceConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBKickSequenceConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBNavigationConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBRobocupConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBStartupConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBConfig.h"
#include "BehaviorConfigs/include/PBConfigs/TestSuiteConfig.h"
#include "BehaviorConfigs/include/GBConfigs/GBLedsConfig.h"
#include "BehaviorConfigs/include/GBConfigs/GBStiffnessConfig.h"
#include "BehaviorConfigs/include/GBConfigs/GBWDConfig.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/BehaviorInfo.h"
#include "Utils/include/DataHolders/Camera.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/GoalInfo.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/DataHolders/ObstacleType.h"
#include "Utils/include/DataHolders/OccupancyMap.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "Utils/include/DataHolders/PlanningState.h"
#include "Utils/include/DataHolders/RoboCupGameControlData.h"
#include "Utils/include/DataHolders/StiffnessState.h"
#include "Utils/include/DataHolders/WorldBallInfo.h"
#include "Utils/include/DataHolders/TeamRobot.h"
#include "Utils/include/HardwareIds.h"

#define DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(TYPE) \
void jsonToType(TYPE& var, Json::Value val, const TYPE& def) { \
  LOG_ERROR("Function jsonToType not implemented for type " << #TYPE << "."); \
}

#define DEFINE_JSON_TO_TYPE_CONFIG(NAME) \
  void jsonToType( \
    boost::shared_ptr<NAME>& var, \
    Json::Value val, \
    const boost::shared_ptr<NAME>& def) \
  { \
    auto cfg = BehaviorConfig::makeFromJson(val); \
    if (boost::static_pointer_cast<NAME>(cfg)) \
      var = boost::static_pointer_cast<NAME>(cfg); \
    else \
      var = def; \
  } \
  \
  Json::Value getJson( \
    const boost::shared_ptr<NAME>& var) \
  { \
    if (var) \
      return var->getJson(); \
    else \
      return Json::nullValue; \
  }

namespace JsonUtils
{
  template<typename T>
  Json::Value getJson(const T& var)
  {
    return Json::Value(var);
  }
  template Json::Value getJson<string>(const string&);
  template Json::Value getJson<float>(const float&);
  template Json::Value getJson<int>(const int&);
  template Json::Value getJson<bool>(const bool&);
  template Json::Value getJson<unsigned>(const unsigned&);

  template<typename T>
  Json::Value getJson(const vector<T>& var)
  {
    Json::Value val;
    for (const auto& v : var) {
      val.append(getJson(v));
    }
    return val;
  }
  template Json::Value getJson<unsigned>(const vector<unsigned>&);
  template Json::Value getJson<int>(const vector<int>&);
  template Json::Value getJson<float>(const vector<float>&);
  template Json::Value getJson<TeamRobot<float> >(const vector<TeamRobot<float> >&);
  template Json::Value getJson<TeamRobot<double> >(const vector<TeamRobot<double> >&);

  /*template<typename Derived>
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
  }*/

  Json::Value getJson(const BaseBehaviorType& type) {
    return getJson(toUType(type));
  }

  Json::Value getJson(const ObstacleType& type) {
    return getJson(toUType(type));
  }

  Json::Value getJson(const PostureState& state) {
    return getJson(toUType(state));
  }

  Json::Value getJson(const PlanningState& state) {
    return getJson(toUType(state));
  }

  Json::Value getJson(const StiffnessState& state) {
    return getJson(toUType(state));
  }

  Json::Value getJson(const LinkChains& lc) {
    return getJson(toUType(lc));
  }

  Json::Value getJson(const RobotFeet& rf) {
    return getJson(toUType(rf));
  }

  Json::Value getJson(const CameraId& id) {
    return getJson(toUType(id));
  }


  template<typename T>
  Json::Value getJson(const BallInfo<T>& ballInfo) {
    return ballInfo.getJson();
  }
  template Json::Value getJson<float>(const BallInfo<float>&);
  template Json::Value getJson<double>(const BallInfo<double>&);

  Json::Value getJson(const BehaviorInfo& behaviorInfo) {
    return behaviorInfo.getJson();
  }

  Json::Value getJson(const BehaviorInfoMap& behaviorInfoMap) {
    Json::Value val;
    for (auto& bim : behaviorInfoMap) {
      JSON_ASSIGN_(val, bim.first, bim.second.getJson());
    }
    return val;
  }

  template<typename T>
  Json::Value getJson(const Camera<T>& camera) {
    return camera.getJson();
  }
  template Json::Value getJson<float>(const Camera<float>&);
  template Json::Value getJson<double>(const Camera<double>&);

  template<typename T>
  Json::Value getJson(const GoalInfo<T>& goalInfo) {
    return goalInfo.getJson();
  }
  template Json::Value getJson<float>(const GoalInfo<float>&);
  template Json::Value getJson<double>(const GoalInfo<double>&);

  template<typename T>
  Json::Value getJson(const Obstacle<T>& obstacle) {
    return obstacle.getJson();
  }
  template Json::Value getJson<float>(const Obstacle<float>&);
  template Json::Value getJson<double>(const Obstacle<double>&);

  template<typename T>
  Json::Value getJson(const ObsObstacles<T>& obsObstacles) {
    return obsObstacles.getJson();
  }
  template Json::Value getJson<float>(const ObsObstacles<float>&);
  template Json::Value getJson<double>(const ObsObstacles<double>&);

  template<typename T>
  Json::Value getJson(const OccupancyMap<T>& occupancyMap) {
    return occupancyMap.getJson();
  }
  template Json::Value getJson<float>(const OccupancyMap<float>&);
  template Json::Value getJson<double>(const OccupancyMap<double>&);

  template<typename T>
  Json::Value getJson(const RobotPose2D<T>& robotPose2D) {
    return robotPose2D.getJson();
  }
  template Json::Value getJson<float>(const RobotPose2D<float>&);
  template Json::Value getJson<double>(const RobotPose2D<double>&);

  template<typename T>
  Json::Value getJson(const cv::Point_<T>& p2) {
    Json::Value val;
    val.append(p2.x);
    val.append(p2.y);
    return val;
  }
  template Json::Value getJson<float>(const cv::Point_<float>&);
  template Json::Value getJson<double>(const cv::Point_<double>&);

  template<typename T>
  Json::Value getJson(const cv::Point3_<T>& p3) {
    Json::Value val;
    val.append(p3.x);
    val.append(p3.y);
    val.append(p3.z);
    return val;
  }
  template Json::Value getJson<float>(const cv::Point3_<float>&);
  template Json::Value getJson<double>(const cv::Point3_<double>&);

  Json::Value getJson(const RoboCupGameControlData& data) {
    return Json::nullValue;
  }

  template <typename T>
  Json::Value getJson(const WorldBallInfo<T>& worldBallInfo) {
    return worldBallInfo.getJson();
  }
  template Json::Value getJson(const WorldBallInfo<float>&);
  template Json::Value getJson(const WorldBallInfo<double>&);

  template<typename T>
  Json::Value getJson(const TeamRobot<T>& teamRobot) {
    return teamRobot.getJson();
  }
  template Json::Value getJson<float>(const TeamRobot<float>&);
  template Json::Value getJson<double>(const TeamRobot<double>&);

  template<typename T>
  Json::Value getJson(const VelocityInput<T>& velocityInput) {
    return velocityInput.getJson();
  }
  template Json::Value getJson<float>(const VelocityInput<float>& velocityInput);
  template Json::Value getJson<double>(const VelocityInput<double>& velocityInput);

  template<typename Derived>
  Json::Value getJson(const MatrixBase<Derived>& mat)
  {
    return MatrixToJson(mat);
  }

  template<typename Scalar, size_t Rows, size_t Cols>
  Json::Value getJson(const Matrix<Scalar, Rows, Cols>& mat)
  {
    return MatrixToJson(mat);
  }
  template Json::Value getJson<float, 2, 1>(const Matrix<float, 2, 1>&);
  template Json::Value getJson<float, 3, 1>(const Matrix<float, 3, 1>&);

  Json::Value getJson(const Matrix4f& mat) {
    return MatrixToJson(mat);
  }

  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(BallInfo<float>);
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(WorldBallInfo<float>);
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(GoalInfo<float>);
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(cv::Point_<float>);
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(RobotFeet);
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(Matrix4f);
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(ObsObstacles<float>);
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(OccupancyMap<float>);
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(PostureState);
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(StiffnessState);
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(PlanningState);
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(RoboCupGameControlData);
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(BehaviorInfo);
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(BehaviorInfoMap);
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(TeamRobot<float>);

  void jsonToType(int& var, Json::Value val, const int& def) {
    if (!val.empty())
      var = val.asInt();
    else
      var = def;
  }

  void jsonToType(unsigned& var, Json::Value val, const unsigned& def) {
    if (!val.empty())
      var = val.asUInt();
    else
      var = def;
  }

  void jsonToType(float& var, Json::Value val, const float& def) {
    if (!val.empty())
      var = val.asFloat();
    else
      var = def;
  }

  void jsonToType(double& var, Json::Value val, const double& def) {
    if (!val.empty())
      var = val.asDouble();
    else
      var = def;
  }

  void jsonToType(bool& var, Json::Value val, const bool& def) {
    if (!val.empty())
      var = val.asBool();
    else
      var = def;
  }

  void jsonToType(string& var, Json::Value val, const string& def) {
    if (!val.empty())
      var = val.asString();
    else
      var = def;
  }

  template <typename T>
  void jsonToType(vector<T>& var, Json::Value val, const vector<T>& def) {
    if (!val.empty()) {
      var.resize(val.size());
      for (int i = 0; i < val.size(); ++i)
        jsonToType(var[i], val[i], def[i]);
    } else {
      var = def;
    }
  }
  template void jsonToType(vector<int>& var, Json::Value val, const vector<int>& def);
  template void jsonToType(vector<unsigned>& var, Json::Value val, const vector<unsigned>& def);
  template void jsonToType(vector<float>& var, Json::Value val, const vector<float>& def);
  template void jsonToType(vector<TeamRobot<float> >& var, Json::Value val, const vector<TeamRobot<float> >& def);

  template <typename Scalar>
  void jsonToType(
    RobotPose2D<Scalar>& var,
    Json::Value val,
    const RobotPose2D<Scalar>& def)
  {
    if (!val.empty()) {
      var.x() = static_cast<Scalar>(val[0].asFloat());
      var.y() = static_cast<Scalar>(val[1].asFloat());
      var.theta() = static_cast<Scalar>(val[2].asFloat());
    } else {
      var = def;
    }
  }
  template void jsonToType<float>(
    RobotPose2D<float>& var, Json::Value val, const RobotPose2D<float>& def);
  template void jsonToType<double>(
    RobotPose2D<double>& var, Json::Value val, const RobotPose2D<double>& def);


  void jsonToType(LinkChains& var, Json::Value val, const LinkChains& def)
  {
    if (!val.empty()) {
      var = static_cast<LinkChains>(val[0].asUInt());
    } else {
      var = def;
    }
  }

  string jsonToMinimalString(const Json::Value& root) {
    Json::StreamWriterBuilder minimalStringBuilder;
    minimalStringBuilder["indentation"] = "";
    return Json::writeString(minimalStringBuilder, root);
  }

  DEFINE_JSON_TO_TYPE_CONFIG(MBBalanceConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(MBBallThrowConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(MBDiveConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(MBGetupConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(MBHeadControlConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(MBKickConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(MBMovementConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(MBPostureConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(PBExternalInterfaceConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(PBKickSequenceConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(PBNavigationConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(PBRobocupConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(PBStartupConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(PBConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(TestSuiteConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(GBLedsConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(GBStiffnessConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(GBWDConfig)
}
