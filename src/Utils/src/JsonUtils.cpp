/**
 * @file Utils/src/JsonUtils.cpp
 *
 * This file defines the class JsonUtils
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

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
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "Utils/include/JsonUtils.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/BehaviorInfo.h"
#include "Utils/include/DataHolders/CommMessage.h"
#include "Utils/include/DataHolders/Camera.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/PositionInput.h"
#include "Utils/include/DataHolders/GoalInfo.h"
#include "Utils/include/DataHolders/Landmark.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/DataHolders/ObstacleType.h"
#include "Utils/include/DataHolders/OccupancyMap.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "Utils/include/DataHolders/PlanningState.h"
#include "Utils/include/DataHolders/RoboCupGameControlData.h"
#include "Utils/include/DataHolders/StiffnessState.h"
#include "Utils/include/DataHolders/WorldBallInfo.h"
#include "Utils/include/DataHolders/TeamRobot.h"
#include "Utils/include/DataHolders/TNRSFootstep.h"
#include "Utils/include/HardwareIds.h"
#include "VisionModule/include/FeatureExtractionIds.h"

#define DEFINE_LINKED_JSON_TO_TYPE(TYPE) \
void jsonToType(TYPE& var, Json::Value val, const TYPE& def) { \
  if (!val.empty()) { \
    var = TYPE::jsonToType(val); \
  } else { \
    var = def; \
  } \
}

#define DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(TYPE) \
void jsonToType(TYPE& var, Json::Value val, const TYPE& def) { \
  LOG_ERROR("Function jsonToType not implemented for type " << #TYPE << "."); \
}

#define DEFINE_ENUM_CLASS_JSON_TO_TYPE(NAME) \
void jsonToType(NAME& var, Json::Value val, const NAME& def) \
{ \
  if (!val.empty()) { \
    var = static_cast<NAME>(val.asUInt()); \
  } else { \
    var = def; \
  } \
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
  Json::Value readJson(const string& path) {
    try {
      Json::Value json;
      using namespace std;
      ifstream config(path, ifstream::binary);
      config >> json;
      return json;
    } catch (Json::Exception& e) {
      LOG_EXCEPTION("Error while reading json configuration:\n\t" << path << "\n" << e.what());
      return Json::nullValue;
    }
  }

  template<typename T>
  Json::Value getJson(const T& var)
  {
    return Json::Value(var);
  }
  template Json::Value getJson<bool>(const bool&);
  template Json::Value getJson<unsigned>(const unsigned&);
  template Json::Value getJson<int>(const int&);
  template Json::Value getJson<float>(const float&);
  template Json::Value getJson<double>(const double&);
  template Json::Value getJson<string>(const string&);

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
  template Json::Value getJson<bool>(const vector<bool>&);
  template Json::Value getJson<float>(const vector<float>&);
  template Json::Value getJson<double>(const vector<double>&);
  template Json::Value getJson<TeamRobot<float> >(const vector<TeamRobot<float> >&);
  template Json::Value getJson<TeamRobot<double> >(const vector<TeamRobot<double> >&);
  template Json::Value getJson<TNRSFootstep<float> >(const vector<TNRSFootstep<float> >&);
  template Json::Value getJson<boost::shared_ptr<KnownLandmark<float> > >(const vector<boost::shared_ptr<KnownLandmark<float> > >&);
  template Json::Value getJson<boost::shared_ptr<UnknownLandmark<float> > >(const vector<boost::shared_ptr<UnknownLandmark<float> > >&);

  template<typename T>
  Json::Value getJson(const boost::shared_ptr<T>& var)
  {
    return getJson(*var);
  }
  template Json::Value getJson<Landmark<float> >(const boost::shared_ptr<Landmark<float> >&);
  template Json::Value getJson<KnownLandmark<float> >(const boost::shared_ptr<KnownLandmark<float> >&);
  template Json::Value getJson<UnknownLandmark<float> >(const boost::shared_ptr<UnknownLandmark<float> >&);

  /*template<typename Derived>
  Json::Value matrixToJson(const MatrixBase<Derived>& mat)
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

  Json::Value matrixToJson(const cv::Mat& mat)
  {
    Json::Value jsonMat;
    for (int i = 0; i < mat.rows; ++i) {
      jsonMat.append(Json::Value::null);
      for (int j = 0; j < mat.cols; ++j) {
        jsonMat[i].append(mat.at<float>(i ,j));
      }
    }
    return jsonMat;
  }

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

  Json::Value getJson(const TNSPLModules& id) {
    return getJson(toUType(id));
  }

  Json::Value getJson(const FeatureExtractionIds& id) {
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
  Json::Value getJson(const PositionInput<T>& positionInput) {
    return positionInput.getJson();
  }
  template Json::Value getJson<float>(const PositionInput<float>&);
  template Json::Value getJson<double>(const PositionInput<double>&);

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

  template<typename T>
  Json::Value getJson(const TNRSFootstep<T>& fs) {
    return fs.getJson();
  }
  template Json::Value getJson<float>(const TNRSFootstep<float>&);
  template Json::Value getJson<double>(const TNRSFootstep<double>&);

  template<typename T>
  Json::Value getJson(const Landmark<T>& l) {
    return l.getJson();
  }
  template Json::Value getJson<float>(const Landmark<float>&);
  template Json::Value getJson<double>(const Landmark<double>&);

  template<typename T>
  Json::Value getJson(const KnownLandmark<T>& l) {
    return l.getJson();
  }
  template Json::Value getJson<float>(const KnownLandmark<float>&);
  template Json::Value getJson<double>(const KnownLandmark<double>&);

  template<typename T>
  Json::Value getJson(const UnknownLandmark<T>& l) {
    return l.getJson();
  }
  template Json::Value getJson<float>(const UnknownLandmark<float>&);
  template Json::Value getJson<double>(const UnknownLandmark<double>&);

  template<typename Derived>
  Json::Value getJson(const MatrixBase<Derived>& mat)
  {
    return matrixToJson(mat);
  }

  template<typename Scalar, size_t Rows, size_t Cols>
  Json::Value getJson(const Matrix<Scalar, Rows, Cols>& mat)
  {
    return matrixToJson(mat);
  }
  template Json::Value getJson<float, 2, 1>(const Matrix<float, 2, 1>&);
  template Json::Value getJson<float, 3, 1>(const Matrix<float, 3, 1>&);
  template Json::Value getJson<float, -1, 1>(const Matrix<float, -1, 1>&);

  Json::Value getJson(const Matrix4f& mat) {
    return matrixToJson(mat);
  }

  Json::Value getJson(const Matrix4d& mat) {
    return matrixToJson(mat);
  }

  Json::Value getJson(const Vector2f& mat) {
    return matrixToJson(mat);
  }

  Json::Value getJson(const VectorXf& mat) {
    return matrixToJson(mat);
  }

  Json::Value getJson(const cv::Mat& mat) {
    return matrixToJson(mat);
  }

  Json::Value getJson(const CommMessage& cMsg) {
    return cMsg.getJson();
  }

  Json::Value getJson(const HeadTargetTypes& state) {
    return getJson(toUType(state));
  }

  Json::Value getJson(const KeyFrameGetupTypes& state) {
    return getJson(toUType(state));
  }

  Json::Value getJson(const KeyFrameDiveTypes& state) {
    return getJson(toUType(state));
  }

  DEFINE_LINKED_JSON_TO_TYPE(CommMessage)
  DEFINE_LINKED_JSON_TO_TYPE(Landmark<float>)
  DEFINE_LINKED_JSON_TO_TYPE(UnknownLandmark<float>)
  DEFINE_LINKED_JSON_TO_TYPE(KnownLandmark<float>)

  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(BallInfo<float>)
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(WorldBallInfo<float>)
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(GoalInfo<float>)
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(RobotFeet)
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(Matrix4f)
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(ObsObstacles<float>)
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(OccupancyMap<float>)
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(RoboCupGameControlData)
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(BehaviorInfo)
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(BehaviorInfoMap)
  DEFINE_UNIMPLEMENTED_JSON_TO_TYPE(TeamRobot<float>)

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
  template void jsonToType(vector<TNRSFootstep<float> >& var, Json::Value val, const vector<TNRSFootstep<float> >& def);
  template void jsonToType(vector<boost::shared_ptr<Landmark<float>> >& var, Json::Value val, const vector<boost::shared_ptr<Landmark<float>> >& def);
  template void jsonToType(vector<boost::shared_ptr<KnownLandmark<float>> >& var, Json::Value val, const vector<boost::shared_ptr<KnownLandmark<float>> >& def);
  template void jsonToType(vector<boost::shared_ptr<UnknownLandmark<float>> >& var, Json::Value val, const vector<boost::shared_ptr<UnknownLandmark<float>> >& def);

  template<typename T>
  void jsonToType(boost::shared_ptr<T>& var, Json::Value val, const boost::shared_ptr<T>& def)
  {
    if (!val.empty()) {
      jsonToType(*var, val, *def);
    } else {
      var = def;
    }
  }
  template void jsonToType(boost::shared_ptr<Landmark<float>>& var, Json::Value val, const boost::shared_ptr<Landmark<float>>& def);
  template void jsonToType(boost::shared_ptr<KnownLandmark<float>>& var, Json::Value val, const boost::shared_ptr<KnownLandmark<float>>& def);
  template void jsonToType(boost::shared_ptr<UnknownLandmark<float>>& var, Json::Value val, const boost::shared_ptr<UnknownLandmark<float>>& def);

  template <typename Scalar>
  void jsonToType(
    TNRSFootstep<Scalar>& var,
    Json::Value val,
    const TNRSFootstep<Scalar>& def)
  {
    JsonUtils::jsonToType(var.foot, Json::Value(val["foot"]), def.foot);
    JsonUtils::jsonToType(var.pose2D, Json::Value(val["pose2D"]), def.pose2D);
    JsonUtils::jsonToType(var.trans, Json::Value(val["trans"]), def.trans);
    JsonUtils::jsonToType(var.timeAtFinish, Json::Value(val["timeAtFinish"]), def.timeAtFinish);
  }
  template void jsonToType<float>(
    TNRSFootstep<float>& var, Json::Value val, const TNRSFootstep<float>& def);

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

  template <typename Scalar>
  void jsonToType(
    PositionInput<Scalar>& var,
    Json::Value val,
    const PositionInput<Scalar>& def)
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
    PositionInput<float>& var, Json::Value val, const PositionInput<float>& def);
  template void jsonToType<double>(
    PositionInput<double>& var, Json::Value val, const PositionInput<double>& def);


  template <typename Scalar, size_t Rows, size_t Cols>
  void jsonToType(
    Matrix<Scalar, Rows, Cols>& var, Json::Value val, const Matrix<Scalar, Rows, Cols>& def)
  {
    if (!val.empty()) {
      if (Rows > 1 && Cols > 1) {
        for (int i = 0; i < val.size(); ++i) {
          for (int j = 0; j < val.size(); ++j) {
            if (val[i][j].asFloat() != Json::nullValue) {
              var(i, j) = static_cast<Scalar>(val[i][j].asFloat());
            } else {
              LOG_ERROR("Invalid json object passed to jsonToType for Json::Value to Eigen::Matrix conversion")
            }
          }
        }
      } else {
        for (int i = 0; i < val.size(); ++i) {
          if (val[i].asFloat() != Json::nullValue) {
            var[i] = static_cast<Scalar>(val[i].asFloat());
          } else {
            LOG_ERROR("Invalid json object passed to jsonToType for Json::Value to Eigen::Matrix conversion")
          }
        }
      }
    } else {
      var = def;
    }
  }
  template void jsonToType<float, 2, 1>(
    Matrix<float, 2, 1>& var, Json::Value val, const Matrix<float, 2, 1>& def);

  void jsonToType(Matrix<unsigned, 3, 1>& var, Json::Value val, const Matrix<unsigned, 3, 1>& def)
  {
    if (!val.empty()) {
      var[0] = val[0].asUInt();
      var[1] = val[1].asUInt();
      var[2] = val[2].asUInt();
    } else {
      var = def;
    }
  }

  void jsonToType(Vector2f& var, Json::Value val, const Vector2f& def)
  {
    if (!val.empty()) {
      var[0] = val[0].asFloat();
      var[1] = val[1].asFloat();
    } else {
      var = def;
    }
  }

  template <typename T>
  void jsonToType(cv::Point_<T>& var, Json::Value val, const cv::Point_<T>& def)
  {
    if (!val.empty()) {
      var.x = val[0].asFloat();
      var.y = val[1].asFloat();
    } else {
      var = def;
    }
  }
  template void jsonToType<float>(cv::Point_<float>&, Json::Value, const cv::Point_<float>&);
  template void jsonToType<double>(cv::Point_<double>&, Json::Value, const cv::Point_<double>&);

  void jsonToType(VectorXf& var, Json::Value val, const VectorXf& def)
  {
    if (!val.empty()) {
      for (int i = 0; i < val.size(); ++i) {
        if (val[i].asFloat() != Json::nullValue) {
          var[i] = val[i].asFloat();
        } else {
          LOG_ERROR("Invalid json object passed to jsonToType for Json::Value to Eigen::Matrix conversion");
        }
      }
    } else {
      var = def;
    }
  }

  void jsonToType(Mat& var, Json::Value val, const Mat& def)
  {
    if (!val.empty()) {
      for (int i = 0; i < var.rows; ++i) {
        for (int j = 0; j < var.cols; ++j) {
          if (val[i][j].asFloat() != Json::nullValue) {
            var.at<float>(i, j) = val[i][j].asFloat();
          } else {
            LOG_ERROR("Invalid json object passed to jsonToType for Json::Value to Eigen::Matrix conversion")
          }
        }
      }
    } else {
      var = def;
    }
  }

  DEFINE_ENUM_CLASS_JSON_TO_TYPE(KeyFrameDiveTypes)
  DEFINE_ENUM_CLASS_JSON_TO_TYPE(KeyFrameGetupTypes)
  DEFINE_ENUM_CLASS_JSON_TO_TYPE(HeadTargetTypes)
  DEFINE_ENUM_CLASS_JSON_TO_TYPE(PostureState)
  DEFINE_ENUM_CLASS_JSON_TO_TYPE(StiffnessState)
  DEFINE_ENUM_CLASS_JSON_TO_TYPE(PlanningState)
  DEFINE_ENUM_CLASS_JSON_TO_TYPE(LinkChains)
  DEFINE_ENUM_CLASS_JSON_TO_TYPE(CameraId)
  DEFINE_ENUM_CLASS_JSON_TO_TYPE(FeatureExtractionIds)

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
  DEFINE_JSON_TO_TYPE_CONFIG(HeadScanConfig)
  DEFINE_JSON_TO_TYPE_CONFIG(HeadTargetTrackConfig)
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
