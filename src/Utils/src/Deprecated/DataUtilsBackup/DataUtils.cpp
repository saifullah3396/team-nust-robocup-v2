/**
 * @file src/DataUtils.cpp
 *
 * This file implements the class DataUtils.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include <opencv2/opencv.hpp>
#include "BehaviorConfigs/include/BehaviorInfo.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/DataHolders/BallInfo.h"
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

namespace DataUtils
{
  template<typename T>
  string varToString(const T& var)
  {
    stringstream ss;
    try {
      ss << var;
    } catch (exception &e) {
      LOG_ERROR(
        "Exception caught while converting a variable to string: " + string(e.what()))
    }
    return ss.str();
  }

  template<typename T>
  void stringToVarTemplate(string const &str, T& var)
  {
    try {
      istringstream istr(str);
      if (!(istr >> std::boolalpha >> var)) throw("Invalid type for string to variable conversion");
    } catch (exception &e) {
      LOG_ERROR(
        "Exception caught while converting a variable to string: " + string(e.what()))
    }
  }

  void stringToVar(string const &str, bool& var)
  {
    stringToVarTemplate<bool>(str, var);
  }

  void stringToVar(string const &str, int& var)
  {
    stringToVarTemplate<int>(str, var);
  }

  void stringToVar(string const &str, unsigned& var)
  {
    stringToVarTemplate<unsigned>(str, var);
  }

  void stringToVar(string const &str, float& var)
  {
    stringToVarTemplate<float>(str, var);
  }

  void stringToVar(string const &str, double& var)
  {
    stringToVarTemplate<double>(str, var);
  }

  void stringToVar(string const &str, string& var)
  {
    var = str;
  }

  void stringToVar(string const &str, vector<int>& var)
  {
    string tmp = str;
    tmp = str.substr(1, str.size() - 2);
    vector < string > values = splitString(tmp, ',');
    if (values.size() != var.size()) return;
    for (size_t i = 0; i < values.size(); ++i) {
      stringToVarTemplate<int>(values[i], var[i]);
    }
  }

  void
  getString(string& out, const ObstacleType& type)
  {
    getString(out, (unsigned) type);
  }

  void
  getString(string& out, const PostureState& state)
  {
    getString(out, (unsigned) state);
  }

  void
  getString(string& out, const PlanningState& state)
  {
    getString(out, (unsigned) state);
  }

  void
  getString(string& out, const StiffnessState& state)
  {
    getString(out, (unsigned) state);
  }

  void
  getString(string& out, const BallInfo& ballInfo)
  {
    getStringStartArray(out);
    getString(out, ballInfo.camera);
    out += ',';
    getString(out, ballInfo.found);
    out += ',';
    getString(out, ballInfo.posRel.x);
    out += ',';
    getString(out, ballInfo.posRel.y);
    out += ',';
    getString(out, ballInfo.velRel.x);
    out += ',';
    getString(out, ballInfo.velRel.y);
    out += ',';
    getString(out, ballInfo.posImage.x);
    out += ',';
    getString(out, ballInfo.posImage.y);
    out += ',';
    getString(out, ballInfo.ballAge);
    out += ',';
    getString(out, ballInfo.radius);
    getStringEndArray(out);
  }

  void
  getString(string& out, const BehaviorInfo& behaviorInfo)
  {
    out += "BehaviorInfo";
  }

  void
  getString(string& out, const BehaviorInfoMap& behaviorInfo)
  {
    out += "BehaviorInfo";
  }  
    
  void
  getString(string& out, const Camera& camera)
  {
    getString(out, camera.name);
    out += ',';
    getString(out, camera.clientName);
    out += ',';
    getString(out, camera.width);
    out += ',';
    getString(out, camera.height);
    out += ',';
    getString(out, camera.fps);
    out += ',';
    getString(out, camera.fovX);
    out += ',';
    getString(out, camera.fovY);
    out += ',';
    getString(out, camera.focalX);
    out += ',';
    getString(out, camera.focalY);
    out += ',';
    getString(out, camera.centerOffX);
    out += ',';
    getString(out, camera.centerOffY);
  }
  
  void
  getString(string& out, const GoalInfo& goalInfo)
  {
    getStringStartArray(out);
    getString(out, goalInfo.found);
    out += ',';
    getString(out, goalInfo.leftPost.x);
    out += ',';
    getString(out, goalInfo.leftPost.y);
    out += ',';
    getString(out, goalInfo.rightPost.x);
    out += ',';
    getString(out, goalInfo.rightPost.y);
    out += ',';
    getString(out, goalInfo.type);
    getStringEndArray(out);
  }

  void
  getString(string& out, const Obstacle& obstacle)
  {
    getString(out, (unsigned) obstacle.type);
    out += ',';
    getString(out, (float) obstacle.center.x);
    out += ',';
    getString(out, (float) obstacle.center.y);
    out += ',';
    getString(out, (float) obstacle.front.p1.x);
    out += ',';
    getString(out, (float) obstacle.front.p1.y);
    out += ',';
    getString(out, (float) obstacle.front.p2.x);
    out += ',';
    getString(out, (float) obstacle.front.p2.y);
    out += ',';
    getString(out, (float) obstacle.depth);
  }

  void
  getString(string& out, const ObsObstacles& obsObstacles)
  {
    getStringStartArray(out);
    for (size_t i = 0; i < obsObstacles.data.size(); ++i) {
      getString(out, obsObstacles.data[i]);
      if (i <  obsObstacles.data.size() - 1)
        out += ',';
    }
    getStringEndArray(out);
  }

  void
  getString(string& out, const OccupancyMap& occupancyMap)
  {
    /*getString(out, occupancyMap.resolution);
    out += ',';
    getString(out, occupancyMap.data.cols);
    out += ',';
    getString(out, occupancyMap.data.rows);
    out += ',';
    getString(out, occupancyMap.originPose.x);
    out += ',';
    getString(out, occupancyMap.originPose.y);
    out += ',';
    getString(out, occupancyMap.originPose.z);
    out += ',';
    vector <uchar> bytesData;
    if (imencode(".jpg", occupancyMap.data, bytesData)) {
      const unsigned char* ptr = &bytesData[0];
      string data = bytesToHexString(ptr, bytesData.size());
      out += data;
    }
    out += ',';*/
    out += "";
  }

  void
  getString(string& out, const RoboCupGameControlData& data)
  {
    out += "RoboCupGameControlData";
  }

  template <typename T>
  void
  getString(string& out, const WorldBallInfo<T>& worldBallInfo)
  {
    getStringStartArray(out);
    getString(out, worldBallInfo.found);
    out += ',';
    getString(out, worldBallInfo.posWorld.x());
    out += ',';
    getString(out, worldBallInfo.posWorld.y());
    out += ',';
    getString(out, worldBallInfo.velWorld.x());
    out += ',';
    getString(out, worldBallInfo.velWorld.y());
    getStringEndArray(out);
  }

  void
  getString(string& out, const TeamRobot& teamRobot)
  {
    getString(out, (bool)teamRobot.dataReceived);
    out += ',';
    getString(out, (int)teamRobot.fallen);
    out += ',';
    getString(out, (int)teamRobot.intention);
    out += ',';
    getString(out, (int)teamRobot.suggestionToMe);
    out += ',';
    getString(out, teamRobot.pose.getX());
    out += ',';
    getString(out, teamRobot.pose.getY());
    out += ',';
    getString(out, teamRobot.pose.getTheta());
    out += ',';
    getString(out, teamRobot.walkingTo.getX());
    out += ',';
    getString(out, teamRobot.walkingTo.getY());
    out += ',';
    getString(out, teamRobot.shootingTo.getX());
    out += ',';
    getString(out, teamRobot.shootingTo.getY());
  }

  /*void
  getString(string& out, const Matrix<float, 2, 1, 0, 2, 1>& vec)
  {
    getStringStartArray(out);
    getString(out, vec[0]);
    out += ',';
    getString(out, vec[1]);
    getStringEndArray(out);
  }*/

  template<typename Scalar>
  void getString(string& out, const Matrix<Scalar, 4, 4>& eigenMat)
  {
    getStringStartArray(out);
    for (size_t i = 0; i < eigenMat.rows(); ++i) {
      for (size_t j = 0; j < eigenMat.cols(); ++j) {
        getString(out, eigenMat(i, j));
        if (i == eigenMat.rows() - 1 && j ==  eigenMat.cols() - 1)
          continue;
        out += ',';
      }
    }
    getStringEndArray(out);
  }

  void
  getString(string& out, const RotatedRect& rRect)
  {
    out += "Rect";
  }
}

template void getString<float>(string& out, const Matrix<float, 4, 4>& eigenMat);
template void getString<double>(string& out, const Matrix<double, 4, 4>& eigenMat);
