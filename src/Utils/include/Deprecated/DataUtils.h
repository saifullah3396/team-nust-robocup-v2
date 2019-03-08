/**
 * @file Utils/include/DataUtils.h
 *
 * This file declares the helper functions for data handling
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace Eigen;
using namespace std;
using namespace cv;

enum class ObstacleType : unsigned int;
enum class PostureState : unsigned int;
enum class PlanningState : unsigned int;
enum class StiffnessState : unsigned int;
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

/**
 * @class DataUtils
 * @brief Class that provides functions for variables data type
 *   conversions or handling.
 */
namespace DataUtils
{
  /**
   * @brief Converts a given variable to string.
   * @param var Input variable
   * @return string Output string
   */
  template<typename T>
  string varToString(const T& var);

  /**
   * @brief Converts a given string to variable.
   * @param str Input string
   */
  template<typename T>
  void stringToVarTemplate(string const &str, T& var);

  /**
   * @brief Converts a given string to variable.
   * @param str Input string
   */
  void stringToVar(string const &str, bool& var);

  /**
   * @brief Converts a given string to variable.
   * @param str Input string
   */
  void stringToVar(string const &str, int& var);

  /**
   * @brief Converts a given string to variable.
   * @param str Input string
   */
  void stringToVar(string const &str, unsigned& var);

  /**
   * @brief Converts a given string to variable.
   * @param str Input string
   */
  void stringToVar(string const &str, float& var);

  /**
   * @brief Converts a given string to variable.
   * @param str Input string
   */
  void stringToVar(string const &str, double& var);

  /**
   * @brief Converts a given string to variable.
   * @param str Input string
   */
  void stringToVar(string const &str, string& var);

  /**
   * @brief Converts a given string to variable.
   * @param str Input string
   */
  void stringToVar(string const &str, vector<int>& var);

  /**
   * @brief Converts a given byte buffer to hex string.
   * @param buffer Input byte buffer
   * @param size Size of the buffer
   * @return string Buffer contents as hex string
   */
  string bytesToHexString(const unsigned char*& buffer, const int& size = -1)
  {
    size_t bufferSize = sizeof(buffer);
    if (size != -1) bufferSize = size;
    char* converted = new char[bufferSize * 2 + 1];
    for (size_t i = 0; i < bufferSize; ++i)
      sprintf(&converted[i * 2], "%02X", buffer[i]);
    string temp(converted);
    delete[] converted;
    return temp;
  }

  /**
   * @brief Converts a given byte buffer to string.
   * @param buffer Input byte buffer
   * @param size Size of the buffer
   * @return string Buffer contents
   */
  string convertBytesToString(const unsigned char*& buffer, const int& size)
  {
    int bufferSize = sizeof(buffer);
    if (size != -1) bufferSize = size;
    string temp = "";
    for (int i = 0; i < bufferSize; i++) {
      if (buffer[i] == '\n') temp += '\n';
      temp += buffer[i];
    }
    return temp;
  }

  /**
   * @brief Returns a vector of splitted strings with respect to the
   *   given delimiter.
   * @param string Input string
   * @param delim Delimiter
   * @param elems Vector of splitted string components
   * @return vector Vector of splitted string components
   */
  vector<string>& splitString(
    const string& s, const char& delim, vector<string>& elems)
  {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
      elems.push_back(item);
    }
    return elems;
  }

  /**
   * @brief Returns a vector of splitted strings with respect to the
   *   given delimiter.
   * @param string Input string
   * @param delim Delimiter
   * @return vector Vector of splitted string components
   */
  vector<string> splitString(const string& s, const char& delim)
  {
    std::vector < std::string > elems;
    splitString(s, delim, elems);
    return elems;
  }

  /**
   * @brief This function sets the starting bracket.
   * @param out String to be modified
   * @return void
   */
  void getStringStartArray(string& out)
  {
    out += '{';
  }

  /**
   * @brief This function sets the ending bracket.
   * @param out String to be modified
   * @return void
   */
  void getStringEndArray(string& out)
  {
    out += '}';
  }

  /**
   * @brief Converts the input boolean to string and adds it to input string.
   * @param out String to be modified
   * @param value Input boolean
   * @return void
   */
  void getString(string& out, const bool& value)
  {
    int temp = value;
    out += varToString(temp);
  }

  /**
   * @brief Converts the input numeric variable to string and adds it to input string.
   * @param out String to be modified
   * @param value Input variable
   * @return void
   */
  template<typename T>
  void getString(string& out, const T& value)
  {
    out += varToString(value);
  }

  /**
   * @brief Converts the input cv::Point2 variable data to string and adds it
   *   to input string.
   * @param out String to be modified
   * @param value Input variable
   * @return void
   */
  template<typename T>
  void getString(string& out, const Point_<T>& value)
  {
    out += string("{") + varToString(value.x) + string(", ") + varToString(
      value.y) + string("}");
  }

  /**
   * @brief Converts the input cv::Point3 variable data to string and adds it
   *   to input string.
   * @param out String to be modified
   * @param value Input variable
   * @return void
   */
  template<typename T>
    void getString(string& out, const Point3_<T>& value)
    {
      out += string("{") + varToString(value.x) + string(", ") + varToString(
        value.y) + string(", ") + varToString(value.z) + string("}");
    }

  /**
   * @brief Converts the input ObstacleType variable data to string and adds it
   *   to input string.
   * @param out String to be modified
   * @param type Input variable
   * @return void
   */
  static void
  getString(string& out, const ObstacleType& type);

  /**
   * @brief Converts the input PostureState variable data to string and adds it
   *   to input string.
   * @param out String to be modified
   * @param state Input variable
   * @return void
   */
  static void
  getString(string& out, const PostureState& state);

  /**
   * @brief Converts the input PlanningState variable data to string and adds it
   *   to input string.
   * @param out String to be modified
   * @param state Input variable
   * @return void
   */
  static void
  getString(string& out, const PlanningState& state);

  /**
   * @brief Converts the input StiffnessState variable data to string and adds it
   *   to input string.
   * @param out String to be modified
   * @param state Input variable
   * @return void
   */
  static void
  getString(string& out, const StiffnessState& state);

  /**
   * @brief Converts the input BallInfo variable data to string and adds it
   *   to input string.
   * @param out String to be modified
   * @param ballInfo Input variable
   * @return void
   */
  static void
  getString(string& out, const BallInfo& ballInfo);

  /**
   * @brief Converts the input BehaviorInfo variable data to string and adds it
   *   to input string.
   * @param out String to be modified
   * @param behaviorInfo Input variable
   * @return void
   */
  static void
  getString(string& out, const BehaviorInfo& behaviorInfo);

  /**
   * @brief Converts the input BehaviorInfoMap variable data to string and adds it
   *   to input string.
   * @param out String to be modified
   * @param BehaviorInfoMap Input variable
   * @return void
   */
  static void
  getString(string& out, const BehaviorInfoMap& behaviorInfo);

  /**
   * @brief Converts the input Camera variable data to string and adds it to
   *   string.
   * @param out String to be modified
   * @param camera Input variable
   * @return void
   */
  static void
  getString(string& out, const Camera& camera);

  /**
   * @brief Converts the input GoalInfo variable data to string and adds it
   *   to input string.
   * @param out String to be modified
   * @param goalInfo Input variable
   * @return void
   */
  static void
  getString(string& out, const GoalInfo& goalInfo);

  /**
   * @brief Converts the input JointRequest variable data to string and adds it
   *   to input string.
   * @param out String to be modified
   * @param req Input variable
   * @return void
   */
  //static void
  //getString(string& out, const JointRequest& req);

  /**
   * @brief Converts the input LedRequest variable data to string and adds it
   *   to input string.
   * @param out String to be modified
   * @param req Input variable
   * @return void
   */
  //static void
  //getString(string& out, const LedRequest& req);

  /**
   * @brief Converts the input Obstacle variable data to string and adds it to
   *   string.
   * @param out String to be modified
   * @param obstacle Input variable
   * @return void
   */
  static void
  getString(string& out, const Obstacle& obstacle);

  /**
   * @brief Converts the input ObsObstacles variable data to string and
   *   adds it to string.
   * @param out String to be modified
   * @param obsObstacles Input variable
   * @return void
   */
  static void
  getString(string& out, const ObsObstacles& obsObstacles);

  /**
   * @brief Converts the input OccupancyMap variable data to string and adds it to the
   *   string.
   * @param out String to be modified
   * @param OccupancyMap Input variable
   * @return void
   */
  static void
  getString(string& out, const OccupancyMap& occupancyMap);

  /**
   * @brief Converts the input RobotPose2D variable data to string and adds it to
   *   string.
   * @param out String to be modified
   * @param robotPose2D Input variable
   * @return void
   */
  template<typename T>
    void
    getString(string& out, const RobotPose2D<T>& robotPose2D)
    {
      getStringStartArray(out);
      getString(out, robotPose2D.getX());
      out += ',';
      getString(out, robotPose2D.getY());
      out += ',';
      getString(out, robotPose2D.getTheta());
      getStringEndArray(out);
    }

  /**
   * @brief Converts the input RoboCupGameControlData variable data
   *   to string and adds it to input string.
   * @param out String to be modified
   * @param data Input variable
   * @return void
   */
  static void
  getString(string& out, const RoboCupGameControlData& data);

  /**
   * @brief Converts the input StiffnessRequest variable data and
   *   adds it to input string.
   * @param out String to be modified
   * @param req Input variable
   * @return void
   */
  //static void
  //getString(string& out, const StiffnessRequest& req);

  /**
   * @brief Converts the input WorldBallInfo variable data to string and adds it
   *   to input string.
   * @param out String to be modified
   * @param ballInfo Input variable
   * @return void
   */
  template <typename T>
  static void
  getString(string& out, const WorldBallInfo<T>& worldBallInfo);

  /**
   * @brief Converts the input TeamRobot variable data to string and adds it to
   *   string.
   * @param out String to be modified
   * @param teamRobot Input TeamRobot variable
   * @return void
   */
  static void
  getString(string& out, const TeamRobot& teamRobot);

  /**
   * @brief Converts the input VelocityInput variable data to string and adds it to
   *   string.
   * @param out String to be modified
   * @param VelocityInput Input variable
   * @return void
   */
  template<typename T>
    void
    getString(string& out, const VelocityInput<T>& velocityInput)
    {
      getStringStartArray(out);
      getString(out, velocityInput.dX);
      out += ',';
      getString(out, velocityInput.dY);
      out += ',';
      getString(out, velocityInput.dTheta);
      getStringEndArray(out);
    }

  /**
   * @brief Converts the input cv::Mat variable data to string and adds it to
   *   string.
   * @param out String to be modified
   * @param mat Input variable
   * @return void
   */
  void
  getString(string& out, const Mat& mat)
  {
    out += 1;
  }

  /**
   * @brief Converts the input Vector2f variable data to string and adds it to
   *   string.
   * @param out String to be modified
   * @param vec Input variable
   * @return void
   */
  //static void
  //getString(string& out, const Matrix<float, 2, 1, 0, 2, 1>& vec);

  /**
   * @brief Converts the input RotatedRect variable data to string and adds it to
   *   string.
   * @param out String to be modified
   * @param teamRobot Input RotatedRect variable
   * @return void
   */
  static void
  getString(string& out, const RotatedRect& rRect);

  /**
   * @brief Converts the input eigen matrix data to a string and adds it to
   *   string.
   * @param out String to be modified
   * @param eigenMat Input variable
   * @return void
   */
  template<typename Scalar> static void
  getString(string& out, const Matrix<Scalar, 4, 4>& eigenMat);

  /**
   * @brief Converts the input vector of variable data to string and adds it to
   *   string.
   * @param out String to be modified.
   * @param value Input vector
   * @return void
   */
  template<typename T>
  void getString(string& out, const vector<T>& value)
  {
    size_t size = value.size();
    int commaLimit = size - 1;
    getStringStartArray(out);
    for (size_t j = 0; j < size; ++j) {
      getString(out, value[j]);
      if (j != commaLimit) {
        out += ',';
      }
    }
    getStringEndArray(out);
  }
}
