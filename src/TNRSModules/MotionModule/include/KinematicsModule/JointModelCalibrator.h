/**
 * @file MotionModule/include/KinematicsModule/JointModelCalibrator.h
 *
 * This file declares the class JointModelCalibrator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#pragma once
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "Utils/include/Solvers/NLOptimizer.h"

using namespace std;

enum class Joints : unsigned int;
template <typename Scalar>
class JointEstimator;

/**
 * @class JointModelCalibrator
 * @brief A class to calibrate the joint model
 */
class JointModelCalibrator : public NLOptimizer
{
public:
  /**
   * @brief JointModelCalibrator Constructor
   * @param measFilePath Path to file containing the imu measurements
   */
  JointModelCalibrator(const Joints& jointIndex, const string& measFilePath);

  /**
   * @brief ~JointModelCalibrator Destructor
   */
  ~JointModelCalibrator() {}

  /**
   * @brief optDef Solves the optimization problem that minimizes the norm
   *   X = |PID(cmd - shifted_measurement)|
   *   for all input measurements.
   */
  virtual void optDef() override;

  /**
   * @brief printResults Prints the calibration results
   */
  void printResults();

  Eigen::Matrix<double, 3, 1> getGains() { return pid; }
  double getTimeShift() { return timeShift; }

protected:
  /**
   * Evaluates the objective function
   */
  double costFunction(const vector<double> &vars, vector<double> &grad, void *data);

  Eigen::Matrix<double, 3, 1> pid; ///< PID gains
  double timeShift; ///< Sensor measurements delay
  Eigen::Matrix<double, Eigen::Dynamic, 1> modelledPositions; ///< Modeled positions
  Eigen::Matrix<double, Eigen::Dynamic, 1> sensedPosition; ///< Input joint position
  Eigen::Matrix<double, Eigen::Dynamic, 1> cmdPosition; ///< Commanded joint positions
  Eigen::Matrix<double, Eigen::Dynamic, 1> time; ///< Time vector
  boost::shared_ptr<JointEstimator<double> > estimator;
  fstream measFile; ///< File containing the measurements

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
