/**
 * @file Utils/include/Solvers/ImuCalibrator.h
 *
 * This file declares the class ImuCalibrator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#pragma once
#include <fstream>
#include <Eigen/Dense>
#include "Utils/include/Solvers/NLOptimizer.h"

using namespace std;

/**
 * @class ImuCalibrator
 * @brief A class to calibrate the IMU data based on input measurements
 */
template <int StateSize = 3>
class ImuCalibrator : public NLOptimizer
{
public:
  /**
   * @brief ImuCalibrator Constructor
   * @param measFilePath Path to file containing the imu measurements
   */
  ImuCalibrator(const string& measFilePath);

  /**
   * @brief ~ImuCalibrator Destructor
   */
  ~ImuCalibrator() {}

  /**
   * @brief optDef Solves the optimization problem that minimizes the norm
   *   X = |R * scale * (inputAcc - bias) - gravity|
   *   for all input measurements.
   */
  virtual void optDef() override;

  /**
   * @brief printResults Prints the calibration results
   */
  void printResults();

  Eigen::Matrix<double, StateSize, StateSize> getScaleMatrix() { return scale; }
  Eigen::Matrix<double, StateSize, 1> getBiasMatrix() { return bias; }

protected:
  /**
   * Evaluates the objective function
   */
  double costFunction(const vector<double> &vars, vector<double> &grad, void *data);

  Eigen::Matrix<double, StateSize, StateSize> scale; ///< Acceleration scale matrix
  Eigen::Matrix<double, StateSize, 1> bias; ///< Acceleration bias matrix
  Eigen::Matrix<double, Eigen::Dynamic, StateSize> inputAcc; ///< Input acceleration observed measurements
  Eigen::Matrix<double, Eigen::Dynamic, 2> inputAngles; ///< Input angles observed measurements
  fstream measFile; ///< File containing the measurements

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
