/**
 * @file Utils/include/PIDController.h
 *
 * This file defines the struct PIDController
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 April 2018
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

/**
 * @struct PIDController
 * @brief Defines a simple discrete time PID controller
 */
template<typename Scalar>
struct PIDController
{
  PIDController() = default;
  PIDController(const PIDController&) = default;
  PIDController(PIDController&&) = default;
  PIDController& operator=(const PIDController&) & = default;
  PIDController& operator=(PIDController&&) & = default;
  virtual ~PIDController() {}

  /**
   * @brief PIDController Constructor
   * @param dt Update cycle time
   */
  PIDController(const Scalar& dt) : _dt(dt) {}

  /**
   * @brief update Performs the controller update tep
   * @param meas Current measurement of controller target
   * @return Returns the controller input
   */
  Scalar update(const Scalar& meas) {
    auto error = (_cmd - meas);
    auto input = _prevInput + pidC[0] * error + pidC[1] * _prevError1 + pidC[2] * _prevError2;
    _prevError2 = _prevError1;
    _prevError1 = error;
    _prevInput = input;
    return input;
  }

  /**
   * @brief setPidGains
   * @param pid
   */
  void setPidGains(const Matrix<Scalar, 3, 1>& pid) {
    this->pid = pid;
    pidC[0] = pid[0] + pid[1] * _dt / 2 + pid[2] / _dt;
    pidC[1] = -pid[0] + pid[1] * _dt / 2 - 2 * pid[2] / _dt;
    pidC[2] = pid[2] / _dt;
  }

  ///< Setters
  void setCmd(const Scalar& cmd) { this->_cmd = cmd; }
  void setDt(const Scalar& dt) { this->_dt = dt; }

  const Scalar& prevError1() { return _prevError1; }
  const Scalar& prevError2() { return _prevError2; }
  const Scalar& prevInput() { return _prevInput; }
private:
  ///< Controller gains proportional, integral, derivative
  Eigen::Matrix<Scalar, 3, 1> pid;

  ///< Constants resulting from pid gains
  Eigen::Matrix<Scalar, 3, 1> pidC;

  Scalar _prevError1 = {0}; ///< Error in cycle k-1
  Scalar _prevError2 = {0}; ///< Error in cycle k-2
  Scalar _prevInput = {0}; ///< Last input
  Scalar _cmd = {0}; ///< Last command
  Scalar _dt = {0.01}; ///< Time step
};
