/**
 * @file Utils/include/Filters/KalmanFilter.h
 *
 * This file declares the class KalmanFilter
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017
 */

#pragma once

#include "Utils/include/MathsUtils.h"
#include "Utils/include/Filters/ProcessModel.h"
#include "Utils/include/DebugUtils.h"

/**
 * @class KalmanFilter
 * @brief Defines the base class for defining any kalman filter variant
 */
template <typename Scalar, size_t StateSize, size_t MeasSize, size_t InputSize, size_t OutputSize>
class KalmanFilter
{
public:
  KalmanFilter() = default;
  KalmanFilter(const KalmanFilter&) = default;
  KalmanFilter(KalmanFilter&&) = default;
  KalmanFilter& operator=(const KalmanFilter&) & = default;
  KalmanFilter& operator=(KalmanFilter&&) & = default;

  /**
   * @brief KalmanFilter Initializes the filter
   * @param model process model
   * @param measSize measurement size
   */
  KalmanFilter(
    const boost::shared_ptr<ProcessModel<Scalar, StateSize, InputSize, OutputSize> >& model) :
    model(model)
  {
    this->P.setZero();
    R.setZero();
  }

  /**
   * @brief KalmanFilter Initializes the filter
   * @param model process model
   * @param measSize measurement size
   * @param H measurement matrix
   */
  KalmanFilter(
    const boost::shared_ptr<ProcessModel<Scalar, StateSize, InputSize, OutputSize> >& model,
    const Matrix<Scalar, MeasSize, StateSize>& H) :
    model(model),
    H(H)
  {
    this->Ht = this->H.transpose();
    this->P.setZero();
    R.setZero();
  }

  /**
   * @brief ~KalmanFilter Destructor
   */
  virtual ~KalmanFilter() {}

  /**
   * @brief predict Performs the filter prediction step
   */
  void predict() {
    if (model) {
      model->update();
      P = model->computeErrorCov(P);
      model->setUpdated(false);
    }
  }

  /**
   * @brief correct Performs the filter correction step
   * @param meas Input measurement
   */
  void correct(const Matrix<Scalar, MeasSize, 1>& meas) {
    if (model) {
      Matrix<Scalar, StateSize, 1> state = model->getState();
      Matrix<Scalar, StateSize, MeasSize> K = P * Ht * (R + H * P * Ht).inverse();
      model->setState(state + K * (meas - H * state));
      P = P - K * H * P;
    }
  }

  /**
   * @brief reset Resets the filter
   */
  void reset() { this->P.setIdentity(); }

  void setModel(const boost::shared_ptr<ProcessModel<Scalar, StateSize, InputSize, OutputSize> >& model) {
    this->model = model;
  }

  /**
   * @brief setMeasMatrix sets the measurement matrix
   * @param H measurement matrix
   */
  void setMeasMatrix(const Matrix<Scalar, MeasSize, StateSize>& H)
  {
    this->H = H;
    this->Ht = H.transpose();
  }

  /**
   * @brief setMeasNoiseCov sets the measurement noise covariance matrix
   * @param R measurement noise covariance matrix
   */
  void setMeasNoiseCov(const Matrix<Scalar, MeasSize, MeasSize>& R)
  {
    this->R = R;
  }

  /**
   * @brief setMeasNoiseCov sets the measurement noise covariance at the specified index
   * @param r measurement noise covariance
   */
  void setMeasNoiseCov(
    const Scalar& r,
    const unsigned i,
    const unsigned j)
  {
    ASSERT(i < MeasSize && j < MeasSize)
    this->R(i, j) = r;
  }

  /**
   * @brief setErrorNoiseCov sets the state error noise covariance matrix
   * @param P measurement noise covariance matrix
   */
  void setErrorNoiseCov(const Matrix<Scalar, StateSize, StateSize>& P)
  {
    this->P = P;
  }

private:
  //! Process model for the system update
  boost::shared_ptr<ProcessModel<Scalar, StateSize, InputSize, OutputSize> > model;

  //! Measurement matrix
  Matrix<Scalar, MeasSize, StateSize> H = {Matrix<Scalar, MeasSize, StateSize>::Zero()};

  //! Measurement matrix transpose
  Matrix<Scalar, StateSize, MeasSize> Ht = {Matrix<Scalar, StateSize, MeasSize>::Zero()};

  //! Measurement noise covariance matrix
  Matrix<Scalar, MeasSize, MeasSize> R = {Matrix<Scalar, MeasSize, MeasSize>::Zero()};

  //! State estimate error covariance
  Matrix<Scalar, StateSize, StateSize> P = {Matrix<Scalar, StateSize, StateSize>::Zero()};
};
