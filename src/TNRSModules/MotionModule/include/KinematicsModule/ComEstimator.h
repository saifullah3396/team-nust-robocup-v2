/**
 * @file MotionModule/include/KinematicsModule/ComEstimator.h
 *
 * This file declares the class ComEstimator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include "Utils/include/DataUtils.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/Filters/KalmanFilter.h"
#include "Utils/include/Constants.h"

/**
 * @class ComEstimator
 * @brief A class that provides a definition for center of mass state
 *   estimator
 */
template <typename Scalar, size_t MeasSize>
class ComEstimator
{
public:
  /**
   * @brief ComEstimator Constructor
   */
  ComEstimator() = default;

  /**
   * @brief ~ComEstimator Destructor
   */
  ~ComEstimator() {}

  /**
   * @brief init Sets up the filter and initiates it with the given state vector
   *
   * @param initState the initial state for the filter
   * @param comHeight com height for the inverted-cart table model
   * @param dT time step for the filter
   */
  void init(
    const Matrix<Scalar, 3, 1>& initState,
    const Scalar& comHeight,
    const Scalar& dT);

  /**
   * @brief reset Resets the filter with the given state vector
   *
   * @param state the reset state for the filter
   */
  void reset(const Matrix<Scalar, 3, 1>& state)
    { model->setState(state); filter->reset(); }

  /**
   * @brief update Updates the filter and estimator for the given
   *   measurement input
   *
   * @param meas The new measurement
   */
  void update(const Matrix<Scalar, MeasSize, 1>& meas);

  /**
   * @brief setState Sets the model state
   * @param state Center of mass state
   */
  void setState(const Matrix<Scalar, 3, 1>& state) {
    model->setState(state);
  }

  /**
   * @brief getState Returns the current system state
   */
  Matrix<Scalar, 3, 1> getState() { return model->getState(); }

  /**
   * @brief getOutput Returns the current system output
   */
  Matrix<Scalar, 1, 1> getOutput() { return model->getOutput(); }

  /**
   * @brief getModel Returns the process model used
   */
  boost::shared_ptr<ProcessModel<Scalar, 3, 1, 1> > getModel() { return model; }

  /**
   * @brief updateComHeight Updates the model for the given constant com height
   * @param comHeight Height of the center of mass
   */
  void updateComHeight(const Scalar& comHeight) {
    C(0, 2) = -comHeight / Constants::gravity;
    model->setOutputMatrix(C);
    H(2, 2) = -comHeight / Constants::gravity;
    filter->setMeasMatrix(H);
  }

  /**
   * Returns true if the filter is already initiated
   */
  bool isInitiated() { return initiated; }

private:
  ///< Kalman filter
  boost::shared_ptr<KalmanFilter<Scalar, 3, 3, 1, 1> > filter;

  ///< Com state transition model
  boost::shared_ptr<ProcessModel<Scalar, 3, 1, 1> > model;

  ///< Com state output matrix
  Matrix<Scalar, 1, 3> C =
   {Matrix<Scalar, 1, 3>::Zero()};

  ///< Com measurement matrix
  Matrix<Scalar, MeasSize, MeasSize> H =
    {Matrix<Scalar, MeasSize, MeasSize>::Zero()};

  ///< Whether the filter has been initiated or not
  bool initiated;

  ///< Measurement noise covariance matrix which can be varied through subsequent cycles
  Matrix<Scalar, MeasSize, MeasSize> R =
    {Matrix<Scalar, MeasSize, MeasSize>::Zero()};

  ///< Last measurement vector
  Matrix<Scalar, MeasSize, 1> lastMeas =
    {Matrix<Scalar, MeasSize, 1>::Zero()};

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
