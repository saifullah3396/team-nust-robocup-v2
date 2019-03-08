/**
 * @file MotionModule/include/KinematicsModule/ImuStateEstimator.h
 *
 * This file declares the class ImuStateEstimator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include "Utils/include/DataUtils.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/Filters/KalmanFilter.h"
#include "Utils/include/Constants.h"

/**
 * @class ImuStateEstimator
 * @brief A class that provides a definition for imu state estimator
 */
template <typename Scalar>
class ImuStateEstimator
{
public:
  /**
   * @brief ImuStateEstimator Constructor
   */
  ImuStateEstimator() :
    stateSize(16), measSize(3), inputSize(3), initiated(false)
  {
  }

  /**
   * @brief ~ImuStateEstimator Destructor
   */
  ~ImuStateEstimator()
  {
  }

  /**
   * @brief init Sets up the filter and initiates it with the given state vector
   *
   * @param initState the initial state for the filter
   * @param comHeight com height for the inverted-cart table model
   * @param dT time step for the filter
   */
  void init(
    const Matrix<Scalar, Dynamic, 1>& initState,
    const Scalar& comHeight,
    const Scalar& dT);

  /**
   * @brief reset Resets the filter with the given state vector
   *
   * @param state the reset state for the filter
   */
  void reset(const Matrix<Scalar, Dynamic, 1>& state) { model->setState(state); filter->reset(); }

  /**
   * @brief update Updates the filter and estimator for the given
   *   measurement input
   *
   * @param meas The new measurement
   */
  void update(const Matrix<Scalar, Dynamic, 1>& meas);

  /**
   * @brief getState Returns the current system state
   */
  Matrix<Scalar, Dynamic, 1> getState() { return model->getState(); }

  /**
   * @brief getOutput Returns the current system output
   */
  Matrix<Scalar, Dynamic, 1> getOutput() { return model->getOutput(); }

  /**
   * @brief getModel Returns the process model used
   */
  boost::shared_ptr<ProcessModel<Scalar> > getModel() { return model; }

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
  //! Kalman filter
  boost::shared_ptr<KalmanFilter<Scalar> > filter;

  //! Com state transition model
  boost::shared_ptr<ProcessModel<Scalar> > model;

  //! Com state output matrix
  Matrix<Scalar, Dynamic, Dynamic> C;

  //! Com measurement matrix
  Matrix<Scalar, Dynamic, Dynamic> H;

  //! State size
  unsigned stateSize;

  //! Measurement size
  unsigned measSize;

  //! Input size
  unsigned inputSize;

  //! Whether the filter has been initiated or not
  bool initiated;

  //! Measurement noise covariance matrix which can be varied through subsequent cycles
  Matrix<Scalar, Dynamic, Dynamic> R;

  //! Last measurement vector
  Matrix<Scalar, Dynamic, 1> lastMeas;
};


/**
 * @brief This function sets the initial parameters of the filter.
 * @return void
 */
void
initiate();

  /**
   * @brief This function performs the main filter update step.
   * @return void
   */
  void
  update();

  /**
   * @brief This function performs the prediction step of the kalman
   *   filter.
   * @return void
   */
  void
  predict();

  /**
   * @brief This function solve the process model of the system.
   * @param xdot: State derivative.
   * @param F: F-matrix.
   * @param G: G-matrix.
   * @return void
   */
  void
  process(Matrix<T, Dynamic, 1>& xdot, Matrix<T, Dynamic, Dynamic>& F,
    Matrix<T, Dynamic, Dynamic>& G);

  /**
   * @brief This function solve the measurement model of the accelerometer
   *   using only the gravity term.
   * @param H: H-matrix.
   * @return void
   */
  void
  measurement(Matrix<T, 3, 1>& acc, Matrix<T, Dynamic, Dynamic>& H);

  /**
   * @brief This function performs the correction step of the ekf.
   * @param z: Actual measurements.
   * @param zhat: Predicted measurements.
   * @param H: H-matrix.
   * @param R: R-matrix.
   * @return void
   */
  void
  correction(Matrix<T, Dynamic, 1> z, Matrix<T, Dynamic, 1> zhat,
    Matrix<T, Dynamic, Dynamic> H, Matrix<T, Dynamic, Dynamic> R);

  /**
   * @brief This function performs the correction step for the acceleration
   *   measurements.
   * @return void
   */
  void
  correctAccMeasurement();

  /**
   * @brief This function copies the state elements from the state vector
   * @param q: The orientation of the imu in the form of quaternions.
   * @param pos: Position.
   * @param vel: Velocity.
   * @param omegaBias: Angular velocity bias.
   * @param accBias: Acceleration bias.
   * @return void
   */
  void
  getState(Quaternion<T>& q, Matrix<T, 3, 1>& pos, Matrix<T, 3, 1>& vel,
    Matrix<T, 3, 1>& omegaBias, Matrix<T, 3, 1>& accBias);

private:
  //!Number of elements in the state vector.
  const size_t nState = 16;

  //!System state vector.
  Matrix<T, Dynamic, 1> x;

  //!State covariance matrix.
  Matrix<T, Dynamic, Dynamic> P;

  //!State covariance matrix.
  const Matrix<T, 3, 1> gravVec = Matrix<T, 3, 1>(0, 0, Constants::gravity);

  //!Gyrometer covariance.
  const T gyroCov = 0.01;

  //!Accelerometer covariance.
  const T accCov = 0.1;

  //!Gravity covariance.
  const T gravCov = 0.1;

  //!Observation noise.
  Matrix<T, Dynamic, Dynamic> Q;

  //!Measurement noise.
  const Matrix<T, 3, 3> gravR = Matrix<T, 3, 3>::Identity() * gravCov;

  //!Acceleraton input in current step.
  Matrix<T, 3, 1> acc;

  //!Gyro input in current step.
  Matrix<T, 3, 1> gyro;

  //!The variable that checks whether the filter has been initiated.
  bool initiated;

  //!Time step after initiation.
  float timeStep;

  //!Cycle time.
  const float cycleTime = 0.01;
};

