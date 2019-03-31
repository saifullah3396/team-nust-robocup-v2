/**
 * @file MotionModule/include/KinematicsModule/JointEstimator.h
 *
 * This file declares the class JointEstimator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include "Utils/include/DataUtils.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/Filters/KalmanFilter.h"
#include "Utils/include/Constants.h"
#include "Utils/include/HardwareIds.h"

/**
 * @class JointEstimator
 * @brief A class that provides a definition for joint estimation
 */
template <typename Scalar>
class JointEstimator
{
public:
  /**
   * @brief JointEstimator Constructor
   */
  JointEstimator() = default;
  JointEstimator(const JointEstimator&) = default;
  JointEstimator(JointEstimator&&) = default;
  JointEstimator& operator=(const JointEstimator&) & = default;
  JointEstimator& operator=(JointEstimator&&) & = default;

  /**
   * @brief ~JointEstimator Destructor
   */
  virtual ~JointEstimator() {}

  /**
   * @brief init Sets up the filter and initiates it with the given state vector
   *
   * @param jointIndex: Index of the joint
   * @param initState Initial state of the joint
   * @param dT: Update cycle time
   */
  void init(
    const Joints& jointIndex,
    const Matrix<Scalar, 2, 1>& initState,
    const Scalar& dt);

  /**
   * @brief reset Resets the filter with the given state vector
   *
   * @param state the reset state for the filter
   */
  void reset(const Matrix<Scalar, 2, 1>& state);

  /**
   * @brief setState Sets the state of the joint
   * @param state Input state
   */
  void setState(const Matrix<Scalar, 2, 1>& state) {
    model->setState(state);
  }

  /**
   * @brief setState Sets the state of the joint
   * @param state Input state
   */
  void setState(const Scalar& state, const unsigned& index) {
    model->setState(state, index);
  }

  /**
   * @brief setControl Sets the control input vector as a p-controller
   * @param error Error between the desired and actual state
   */
  void setControl(const Scalar& cmd);

  /**
   * @brief setInput sets the control input vector
   * @param input control input
   */
  void setInput(const Scalar& input)
  {
    model->setInput(input);
  }

  /**
   * @brief setPidGains Sets the pid gains for virtual controller
   * @param pid Gains
   */
  void setPidGains(const Matrix<Scalar, 3, 1>& pid);

  /**
   * @brief update Updates the filter and estimator for the given
   *   measurement input
   *
   * @param meas The new measurement
   */
  void update(const Scalar& meas);

  /**
   * @brief updateInput Updates the input for the underlying model and performs
   *   prediction step
   */
  void updateModel();

  /**
   * @brief getState Returns the current system state
   */
  const Matrix<Scalar, 2, 1>& getState() { return model->getState(); }

  /**
   * @brief getInput Returns the current system input
   */
  const Matrix<Scalar, 1, 1>& getInput() { return model->getInput(); }

  /**
   * @brief getCmd Gets the latest command sent to joint
   * @return Scalar
   */
  const Scalar& getCmd() { return lastCmd; }

  /**
   * @brief getModel Returns the process model used
   */
  boost::shared_ptr<ProcessModel<Scalar, 2, 1, 1> > getModel() { return model; }

  /**
   * Returns true if the filter is already initiated
   */
  bool isInitiated() { return initiated; }

private:
  //! Kalman filter
  boost::shared_ptr<KalmanFilter<Scalar, 2, 1, 1, 1> > filter;

  //! Joint state transition model
  boost::shared_ptr<ProcessModel<Scalar, 2, 1, 1> > model;

  //! Measurment noise covariance
  Matrix<Scalar, 1, 1> R;

  //! Last measurement received
  Scalar lastMeas;

  //! Whether the filter has been initiated or not
  bool initiated;

  //! PID-controller constants
  Matrix<Scalar, 3, 1> pidC;

  //! Last error
  Scalar prevError1 = {0};

  Scalar prevError2 = {0};

  //! Last input
  Scalar prevInput = {0};

  //! Last command
  Scalar lastCmd = {0};

  //! Time step
  Scalar dt = {0.01};

  //! Whether to use generate control input for this joint
  bool useInput = {false};

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
