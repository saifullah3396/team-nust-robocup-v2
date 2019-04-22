/**
 * @file MotionModule/include/KinematicsModule/RobotTracker.h
 *
 * This file declares the class RobotTracker
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include "Utils/include/Filters/KalmanFilter.h"
#include "Utils/include/DataHolders/ObstacleType.h"

/**
 * @class RobotTracker
 * @brief A kalman filter based robot position tracker
 */
class RobotTracker
{
public:
  /**
   * @brief RobotTracker Constructor
   */
  RobotTracker() = default;
  RobotTracker(const RobotTracker&) = default;
  RobotTracker(RobotTracker&&) = default;
  RobotTracker& operator=(const RobotTracker&) & = default;
  RobotTracker& operator=(RobotTracker&&) & = default;

  /**
   * @brief RobotTracker Constructor
   * @param obstacleType Type of robot obstacle tracked
   */
  RobotTracker(const ObstacleType& obstacleType) :
    obstacleType(obstacleType)
  {
  }

  /**
   * @brief ~RobotTracker Destructor
   */
  virtual ~RobotTracker() {}

  /**
   * @brief init Sets up the filter and initiates it with the given state vector
   *
   * @param initState Initial state of the joint
   * @param dT Update cycle time
   * @param time Time at initiation
   */
  void init(
    const Matrix<float, 12, 1>& initState,
    const float& dt,
    const float& time);

  /**
   * @brief reset Resets the filter with the given state vector
   *
   * @param state the reset state for the filter
   */
  void reset(const Matrix<float, 12, 1>& state);

  /**
   * @brief setState Sets the state of the joint
   * @param state Input state
   */
  void setState(const Matrix<float, 12, 1>& state) {
    model->setState(state);
  }

  /**
   * @brief update Updates the filter and estimator for the given
   *   measurement input
   *
   * @param meas The new measurement
   */
  void update(const Matrix<float, 10, 1>& meas, const float& time);

  /**
   * @brief updateInput Updates the input for the underlying model and performs
   *   prediction step
   */
  void updateModel();

  /**
   * @brief getState Returns the current system state
   */
  const Matrix<float, 12, 1>& getState() { return model->getState(); }

  /**
   * @brief obstacleType Returns the type of obstacle
   */
  ObstacleType& getObstacleType() { return obstacleType; }

  /**
   * @brief getModel Returns the process model used
   */
  boost::shared_ptr<ProcessModel<float, 12, 1, 1> > getModel() { return model; }

  const float& getLastUpdateTime() { return timeUpdated; }

  /**
   * @brief getDist Returns the distance from the tracked robot
   * @return Distance
   */
  float getDist() {
    const auto& state = model->getState();
    return sqrt(state[0] * state[0] + state[1] * state[1]);
  }

  /**
   * Returns true if the filter is already initiated
   */
  bool isInitiated() { return initiated; }

private:
  /**
   * @brief loadModel Loads the model parameters
   * @param dt Update cycle time
   */
  void loadModel(const float& dt);

  ///< Kalman filter
  boost::shared_ptr<::KalmanFilter<float, 12, 10, 1, 1> > filter;

  ///< Joint state transition model
  boost::shared_ptr<ProcessModel<float, 12, 1, 1> > model;

  ///< Whether the filter has been initiated or not
  bool initiated;

  ///< Time step
  static float dt;

  ///< State transition and noise matrices
  static Matrix<float, 12, 12> A, Q;

  ///< Measurment noise covariance
  static Matrix<float, 10, 10> R;

  ///< Input transition matrix
  static Matrix<float, 12, 1> B;

  ///< Measure Matrix H
  static Matrix<float, 10, 12> H;

  ///< Time at which it was last updated
  float timeUpdated;

  ///< Robot obstacle type
  ObstacleType obstacleType;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
