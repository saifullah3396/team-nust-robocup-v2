/**
 * @file MotionModule/include/KinematicsModule/ObstacleTracker.h
 *
 * This file declares the class ObstacleTracker
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 April 2019
 */

#include "Utils/include/Filters/KalmanFilter.h"
#include "Utils/include/DataHolders/ObstacleType.h"

#define STATE_SIZE 4
#define MEAS_SIZE 2

/**
 * @class ObstacleTracker
 * @brief A kalman filter based robot position tracker
 */
class ObstacleTracker
{
public:
  /**
   * @brief ObstacleTracker Constructor
   */
  ObstacleTracker() = default;
  ObstacleTracker(const ObstacleTracker&) = default;
  ObstacleTracker(ObstacleTracker&&) = default;
  ObstacleTracker& operator=(const ObstacleTracker&) & = default;
  ObstacleTracker& operator=(ObstacleTracker&&) & = default;

  /**
   * @brief ObstacleTracker Constructor
   * @param obstacleType Type of robot obstacle tracked
   * @param radius Obstacle radius
   */
  ObstacleTracker(const ObstacleType& obstacleType, const float& radius) :
    obstacleType(obstacleType), radius(radius)
  {
  }

  /**
   * @brief ~ObstacleTracker Destructor
   */
  virtual ~ObstacleTracker() {}

  /**
   * @brief init Sets up the filter and initiates it with the given state vector
   *
   * @param initState Initial state of the joint
   * @param dT Update cycle time
   * @param time Time at initiation
   */
  void init(
    const Matrix<float, STATE_SIZE, 1>& initState,
    const float& dt,
    const float& time);

  /**
   * @brief reset Resets the filter with the given state vector
   *
   * @param state the reset state for the filter
   */
  void reset(const Matrix<float, STATE_SIZE, 1>& state);

  /**
   * @brief setState Sets the state of the joint
   * @param state Input state
   */
  void setState(const Matrix<float, STATE_SIZE, 1>& state) {
    model->setState(state);
  }

  /**
   * @brief update Updates the filter and estimator for the given
   *   measurement input
   *
   * @param meas The new measurement
   */
  void update(const Matrix<float, MEAS_SIZE, 1>& meas, const float& time);

  /**
   * @brief updateInput Updates the input for the underlying model and performs
   *   prediction step
   */
  void updateModel();

  /**
   * @brief getState Returns the current system state
   */
  const Matrix<float, STATE_SIZE, 1>& getState() { return model->getState(); }

  /**
   * @brief obstacleType Returns the type of obstacle
   */
  ObstacleType& getObstacleType() { return obstacleType; }

  /**
   * @brief getModel Returns the process model used
   */
  boost::shared_ptr<ProcessModel<float, STATE_SIZE, 1, 1> > getModel() { return model; }

  /**
   * @brief getLastUpdateTime Time of last update
   * @return float
   */
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
   * @brief getRadius Returns the obstacle radius
   * @return float
   */
  const float& getRadius() { return radius; }

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

  ///< Kalman filter (Conflicts with opencv KalmanFilter in namespace)
  boost::shared_ptr<::KalmanFilter<float, STATE_SIZE, MEAS_SIZE, 1, 1> > filter;

  ///< Joint state transition model
  boost::shared_ptr<ProcessModel<float, STATE_SIZE, 1, 1> > model;

  ///< Whether the filter has been initiated or not
  bool initiated;

  ///< Time step
  static float dt;

  ///< State transition and noise matrices
  static Matrix<float, STATE_SIZE, STATE_SIZE> A, Q;

  ///< Measurment noise covariance
  static Matrix<float, MEAS_SIZE, MEAS_SIZE> R;

  ///< Input transition matrix
  static Matrix<float, STATE_SIZE, 1> B;

  ///< Measure Matrix H
  static Matrix<float, MEAS_SIZE, STATE_SIZE> H;

  ///< Time at which it was last updated
  float timeUpdated;

  ///< Obstacle radius
  float radius;

  ///< Robot obstacle type
  ObstacleType obstacleType;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
