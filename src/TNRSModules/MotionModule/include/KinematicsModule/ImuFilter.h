/**
 * @file MotionModule/include/KinematicsModule/ImuFilter.h
 *
 * This file declares the class ImuFilter
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include <Eigen/Dense>
#include "Utils/include/AngleDefinitions.h"

/**
 * @class ImuFilter
 * @brief A class that provides a definition of madgewick filter
 *   for the inertial measurement unit (IMU) of the robot
 */
template <typename Scalar>
class ImuFilter
{
public:
  /**
   * @brief ImuFilter Constructor
   * @param cycleTime Update time period
   */
  ImuFilter(const Scalar& cycleTime);
  /**
   * @brief ~ImuFilter Destructor
   */
  ~ImuFilter() {}

  /**
   * @brief initiate Initiates the filter
   */
  void initiate();

  /**
   * @brief update Updates the filter with new measurement
   * @param measAcc Measured acceleration
   * @param measGyr Measured gyro angular velocities
   */
  void update(
    Eigen::Matrix<Scalar, 3, 1> measAcc,
    const Eigen::Matrix<Scalar, 3, 1>& measGyr);

  //! Getters
  Eigen::Matrix<Scalar, 4, 1> getQuaternion();
  Eigen::Matrix<Scalar, 3, 3> getRotation();
private:

  Eigen::Matrix<Scalar, 4, 1> q;
  Scalar beta;
  Scalar gyrMeasError = {Angle::DEG_1};
  Scalar cycleTime;
  size_t maxIterations = {10};

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
