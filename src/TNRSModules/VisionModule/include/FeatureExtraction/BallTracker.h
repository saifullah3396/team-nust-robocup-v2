/**
 * @file VisionModule/include/FeatureExtraction/BallTracker.h
 *
 * This file declares the class BallTracker
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include <opencv2/video/tracking.hpp>
#include "Utils/include/DataUtils.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/VisionUtils.h"
#include "VisionModule/include/CameraTransform.h"
#include "VisionModule/include/VisionModule.h"

//!@todo: Replace Opencv kalman filter with our own kalman filter in BallTracker
/**
 * @class BallTracker
 * @brief A class that provides a definition of a ball tracker based on
 *   kalman filter.
 */
class BallTracker : public MemoryBase
{
public:
  /**
   * @brief BallTracker Constructor
   * @param visionModule Pointer to base vision module
   */
  BallTracker(VisionModule* visionModule) :
    MemoryBase(visionModule), visionModule(visionModule),
    cameraTransforms(visionModule->getCameraTransforms()),
    cycleTime(visionModule->getPeriodMinMS() / ((float) 1000))
  {
  }

  /**
   * @brief BallTracker Destructor
   */
  ~BallTracker() {}

  void init(const CameraId& camIndex);
  void reset(const CameraId& camIndex);
  void reset(const CameraId& camIndex, const Mat& state);
  Mat predict();
  Mat getEstimatedState();
  void updateFilter(vector<Rect> ballRect);
  void setupKalmanFilter();
  void getBallPreview(
    vector<Mat>& ballPreview, const unsigned& n, const float& dt);
  bool getBallFound() const
    { return !lost; }
  CameraId getCamIndex() const
   { return camIndex; }
  float getTimeSinceLost() const
    { return timeSinceLost; }
  void setCamIndex(const CameraId& camIndex)
    { this->camIndex = camIndex; }

private:
  ///< OpenCv based kalman filter object.
  cv::KalmanFilter kFilter;

  ///< Vector of state variables.
  Mat state;

  ///< Vector of motion state variables.
  ///< [posX, posY, velocityX, velocityY, accX, accY]
  Mat motionState;

  ///< State transition matrix for motion update for trajectory
  ///< extrapolation
  Mat motionStateTransition;

  ///< Vector of measurement variables.
  Mat meas;

  ///< A matrix defining the actual noise covariance matrix for measurements
  Mat measNoiseCov;

  ///< A matrix defining noise covariance matrix for when no measurement input is recieved
  Mat infMeasNoiseCov;

  ///< Number of state variables:
  ///< [posX, posY, velocityX, velocityY, accX, accY,
  ///<  posImageX, posImageY, velocityImageX, velocityImageY,
  ///<  wImage, hImage]
  int stateSize = {12};

  ///< Number of measurement variables:
  ///< [measPosX, measPosY, measPosImageX, measPosImageY,
  ///<  measWImage, measHImage]
  int measSize = {6};

  ///< Number of motion state variables
  int motionStateSize = {6};

  ///< Number of variables in the contr state vector.
  int contrSize = {0};

  ///< Time at which the ball was lost.
  float timeAtLost = {-1.f};

  ///< Time since the ball was lost.
  float timeSinceLost = {-1.f};

  ///< Whether the filter is to be reinitialized.
  bool reinitialize = {true};

  ///< Whether the ball is lost.
  bool lost = {true};

  ///< Whether we are currently getting no ball observation.
  bool losingBall = {false};

  ///< Time step for the kalman filter updates.
  float cycleTime;

  ///< Image transform object.
  vector<boost::shared_ptr<CameraTransform> > cameraTransforms;

  ///< Cam index for ball tracker information.
  CameraId camIndex;

  ///< Vision module pointer object.
  VisionModule* visionModule;
};
