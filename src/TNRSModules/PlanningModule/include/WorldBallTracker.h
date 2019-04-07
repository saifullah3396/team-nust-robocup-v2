/**
 * @file PlanningModule/include/WorldBallTracker.h
 *
 * This file defines the class WorldBallTracker
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 May 2018
 */

#include "PlanningModule/include/PlanningModule.h"
#include "Utils/include/DataUtils.h"
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;

/**
 * @class WorldBallTracker
 * @brief A class that uses kalman filter to track the position of the ball
 *   in the world based on observations of all team members
 */
class WorldBallTracker : public MemoryBase
{
public:
  /**
   * Default constructor for this class.
   * 
   * @param planningModule: Pointer to parent PlanningModule.
   */
  WorldBallTracker(PlanningModule* planningModule) :
    MemoryBase(planningModule), planningModule(planningModule),
      cycleTime(planningModule->getPeriodMinMS() / ((float) 1000)), stateSize(4),
      measSize(2), contrSize(0), timeSinceLost(-1.f), timeAtLost(-1.f),
      losingBall(false)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~WorldBallTracker()
  {
  }

  void
  init()
  {
    lost = true;
    setupKalmanFilter();
  }

  Mat
  predict()
  {
    return kFilter.predict();
  }

  Mat
  getEstimatedState()
  {
    if (losingBall || lost) return kFilter.statePre;
    else return kFilter.statePost;
  }

  void
  updateFilter(vector<float> measurements)
  {
    if (measurements.empty()) {
      if (!losingBall) {
        timeSinceLost = 0.f;
        timeAtLost = planningModule->getModuleTime();
        losingBall = true;
      }
    } else {
      meas.at<float>(0) = measurements[0];
      meas.at<float>(1) = measurements[1];
      if (lost) { ///< First detection!
        ///< >>>> Initialization
        kFilter.errorCovPre.at<float>(0) = 1.0f;
        kFilter.errorCovPre.at<float>(5) = 1.0f;
        kFilter.errorCovPre.at<float>(10) = 1.0f;
        kFilter.errorCovPre.at<float>(15) = 1.0f;

        state.at<float>(0) = meas.at<float>(0);
        state.at<float>(1) = meas.at<float>(1);
        state.at<float>(2) = 0.f;
        state.at<float>(3) = 0.f;
        ///< <<<< Initialization
        kFilter.statePost = state;
        lost = false;
        losingBall = false;
        timeSinceLost = 0.f;
      } else {
        kFilter.correct(meas); ///< Kalman Correction
      }
    }
    if (losingBall) timeSinceLost = planningModule->getModuleTime() - timeAtLost;
    if (timeSinceLost >= 0.25f) lost = true;
  }

  void
  setupKalmanFilter()
  {
    unsigned int type = CV_32F;
    kFilter = KalmanFilter(stateSize, measSize, contrSize, type);
    state = Mat(stateSize, 1, type);
    meas = Mat(measSize, 1, type);
    ///< Transition State Matrix A
    ///< [ 1  0    dT   0]
    ///< [ 0  1    0   dT]
    ///< [ 0  0    1    0] 
    ///< [ 0  0    0    1]
    setIdentity(kFilter.transitionMatrix);
    kFilter.transitionMatrix.at<float>(2) = cycleTime;
    kFilter.transitionMatrix.at<float>(7) = cycleTime;
    ///< Measure Matrix H
    ///< [ 1 0 0 0 ]
    ///< [ 0 1 0 0 ]
    kFilter.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kFilter.measurementMatrix.at<float>(0) = 1.0f;
    kFilter.measurementMatrix.at<float>(5) = 1.0f;
    ///< Process Noise Covariance Matrix Q
    ///< [ eX   0    0     0   ]
    ///< [ 0    eY   0     0   ]
    ///< [ 0    0    eVX   0   ]
    ///< [ 0    0    0     eVY ]

    kFilter.processNoiseCov.at<float>(0) = 0.001;
    kFilter.processNoiseCov.at<float>(5) = 0.001;
    kFilter.processNoiseCov.at<float>(10) = 0.001;
    kFilter.processNoiseCov.at<float>(15) = 0.001;

    setIdentity(kFilter.measurementNoiseCov);
    ///< Measurement Noise Covariance Matrix Q
    ///< [ eX   0  ]
    ///< [ 0    eY ]
    kFilter.measurementNoiseCov.at<float>(0) = 0.01; // Robot movement noise + measurement noise
    kFilter.measurementNoiseCov.at<float>(3) = 0.01; // Robot movement noise + measurement noise
  }

  bool
  getBallFound() const
  {
    return !lost;
  }

  float
  getTimeSinceLost() const
  {
    return timeSinceLost;
  }
private:
  ///< OpenCv based kalman filter object.
  KalmanFilter kFilter;

  ///< Vector of state variables.
  Mat state;

  ///< Vector of measurement variables.
  Mat meas;

  ///< Number of state variables: 
  ///< [posX, posY, velocityX, velocityY]
  int stateSize;

  ///< Number of measurement variables: 
  ///< [measPosX, measPosY]
  int measSize;

  ///< Number of variables in the contr state vector. 
  int contrSize;

  ///< Time at which the ball was lost.
  float timeAtLost;

  ///< Time since the ball was lost.
  float timeSinceLost;

  ///< Whether the ball is lost.
  bool lost;

  ///< Whether we are currently getting no ball observation.
  bool losingBall;

  ///< Time step for the kalman filter updates.
  float cycleTime;

  ///< Comm module pointer object.
  PlanningModule* planningModule;
};
