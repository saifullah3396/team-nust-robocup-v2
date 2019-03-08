/**
 * @file LocalizationModule/include/ParticleFilter.h
 *
 * This file declares the class ParticleFilter
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author TeamNust 2015
 * @date 17 Sep 2017
 */

#pragma once

#define MAX_STATE_ITERATIONS 100

#include <queue>
#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>
#include "LocalizationModule/include/Particle.h"
#include "TNRSBase/include/MemoryBase.h"
#include "TNRSBase/include/DebugBase.h"
#include "RandomLib/include/Random.hpp"
#include "Utils/include/DataHolders/PositionInput.h"

template <typename Scalar> struct RobotPose2D;
template <typename Scalar> struct Landmark;
typedef boost::shared_ptr<Landmark<float> > LandmarkPtr;
template <typename Scalar> struct KnownLandmark;
typedef boost::shared_ptr<KnownLandmark<float> > KnownLandmarkPtr;
template <typename Scalar> struct UnknownLandmark;
typedef boost::shared_ptr<UnknownLandmark<float> > UnknownLandmarkPtr;
class LocalizationModule;
using namespace RandomLib;

/**
 * @class ParticleFilter
 * @brief The class for implementing particle filter for robot
 *   localization.
 */
class ParticleFilter : public MemoryBase, DebugBase
{
  INIT_DEBUG_BASE_(
    //! Option to display landmarks map that is initialized at start
    (int, displayLandmarksMap, 0),
    //! Option to display voronoi map made from landmark positions
    (int, displayVoronoiMap, 0),
  )
public:
  /**
   * Constructor
   *
   * @param lModule: Pointer to parent LocalizationModule.
   */
  ParticleFilter(LocalizationModule* lModule);

  /**
   * Destructor
   */
  ~ParticleFilter() {}

  /**
   * Initializes the particles based on a normal distribution
   * around the initial state estimate for the given standard deviations
   *
   * @param state: Initial state estimate of the robot
   * @param stdDeviation: Standard deviations for the uncertainty in
   *   state estimate
   */
  void init(const RobotPose2D<float>& state);

  /**
   * @brief init Initializes the particles based on a normal distribution
   *   around the gives states
   *
   * @param states Input states vector
   */
  void init(const boost::circular_buffer<RobotPose2D<float> >& states);

  /**
   * Updates the particle filter
   */
  void update();

  /**
   * @brief reset Resets the filter
   */
  void reset();

  /**
   * Returns whether particle filter is initiated yet or not.
   */
  const bool isInitiated() const { return initiated; }

  /**
   * @brief isLocalized Returns true if the robot is localized
   * @return bool
   */
  const bool isLocalized() const { return localized; }

  /**
   * Gets the current average state of the robot.
   *
   * @return RobotPose2D<float>
   */
  RobotPose2D<float> getFilteredState() const { return avgFilteredState; }

  /**
   * @brief getParticles Gets the current particles
   * @return vector<Particle>
   */
  vector<Particle> getParticles() const { return particles; }

  /**
   * @brief getFieldLandmarks Gets known field landmarks
   * @return vector<LandmarkPtr>
   */
  vector<boost::shared_ptr<Landmark<float> > > getFieldLandmarks() const
    { return fieldLandmarks; }

  /**
   * @brief addPositionInput Adds a position input
   * @param input Input
   */
  void addPositionInput(const PositionInput<float>& input);

  /**
   * @brief setKnownLandmarks Sets the known landmarks vector
   * @param landmarks Vector
   */
  void setKnownLandmarks(const vector<KnownLandmarkPtr>& landmarks);

  /**
   * @brief setUnknownLandmarks Sets the unknown landmarks vector
   * @param landmarks Vector
   */
  void setUnknownLandmarks(const vector<UnknownLandmarkPtr>& landmarks);

private:
  /**
   * Sets up the landmarks information for the filter. Called once in constructor.
   */
  void setupLandmarks();

  /**
   * Sets up the voronoi map for landmark labels. Called once in constructor.
   */
  void setupVoronoiMap();

  /**
   * Sets up the camera range vectors. Called once in constructor.
   */
  void setupViewVectors();

  /**
   * Tries to determine the robots position according to known landmarks 
   * information.
   */
  void updateRobotStateEstimate();

  /**
   * @brief estimateForSideLines Estimates robot position based on initial estimate
   *   of side line position
   */
  void estimateForSideLines();

  /**
   * @brief estimateFromLandmarks Estimates robot position based on known
   *   landmarks information
   */
  void estimateFromLandmarks();

  /**
   * @brief minMaxGroundDist Finds the minimum and maximum distance that is in
   *   view of camera of the robot. This info is used to remove outlier particles
   */
  void minMaxGroundDist();

  /**
   * @brief prediction This function updates the particles according to the motion
   * model for a given velocity input and adds random gaussian noise.
   */
  void prediction();

  /**
   * @brief prediction Updates the particles according to the motion
   * model for a given velocity input and adds random gaussian noise.
   *
   * @param velocityInput: Velocity input that updates a particle state
   *   from time t to t+1.
   */
  void prediction(const VelocityInput<double>& vI);

  /**
   * @brief addPredictionNoise Adds noise prediction noise to particles
   */
  void addPredictionNoise();

  /**
   * Updates particle weights based on the likelihood of the observed 
   *   measurements.
   * @param obsLandmarks: Vector of observed landmarks
   */
  void updateWeights(const vector<LandmarkPtr>& obsLandmarks);

  /**
  * @brief normalizeWeights Normalizes all particle weights. Sets all weights equal in case
  *   there is no observation data available
  * @param noData True if there is no observation data
  */
  void normalizeWeights(const bool& noData);

  /**
  * @brief updatePositionConfidence Updates the position confidence
  */
  void updatePositionConfidence();

  /**
  * @brief updateSideConfidence Updates the side confidence
  */
  void updateSideConfidence();

  /**
  * Resampling from the updated set of particles to form the new set
  * of particles.
  */
  void resample();

  /**
   * @brief landmarkPoseToWorldPose Converts robot pose wrt to landmark to world pose
   * @param lPose Pose from landmark
   * @param rPose Robot pose
   */
   RobotPose2D<float> landmarkPoseToWorldPose(
    const RobotPose2D<float> lPose, const RobotPose2D<float> rPose);

  /**
   * @brief worldToVoronoiMap Converts a point in world to a point on voronoi map
   * @param p Input point in world
   * @return Output point in map
   */
  Point worldToVoronoiMap(const Point& p);

  /**
   * @brief worldToVoronoiMap Converts a point in world to a point on voronoi map
   * @param p Input point in world
   * @return Output point in map
   */
  Point2f worldToVoronoiMap(const Point2f& p);

  //! Whether the filter has been initiated with some initial estimate
  bool initiated;

  //! Whether the robot has been localized to a good extent
  bool localized;

  //! Total number of generated particles
  int nParticles;

  //! Vector containing all the particles
  vector<Particle> particles;

  //! The gaussian covariance constant parameter
  float gaussianConst;

  //! The exponential constant value for x-axis
  float expConstX;

  //! The exponential constant value for y-axis
  float expConstY;

  //! Sum of weights of all the particles
  double sumWeights;

  //! Maximum weight from all the particles
  double maxWeight;

  //! Landmarks grid resolution
  double lGridResolution;

  //! Voronoi map resolution
  double vMapResolution;

  //! Voronoi map size
  Size vMapSize;

  //! The number of times the robot is lost after which particle filter is reset
  int lostCount;

  //! Estimated poses of the robot found from known landmarks
  //! Currnetly only goal info is used for this estimate
  boost::circular_buffer<RobotPose2D<float> > estimatedStates;

  //! Vector of the actual field landmarks
  vector<LandmarkPtr> fieldLandmarks;

  //! Vector of the actual field landmarks, labelled according to their
  //! position using voronoi map of the environment
  vector<vector<LandmarkPtr> > labelledLandmarks;

  //! Number of landmarks in each type
  vector<unsigned> landmarkTypeCount;

  //! Starting index of each type of landmark
  vector<unsigned> landmarkTypeStarts;

  //! The unit vector that represents the minimum field of view angle
  //! from the lower camera frame and maximum field of view angle from
  //! the upper camera frame
  vector<Vector3f> unitVecY;

  //! Draws a particle on map
  void drawParticle(const Particle& p, Mat& image);
  void drawParticle(const RobotPose2D<float>& p, Mat& image);

  //! The minimum distance from the camera view on the ground
  float minDistGround;

  //! The maximum distance from the camera view on the ground
  float maxDistGround;

  //! Random  library object
  Random random;

  //! Parameters for augmented monte-carlo localization method
  double wSlow, wFast;

  //! Current average state of the robot
  RobotPose2D<float> avgFilteredState;

  //! Last known half the robot is in: 0 us/1 opponents.
  unsigned lastKnownHalf;

  //! Filter prediction standard deviation when the robot is standing
  Vector3d predictionStd;

  //! Filter prediction standard deviation when the robot is in motion
  Vector3d motionStd;

  //! Filter measurement standard deviation
  Vector3d measurementStd;

  //! Standard deviation used for states generation during initialization
  Vector3d genStd;

  //! Cycle time of the filter loop
  float cycleTime;

  //! Queue containing all the prediction control inputs
  queue<PositionInput<float> > positionInputs;
  
  //! Queue containing latest known landmarks observation.
  vector<KnownLandmarkPtr> knownLandmarks;
  
  //! Queue containing latest unknown landmarks observation.
  vector<UnknownLandmarkPtr> unknownLandmarks;

  //! Id of the last received goal info
  int prevGoalInfoId;

  //! A matrix containing the labelled voronoi image
  Mat voronoiLabels;

  //! Pointer to localization module object
  LocalizationModule* lModule;
};
