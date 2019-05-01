/**
 * @file LocalizationModule/include/FieldMap.h
 *
 * The class for generating the map for robot localization.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author TeamNust 2015
 * @date 28 Sep 2017
 */

#pragma once

#include "TNRSBase/include/BaseIncludes.h"
#include "TNRSBase/include/DebugBase.h"

template <typename Scalar>
struct TNRSLine;
class ParticleFilter;
class LocalizationModule;
class ObstacleTracker;

/**
 * @class FieldMap
 * @brief The class for generating the map for robot localization.
 */
class FieldMap : public MemoryBase, DebugBase
{
  INIT_DEBUG_BASE_(
    ///< Option to enable any kind of debugging.
    (int, debug, 0),
    ///< Option to send particles to the debugger.
    (int, sendParticles, 0),
    ///< Option to send the obstacles map to the debugger.
    (int, sendObstacleMap, 0),
    ///< Draws the particles on the map
    (int, drawParticles, 0),
    ///< Draws the FOV lines on the map
    (int, drawFOVLines, 0),
    ///< Displays occupancy map as output
    (int, displayOutput, 0),
    ///< Displays information about the results
    (int, displayInfo, 0),
  );
public:
  /**
   * @brief FieldMap Constructor
   * @param localizationModule Pointer to base localization module
   */
  FieldMap(LocalizationModule* localizationModule);

  /**
   * @brief ~FieldMap Destructor
   */
  ~FieldMap() {}

  /**
   * @brief update Updates the observed obstacles and occupancy map
   */
  void update();

  /**
   * @brief updateRobotPose2D Updates the current robot pose to memory
   */
  void updateRobotPose2D();

  /**
   * @brief worldToMap Transforms a point from world coordinates
   *   to map coordinates
   * @param p Point
   * @return Transformed point
   */
  template<typename T>
  cv::Point_<int> worldToMap(const cv::Point_<T>& p);

  ///< Setters
  void setBallObstacle(const bool& addBallObstacle)
    { this->addBallObstacle = addBallObstacle; }

private:
  /**
   * @brief updateFOVLines Updates the field of view lines in
   *   the field
   */
  void updateFOVLines();

  /**
   * @brief updateOccupancyMap Updates the occupancy map
   */
  void updateOccupancyMap();

  /**
   * @brief updateObstacleTrackers Updates the obstacle trackers with
   *   new obstacle data
   */
  void updateObstacleTrackers();

  /**
   * @brief setupViewVectors Sets up the field of view min-max vectors
   */
  void setupViewVectors();

  /**
   * @brief obstacleInFOV Returns true if the given obstacle is within FOV of robot
   * @param obsTracker Obstacle
   * @return boolean
   */
  bool obstacleInFOV(const boost::shared_ptr<ObstacleTracker>& obsTracker);

  ///< Pointer to localizer
  boost::shared_ptr<ParticleFilter> localizer;

  ///< Base matrix for generating occupancy map
  Mat occupancyMapBase;

  ///< The unit vector that represents the minimum field of view angle
  ///< from the lower camera frame and maximum field of view angle from
  ///< the upper camera frame
  vector<Vector3f> unitVecX;

  ///< The unit vector that represents the minimum field of view angle
  ///< from the lower camera frame and maximum field of view angle from
  ///< the upper camera frame
  vector<Vector3f> unitVecY;

  ///< Field of view lines
  boost::shared_ptr<TNRSLine<float> > leftFOVLine;
  boost::shared_ptr<TNRSLine<float> > rightFOVLine;

  ///< Whether to update the occupancy map or not
  bool useOccupancyMap = {false};

  ///< Previous id of the observed obstacles
  int prevObsId = {0};

  ///< Whether to add ball as an obstacle
  bool addBallObstacle = {false};

  ///< Radius of the ball used for obstacle generation
  float ballRadius = {0.05};

  ///< Maximum distance ratio to be considered a match with a previous obstacle
  float obstacleMatchMaxDistanceRatio = {0.5};

  ///< Maximum possible trackers that can exist
  unsigned maxObstacleTrackers = {5};

  ///< Refresh time for removing obstacles after prologed periods
  float refreshTime = {10};

  ///< Refresh time when obstacle is within view but not updated
  float refreshTimeForObsInFOV = {1};

  ///< Maximum field of view distance from robot, 3m
  float maxFOVDistance = {3};

  ///< Container for tracked obstacles in world
  vector<boost::shared_ptr<ObstacleTracker>> trackedObstacles;

  ///< Module cycle time
  float cycleTime = {0.05};

  ///< Pointer to the base localization module
  LocalizationModule* localizationModule;
};
