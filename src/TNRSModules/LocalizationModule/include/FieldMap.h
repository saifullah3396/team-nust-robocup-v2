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

class ParticleFilter;
class LocalizationModule;

/**
 * @class FieldMap
 * @brief The class for generating the map for robot localization.
 */
class FieldMap : public MemoryBase, DebugBase
{
  INIT_DEBUG_BASE_(
    //! Option to enable any kind of debugging.
    (int, debug, 0),
    //! Option to send particles to the debugger.
    (int, sendParticles, 0),
    //! Option to send the obstacles map to the debugger.
    (int, sendObstacleMap, 0),
    //! Draws the particles on the map
    (int, drawParticles, 0),
    //! Displays particle map
    (int, displayOutput, 0),
  )
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

  //! Setters
  void setBallObstacle(const bool& addBallObstacle)
    { this->addBallObstacle = addBallObstacle; }

private:
  //void updateWorldImage();
  //void drawField();
  //void drawRobot();
  /**
   * @brief updateOccupancyMap Updates the occupancy map
   */
  void updateOccupancyMap();

  //! Pointer to localizer
  boost::shared_ptr<ParticleFilter> localizer;

  //! Base matrix for generating occupancy map
  Mat_<float> occupancyMapBase;

  //Mat worldImage;
  //Mat mapDrawing;

  //! Previous id of the observed obstacles
  int prevObsId = {0};

  //! Whether to add ball as an obstacle
  bool addBallObstacle = {false};

  //! Radius of the ball used for obstacle generation
  float ballRadius;
};
