/**
 * @file VisionModule/include/FeatureExtraction/RobotExtraction.h
 *
 * This file declares the class for field corners extraction from
 * the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 24 Aug 2017
 */

#pragma once

#include "TNRSBase/include/DebugBase.h"
#include "Utils/include/DataHolders/ObstacleType.h"
#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"
#include "VisionModule/include/FeatureExtraction/RobotTracker.h"

class RegionSegmentation;
class FieldExtraction;
class RobotRegion;
typedef boost::shared_ptr<RobotRegion> RobotRegionPtr;

/**
 * @class RobotExtraction
 * @brief The class for extracting robots from the input image.
 */
class RobotExtraction : public FeatureExtraction, public DebugBase
{
  INIT_DEBUG_BASE_(
    ///< Option to send total module time
    (int, sendTime, 0),
    ///< Option to draw robot scanned lines
    (int, drawScannedLines, 0),
    ///< Option to draw the bounding boxes for extracted jerseys
    (int, drawJerseyRegions, 0),
    ///< Option to draw LowerBody obstacles
    (int, drawLowerBodyRegions, 0),
    ///< Option to draw robot scanned regions
    (int, drawRobotRegions, 0),
    ///< Option to display info about extraction results
    (int, displayInfo, 0),
    ///< Option to display image output
    (int, displayOutput, 0),
  );

public:
  /**
   * Constructor
   *
   * @param visionModule: Pointer to parent VisionModule.
   */
  RobotExtraction(VisionModule* visionModule);

  /**
   * Destructor
   */
  ~RobotExtraction() {}

  /**
   * @brief processImage Derived from FeatureExtraction
   */
  void processImage();

  /**
   * @brief getRobotRegions Returns the robot regions in image found
   * @return
   */
  const vector<RobotRegionPtr>& getRobotRegions() { return classifiedRobotRegions; }

  /**
   * @brief getRobotRegions Returns the robot regions found in lower cam
   * @return Robot regions
   */
  const vector<ScannedRegionPtr>& getLowerCamRobotRegions() { return lowerCamRobotRegions; }

  /**
   * @brief getTrackedRobots Returns the vector containing robot trackers
   */
  const vector<boost::shared_ptr<RobotTracker>>& getTrackedRobots()
    { return trackedRobots; }

private:
  /**
   * @brief loadRobotClassifier Loads robot classifier from config/Classifiers
   */
  void loadRobotClassifier();

  /**
   * @brief refreshRobotRegions Refreshes the robot regions
   *   Every found region is kept for amount of 'refreshTime' defined and
   *   then removed.
   */
  void refreshRobotRegions();

  /**
   * @brief filterRobotLines Filters the lines scanned for robots
   */
  void filterRobotLines();

  /**
   * @brief findJerseys Finds jersey regions from jersey scanned lines
   * @param jerseyRegions Output jersey regions
   */
  void findJerseys(vector<RobotRegionPtr>& outputRegions);

  /**
   * @brief classifyRobots Integrates the info from robot scanned lines
   *   and jersey regions to find overall robot regions
   * @param robotRegions Output robot regions extracted from jerseys
   * @param jerseyRegions Regions extracted from jerseys
   */
  void classifyRobots(vector<RobotRegionPtr>& robotRegions, vector<RobotRegionPtr>& jerseyRegions);

  /**
   * @brief findLowerBodyRegions Finds LowerBody regions that do not count as robots
   * @param robotRegions Output robot regions
   */
  void findLowerBodyRegions(vector<RobotRegionPtr>& robotRegions);

  /**
   * @brief updateRobotTrackers Updates robot trackers with new robots information
   * @param robotRegions New robot regions found
   */
  void updateRobotTrackers(
    const vector<boost::shared_ptr<RobotRegion>>& robotRegions);

  /**
   * @brief updateRobotsInfo Updates robots info in memory
   */
  void updateRobotsInfo();

  /**
   * @brief drawResults Derived from FeatureExtraction
   */
  void drawResults();

  ///< OpenCv Cascade classifier for robots.
  // Not used anymore
  //CascadeClassifier classifier;

  ///< Field Extraction module object.
  boost::shared_ptr<FieldExtraction> fieldExt;

  ///< Lines Extraction module object.
  boost::shared_ptr<RegionSegmentation> regionSeg;

  ///< Tracked robot Regions
  vector<boost::shared_ptr<RobotTracker>> trackedRobots;

  ///< Tracked robot Regions obtained from trackers
  vector<boost::shared_ptr<RobotRegion>> classifiedRobotRegions;

  ///< Lower cam robot regions
  vector<boost::shared_ptr<ScannedRegion>> lowerCamRobotRegions;

  ///< Processing times
  float processTime;
  float linesFilterTime;
  float findJerseysTime;
  float classifyRobotsTime;
  float findStratRegionsTime;
  float updateRobotsInfoTime;

  typedef vector<RobotRegionPtr>::iterator RRIter;

  ///< Time taken to refresh previous robot information
  static float refreshTime;

  ///< Max number of robots that can be tracked at a time
  static unsigned maxRobotTrackers;

  ///< Threshold for maximum width of a standing robot in real world
  static float maxRobotWorldWidth;

  ///< Threshold for maximum width of a fallen robot in real world
  static float fallenRobotWidth;

  ///< Maximum distance between two robots to be considered the same
  static float robotMatchMaxDistance;

  ///< Approximate height of jersey for computations when robot feet cannot be seen
  static float jerseyApproxHeight;

  ///< Maximum jersey width possible with respect to
  static float maxJerseyWidthRatio;
};
