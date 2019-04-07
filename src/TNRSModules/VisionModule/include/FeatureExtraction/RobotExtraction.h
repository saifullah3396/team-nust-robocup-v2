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

class BallExtraction;
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
    ///< Option to draw stray obstacles
    (int, drawStrayRegions, 0),
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
   * @brief getRobotRegions Returns the robot regions found
   * @return
   */
  vector<RobotRegionPtr>& getRobotRegions() { return filtRobotRegions; }

  /**
   * @brief getStrayRegions Returns the unknown regions found in field
   * @return
   */
  vector<Rect>& getStrayRegions() { return strayRegions; }

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
   * @brief filterBallRegions Filter out lines in ball region
   * @param robotLines Lines to filter
   * @param direction Direction of filter, false for horizontal, true for vertical
   */
  void filterBallRegions(
    vector<boost::shared_ptr<LinearScannedLine> >& robotLines, const bool& direction);

  /**
   * @brief findJerseys Finds jersey regions from jersey scanned lines
   */
  void findJerseys();

  /**
   * @brief classifyRobots Integrates the info from robot scanned lines
   *   and jersey regions to find overall robot regions
   */
  void classifyRobots();

  /**
   * @brief findStrayRegions Finds stray regions that do not count as robots
   */
  void findStrayRegions();

  /**
   * @brief updateRobotsInfo Updates robots info in memory
   */
  void updateRobotsInfo();

  ///< Robot Regions
  vector<RobotRegionPtr> jerseyRegions;

  ///< Robot Regions
  vector<RobotRegionPtr> filtRobotRegions;

  ///< Robot vertical scanned lines
  vector<LinearScannedLinePtr> verRobotLines;

  ///< Robot horizontal scanned lines
  vector<LinearScannedLinePtr> horRobotLines;

  ///< Field border
  vector<int> border;

  ///< Vector of regions that are not part of lines.
  vector<Rect> strayRegions;

  /**
   * @brief drawResults Derived from FeatureExtraction
   */
  void drawResults();

  ///< OpenCv Cascade classifier for robots.
  // Not used anymore
  //CascadeClassifier classifier;

  ///< Ball Extraction module object.
  boost::shared_ptr<BallExtraction> ballExt;

  ///< Field Extraction module object.
  boost::shared_ptr<FieldExtraction> fieldExt;

  ///< Lines Extraction module object.
  boost::shared_ptr<RegionSegmentation> regionSeg;

  ///< Processing times
  float processTime;
  float linesFilterTime;
  float findJerseysTime;
  float classifyRobotsTime;
  float findStratRegionsTime;
  float updateRobotsInfoTime;

  typedef vector<RobotRegionPtr>::iterator RRIter;

  ///< Time taken to refresh previous robot information
  static constexpr float refreshTime = 0.01f;
};
