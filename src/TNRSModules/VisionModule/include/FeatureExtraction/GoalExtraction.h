/**
 * @file VisionModule/include/FeatureExtraction//GoalExtraction.h
 *
 * This file declares the class for goal extraction from
 * the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017
 */

#pragma once

#include "TNRSBase/include/DebugBase.h"
#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"

class ScannedRegion;
typedef boost::shared_ptr<ScannedRegion> ScannedRegionPtr;
class GoalPost;
typedef boost::shared_ptr<GoalPost> GoalPostPtr;
class RegionSegmentation;
class FieldExtraction;
class RobotExtraction;

/**
 * @class GoalExtraction
 * @brief The class for extracting goal from the input image.
 */
class GoalExtraction : public FeatureExtraction, public DebugBase
{
  INIT_DEBUG_BASE_(
    ///< Option to send total module time.
    (int, sendTime, 0),
    ///< Option to draw goal scanned lines
    (int, drawScannedLines, 0),
    ///< Option to draw goal scanned regions
    (int, drawScannedRegions, 0),
    ///< Option to draw shifted goal base border
    (int, drawShiftedBorderLines, 0),
    ///< Option to draw goal base windows
    (int, drawGoalBaseWindows, 0),
    ///< Option to draw the goal posts base points.
    (int, drawGoalPostBases, 0),
    ///< Option to display information about the extracted results
    (int, displayInfo, 0),
    ///< Option to display image output
    (int, displayOutput, 0),
  );

public:

  /**
   * Default constructor for this class.
   *
   * @param visionModule: Pointer to parent VisionModule.
   */
  GoalExtraction(VisionModule* visionModule);

  /**
   * Default destructor for this class.
   */
  ~GoalExtraction() {}

  /**
   * @brief processImage Derived from FeatureExtraction
   */
  void processImage();

  /**
   * Returns the best goal posts extracted from the image.
   *
   * @return vector<Point>
   */
  //vector<vector<Point2f> > getBestGoalPosts() { return bestGoalPosts; }

private:
  /**
   * @brief refreshGoalPosts Removes goal posts previous info after refresh time
   */
  void refreshGoalPosts();

  /**
   * @brief filterGoalLines Filters out the scanlines of goalposts using border information
   * @param verGoalLines vertical scan lines for goal regions to be updated
   * @return false if unsuccessful
   */
  bool filterGoalLines(vector<LinearScannedLinePtr>& horGoalLines);

  /**
   * @brief classifyPosts Classifies a region as goal post
   * @param horGoalRegions Convex hulls for possible goal posts
   */
  void classifyPosts(vector<vector<Point>>& horGoalHulls);

  /**
   * @brief findBestPosts Finds best description of the goal post based on its
   *   position in image and in real world
   */
  void findBestPosts();

  /**
   * @brief updateGoalInfo Updates the newly observed goal info to memory
   */
  void updateGoalInfo();

  /**
   * @brief findGoalSide Finds the goal side based on team robot data
   * @param goalInfo GoalInfo that is updated based on found side
   */
  void findGoalSide(GoalInfo<float>& goalInfo);

  /**
   * @brief addGoalPost Adds the goal post to memory
   * @param goalPost Goal post to be added
   */
  void addGoalPost(const GoalPostPtr& goalPost);

  /**
   * @brief addGoalPost Adds the goal post to memory as an obstacle
   * @param goalPost Goal post to be added
   */
  void addGoalObstacle(const GoalPostPtr& goalPost);

  /**
   * @brief addGoalPost Adds the goal post to memory as a landmark
   * @param goalPost Goal post to be added
   */
  void addGoalLandmark(const GoalPostPtr& goalPost);

  /**
   * Derived from FeatureExtraction
   */
  void drawResults();

  ///< Procesing times
  float processTime;
  float scanTime;
  float classifyPostsTime;
  float findBestPostsTime;
  float updateGoalInfoTime;

  ///< Best goals extracted.
  //vector<vector<Point2f> > bestGoalPosts;

  ///< Vector of detected goal posts along with history information
  vector<GoalPostPtr> goalPosts;

  ///< Region segmentation module object
  boost::shared_ptr<RegionSegmentation> regionSeg;

  ///< Field Extraction module object
  boost::shared_ptr<FieldExtraction> fieldExt;

  ///< Robot Extraction module object
  boost::shared_ptr<RobotExtraction> robotExt;

  static constexpr float refreshTime = 0.5;
  typedef vector<GoalPostPtr>::iterator GPIter;
};
