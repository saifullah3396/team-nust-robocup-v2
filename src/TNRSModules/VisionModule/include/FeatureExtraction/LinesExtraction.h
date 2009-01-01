/**
 * @file VisionModule/include/FeatureExtraction//LinesExtraction.h
 *
 * This file declares the class for field lines extraction from
 * the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017
 */

#pragma once

#include <Eigen/Dense>
#include "TNRSBase/include/DebugBase.h"
#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"

using namespace cv;

class FieldExtraction;
class GoalExtraction;
class RegionSegmentation;
class RobotExtraction;
class FittedLine;
class ScannedEdge;
typedef boost::shared_ptr<ScannedEdge> ScannedEdgePtr;
typedef boost::shared_ptr<FittedLine> FittedLinePtr;
struct Circle;

/**
 * @class LineExtraction
 * @brief The class for extracting lines from the input image.
 */
class LinesExtraction : public FeatureExtraction, public DebugBase
{
  INIT_DEBUG_BASE_(
    ///< Option to send total module time.
    (int, sendTime, 0),
    ///< Option to draw scanned edges
    (int, drawScannedEdges, 0),
    ///< Option to draw field border lines
    (int, drawBorderLines, 0),
    ///< Option to draw world lines
    (int, drawWorldLines, 0),
    ///< Option to draw filtered world lines
    (int, drawFiltWorldLines, 0),
    ///< Option to draw middle circle if found
    (int, drawCircle, 0),
    ///< Option to draw middle circle if found
    (int, drawUnknownLandmarks, 0),
    ///< Option to draw L,T corners
    (int, drawCorners, 0),
    ///< Option to display info about extraction
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
  LinesExtraction(VisionModule* visionModule);

  /**
   * Destructor
   */
  ~LinesExtraction() {}

  /**
   * @brief processImage Derived from FeatureExtraction
   */
  void processImage();

  //static bool checkEllipse(Ellipse& e);
private:
  /**
   * @brief scanForEdges Scans the image to find line edges
   * @param connectedEdges Output edges
   * @return true if edges are found
   */
  bool scanForEdges(vector<vector<ScannedEdgePtr> >& connectedEdges);

  /**
   * @brief findConnectedEdges Links scanned edges together based on
   *   distance threshold in x-y
   * @param connectedEdges Output linked edges
   * @param scannedEdges Input scanned edgse
   * @param xTol Tolerance in X
   * @param yTol Tolerance in Y
   * @param verticalScan True if vertical scan else false
   */
  void findConnectedEdges(
    vector<vector<ScannedEdgePtr> >& connectedEdges,
    vector<ScannedEdgePtr>& scannedEdges,
    const unsigned& xTol,
    const unsigned& yTol,
    const bool& verticalScan);

  /**
   * @brief findLinesFromEdges Creates lines by fitting on edges
   * @param connectedEdges Input edges
   * @param lines Output lines
   */
  void findLinesFromEdges(
    const vector<vector<ScannedEdgePtr> >& connectedEdges,
    vector<FittedLinePtr>& lines);

  /**
   * @brief filterLines Filters lines based on their similarity in
   *   angles and distances in world frame (Taking hints from robocup field)
   * @param worldLines Input lines that are updated
   * @param circlePoints Points that might belong to the middle circle
   */
  void filterLines(vector<FittedLinePtr>& worldLines, vector<Point2f>& circlePoints);

  /**
   * @brief findFeatures Finds world features from line intersections
   * @param worldLines Input world lines
   */
  void findFeatures(vector<FittedLinePtr>& worldLines);

  /**
   * @brief computeLandmark Computes a lines intersection landmark
   * @param inter Point of intersection
   * @param unitToBaseLine Unit vector to landmark base line
   * @param type Type of landmark
   */
  void computeLandmark(
    const Point2f& inter,
    const Point2f& unitToBaseLine,
    const unsigned& type);

  /**
   * @brief computeCircleLandmark Computes a field middle circle landmark
   * @param c Circle
   * @param worldLines Lines
   */
  void computeCircleLandmark(
    const Circle& c,
    const vector<FittedLinePtr>& worldLines);

  /**
   * @brief findCircle Fits a circle on circle points
   * @param circleOutput Ouput circle
   * @param circlePoints Input circle points
   * @return true if a circle is found
   */
  bool findCircle(Circle& circleOutput, vector<Point2f>& circlePoints);

  /**
   * @brief findCircleLineIntersection Finds intersections of line with circle
   * @param c Circle
   * @param wl Line
   * @param intersections Points of intersection
   * @return
   */
  bool findCircleLineIntersection(
    const Circle& c, const FittedLinePtr& wl,
    vector<Point2f>& intersections);

  /**
   * @brief addLineLandmarks Adds a line as a landmark
   * @param worldLines
   */
  void addLineLandmarks(vector<FittedLinePtr>& worldLines);

  /**
   * @brief worldToImage Converts a point from world coordinates to world image coordinates
   * @param point Input point in world
   * @return Output point in world image
   */
  Point worldToImage(const Point& point);

  /**
   * @brief worldToImage Converts a point from world coordinates to world image coordinates
   * @param point Input point in world
   * @return Output point in world image
   */
  Point2f worldToImage(const Point2f& point);

  ///< World lines output image
  Mat worldImage;

  ///< Field Extraction module object
  boost::shared_ptr<FieldExtraction> fieldExt;

  ///< Robot Extraction module object
  boost::shared_ptr<RobotExtraction> robotExt;

  ///< Lines Extraction module object
  boost::shared_ptr<RegionSegmentation> regionSeg;

  ///< FittedLine iterator
  typedef vector<FittedLinePtr>::iterator FlIter;

  ///< Processing times
  float edgeScanTime;
  float fitLinesTime;
  float filterLinesTime;
  float findFeaturesTime;
  float findCircleTime;
  float addLandmarksTime;

  ///< Scanning step sizes
  int scanStepHighUpperCam;
  int scanStepHighLowerCam;
  int scanStepLowUpperCam;
  int scanStepLowLowerCam;

  float connectedEdgesHighTolRatio = {2.5}; ///< Tolerance ratio wrt highStep in direction of scan for scanned edges
  int minLineChainLength = {3}; ///< Minimum number of points for making a chain of line out of line edges
  int linesRANSACMaxIter = {5}; ///< RANSAC maximum iterations for fitting a line on points
  float linesRANSACMaxDist = {0.05}; ///< Maxmimum distane from line for RANSAC fitting
  float linesRANSACGoodCntRatio = {0.95}; ///< Ratio of points above which the line is considered a ver good fit
  float linesRANSACBadCntRatio = {0.5}; ///< Minimum ratio of points required from given points for considering it a sufficient fit
  int linesFitMinPointCnt = {3}; ///< Minimum number of points needed for fitting a line after RANSAC
  float pointsOnLineDistTol = {0.1}; ///< Distance within which points are associated with a line resulting from RANSAC
  float minDistFromBorder = {0.1}; ///< Minimum distance from border in cms for line to be consdered
  float circleLineMaxLength1 = {1.0}; ///< Maximum length of a line to be consdered a part of a circle, first time filteration
  float minDistFromBorderCircleLine1 = {1.5}; ///< Minimum distance required from border for a line to be consdered a circle line, first time filteration
  float overlappingLinesTolerance = {0.1}; ///< Distane within which lines are considered overlapping and filtered out
  float circleLineMaxLength2 = {0.65}; ///< Maximum length of a line to be consdered a part of a circle, second time filteration
  float minDistFromBorderCircleLine2 = {2.5}; ///< Minimum distance required from border for a line to be consdered a circle line, second time filteration
  int interPointImagePadding = {20}; ///< Image boundary padding within which a line itersection point is considered
  float midCircleRadiusWorld = {0.75}; ///< Circle radius in world, 0.75 m
  int minRANSACCirclePointsCnt = {15}; ///< Minimum points considered in RANSAC for circle
  float minMeanToMaxCircleDistRatio = {0.5}; ///< Ratio of minimum distance required from mean of points to furthest point wrt circle radius
  int maxRANSACCirclePointsCnt = {40}; ///< Maximum points considered in RANSAC for circle
  float maxRANSACCircleToPointDist = {0.05}; ///< Maximum points to circle distance considered in RANSAC for circle
  float circleRANSACGoodCntRatio = {0.65}; ///< Ratio of points count above which a circle is considered a good fit
  float cirlceBisectorMaxDist = {0.2}; ///< Maximum distance of cirlce bisector from center for considered a circle mid line
  float linesLandmarktNDiscretizationDist = {0.2}; ///< Distance for equally spaced points resulting from a line
  float maxLineUnknownLandmarks = {40};
  unsigned maxCircleStates = {5};
  Mat mapDrawing;
};
