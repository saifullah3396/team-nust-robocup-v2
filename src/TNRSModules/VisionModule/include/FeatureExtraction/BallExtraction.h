/**
 * @file VisionModule/include/FeatureExtraction/BallExtraction.h
 *
 * This file declares the class for ball extraction from the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <opencv2/objdetect/objdetect.hpp>
#include "TNRSBase/include/DebugBase.h"
#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/optional_debug_tools.h"

using namespace tflite;

typedef boost::shared_ptr<Rect> RectPtr;

class FieldExtraction;
class RobotExtraction;
class RegionSegmentation;
class BallTracker;

/**
 * @class BallExtraction
 * @brief The class for ball extraction.
 */
class BallExtraction : public FeatureExtraction, public DebugBase
{
  INIT_DEBUG_BASE_(
    (int, sendTime, 0),
    (int, drawPredictionState, 0),
    (int, drawScannedLines, 0),
    (int, drawUnlinkedScannedRegions, 0),
    (int, drawClassifiedPentagons, 0),
    (int, drawClassifiedTriangles, 0),
    (int, drawBallCircles, 0),
    (int, drawScannedRegions, 0),
    (int, drawBallContour, 0),
    (int, drawPredictionROI, 0),
    (int, displayInfo, 0),
    (int, displayOutput, 0),
  );

public:
  /**
   * Constructor
   *
   * @param visionModule: Pointer to parent VisionModule.
   */
  BallExtraction(VisionModule* visionModule);

  /**
   * Destructor
   */
  ~BallExtraction() {}

  /**
   * @brief processImage Derived from FeatureExtraction
   */
  void processImage();

  /**
   * Returns the state of the ball if it is found or not
   */
  bool getBallFound();
private:
  /**
   * @brief drawResults Derived from FeatureExtraction
   */
  void drawResults();

  /**
   * @brief loadBallClassifier Loads the ball classifier from config/Classifiers
   * @throws VisionException if not found
   */
  void loadBallClassifier();

  /**
   * @brief resetBallTracker Resets the ball tracker state according to
   *   current image
   * @return False if the current image does not match the expected ball image
   */
  bool resetBallTracker();

  /**
   * @brief redBallDetector A red ball detector
   * @param origRect image rect
   */
  void redBallDetector(const Rect& origRect);

  /**
   * @brief drawState Draws a ball state on image
   * @param state State of the ball
   * @param color Color to draw with
   */
  void drawState(const Mat& state, const Scalar& color = Scalar(255,255,255));

  /**
   * @brief getBallFrameInNextCycle Checks whether ball would go in other camera range
   *   in next update
   * @param ballInfo BallInfo
   */
  unsigned getBallFrameInNextCycle(BallInfo<float>& ballInfo);

  /**
   * @brief findBallRegions Finds possible ball regions from robot lines
   * @param ballRegions Output regions
   */
  void findBallRegions(vector<boost::shared_ptr<ScannedRegion> >& ballRegions);

  /**
   * @brief findBallROIFromPredState Uses predicted ball state to
   *   find the ball
   * @param roi Output ball roi
   * @param predState Predicted state
   */
  void findBallROIFromPredState(Rect& roi, Mat& predState);

  /**
   * @brief filterFromRobotRegions Filters out the ball regions by removing
   *   known robot regions from them
   * @param ballRegions Ball regions
   */
  void filterFromRobotRegions(
    vector<boost::shared_ptr<ScannedRegion> >& ballRegions);

  /**
   * @brief findCandidatesWithBallFeatures Finds ball regions based on ball features
   * @param ballRegions Input regions
   * @param bestCandidates Ball region candidates that have passed all tests
   * @param poorCandidates Ball region candidates that have passed only a
   *   sufficient amount of tests
   */
  void findCandidatesWithBallFeatures(
    vector<boost::shared_ptr<ScannedRegion> >& ballRegions,
    vector<Rect>& bestCandidates,
    vector<Rect>& poorCandidates);

  /**
   * @brief classifyRegionsCNN Classifies all the regions based on CNN classifier
   * @param boundRects Possible ball regions
   */
  void classifyRegionsCNN(vector<Rect>& boundRects);

  /**
   * @brief classifyRegionsCascade Classifies all the regions based on cascade classifier
   * @param boundRects Possible ball regions
   */
  void classifyRegionsCascade(vector<Rect>& boundRects);

  /**
   * Applies the classifier to given image and stores ball bouding box
   * in detectionResult.
   *
   * @param origRect: The rect defining the cropped region in original
   *   image
   * @param croppedImage: Image under consideration.
   */
  void applyClassifier(const Rect& origRect, Mat& croppedImage);

  /**
   * Updates ballInfo variable in the memory.
   */
  void updateBallInfo();

  boost::shared_ptr<FieldExtraction> fieldExt; ///< Field Extraction module object.
  boost::shared_ptr<RobotExtraction> robotExt; ///< Robot Extraction module object.
  boost::shared_ptr<RegionSegmentation> regionSeg; ///< Field Extraction module object.
  boost::shared_ptr<BallTracker> ballTracker;///< Ball tracker class object.

  vector<Rect> classifiedBalls; ///< Balls classified in current iteration
  float ballRadius = {0.05}; ///< Ball radius in xyz frame
  unsigned ballType = {1}; ///< Type of the ball
  float coeffSF; ///< Static friction coefficients
  float coeffRF; ///< Rolling friction coefficients

  float lineLinkHorXTolRatio = {1.5}; ///< Line difference ratio in X for horizontal scan
  float lineLinkHorYTolRatio = {1.5}; ///< Line difference ratio in Y for horizontal scan
  float lineLinkVerXTolRatio = {1.5}; ///< Line difference ratio in X for vertical scan
  float lineLinkVerYTolRatio = {1.5}; ///< Line difference ratio in Y for vertical scan
  float maxLineLengthDiffRatio = {1.25}; ///< Line length difference ratio for both scans
  float regionsXDiffTol = {16}; ///< Region link max distance in X in pixels
  float regionsYDiffTol = {16}; ///< Region link max distance in Y in pixels
  float maxRegionSizeDiffRatio = {2.5}; ///< Region link max difference in sizes
  float regionFilterMinWidth = {10}; ///< Minimum width for regions found
  float regionFilterMinHeight = {10}; ///< Minimum height for regions found
  float regionFilterMaxAspectRatio = {5}; ///< Minimum aspect ratio for regions found in width and height

  float robotFilterHeightRatio = {0.8}; ///< Max height to be considered for found robot regions
  float fallenRobotFilterHeightRatio = {0.5}; ///< Max height to be considered for found fallen robot regions
  float robotMaxOverlapTop = {0.65}; ///< Max overlap threshold for ball to top robot regions
  float robotMaxOverlapBottom = {0.95}; ///< Max overlap threshold for ball to bottom robot regions
  float minOverlapAreaThreshold = {100}; ///< Minimum overlap area threshold for overlap to be considered
  float maxBallRegionSizeRatio = {0.5}; ///< Maximum regions size wrt image size above which they are discarded

  float ballRegionPaddingRatio = {1.25}; ///< Padding ratio wrt region size when finding ball features
  float adaptiveThresholdWindowSizeRatio = {2.0}; ///< Ratio of size of window to size of expected ball size for adaptive threshold
  float adaptiveThresholdSubConstantRatio = {2.0}; ///< Ratio of subtraction constant wrt expected ball size for adaptive threshold
  float maxBallBlobSizeRatio = {0.8}; ///< Ratio of maximum blob size for ball pentagon features wrt expected ball size
  float minBallBlobSizeRatio = {0.2}; ///< Ratio of minimum blob size for ball pentagon features wrt expected ball size
  float ballBlobMaxAspectRatio = {3}; ///< Minimum aspect ratio for regions found in width and height
  float maxBallBlobIntensity = {100}; ///< Maximum blob intensity for ball blobs
  int minPentagonsRequired = {3}; ///< Minimum 3 are required for making a triangle combination
  int maxPentagonsRequired = {6}; ///< Maximum 6 are set because ball cannot have many blobs
  int minPentagonsPoorCandidates = {2}; ///< Minimum 2 pentagons are required to call it a poor but sufficient candidates
  float maxBlobToBlobSizeDiffRatio = {0.5}; ///< Maximum difference of sizes between blobs in one combination of 3

  int gaussianSizeX = {3}; ///< X Size of gaussian filter to be applied on binary image for hough circle
  int gaussianSizeY = {3}; ///< Y Size of gaussian filter to be applied on binary image for hough circle
  int gaussianSigmaX = {2}; ///< X Sigma of gaussian filter to be applied on binary image for hough circle
  int gaussianSigmaY = {2}; ///< Y Sigma of gaussian filter to be applied on binary image for hough circle

  int houghCirclesMethod = {CV_HOUGH_GRADIENT}; ///< OpenCv method used for hough circles
  int houghCirclesMinDistRatio = {2}; ///< Minimum distance between two circles wrt image size
  int minRadiusPixelTolerance = {25}; ///< Minimum radius limit wrt expected ball size
  int maxRadiusPixelTolerance = {25}; ///< Maximum radius limit wrt expected ball size

  float triangleToCircleMaxDistRatio = {2.0}; ///< Maximum distance of circle center to triangle center wrt ball size
  float cascadePaddingRatio = {1.25}; ///< Padding of rectangle while applying cascade filter
  float CNNClassificationTolerance = 0.25; ///< Max difference of classified label and actual label

  float predictedAreaRoiRatio = {3}; /// Size of ROI wrt predicted state

  std::unique_ptr<FlatBufferModel> model; ///< Tflite model
  ops::builtin::BuiltinOpResolver resolver; ///< Tflite model resolvier
  std::unique_ptr<Interpreter> interpreter;///< Tflite model interpreter

  CascadeClassifier cascade; ///< OpenCv Cascade classifier for ball.

  ///< Processing times
  float processTime = {0.f};
  float resetBallTrackerTime = {0.f};
  float findBallFromPredStateTime = {0.f};
  float findBallRegionsTime = {0.f};
  float filterFromRobotRegionsTime = {0.f};
  float findCandidatesWithBallFeaturesTime = {0.f};
  float classifyRegionsCNNTime = {0.f};
  float classifyRegionsCascadeTime = {0.f};
  float redBallDetectorTime = {0.f};
  float updateBallInfoTime = {0.f};
};
