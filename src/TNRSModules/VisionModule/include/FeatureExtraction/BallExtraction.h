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
/*
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/optional_debug_tools.h"

using namespace tflite;
*/
typedef boost::shared_ptr<Rect> RectPtr;

class FieldExtraction;
class RegionSegmentation;
class BallTracker;

/**
 * @class BallExtraction
 * @brief The class for ball extraction.
 */
class BallExtraction : public FeatureExtraction, public DebugBase
{
  INIT_DEBUG_BASE_(
    ///< Option to send total module time
    (int, sendTime, 0),
    ///< Option to draw ball predicted state
    (int, drawPredictionState, 0),
    ///< Option to draw ball scanned regions
    (int, drawScannedRegions, 0),
    ///< Option to draw extracted ball contour
    (int, drawBallContour, 0),
    ///< Option to display info about the results
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
   * @brief getPredRoi Makes a roi for predicted state
   * @param predRoi Ouput roi
   * @param predState Predicted ball state
   */
  void getPredRoi(Rect& predRoi, const Mat& predState);

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

  void findBallRegions();

  /**
   * @brief findBallUpperCam Scans the ball in upper camera image
   */
  void findBall(vector<int>& pairIndices);

  /**
   * @brief findBallFromPredState Uses predicted ball state to
   *   find the ball
   * @param predState Predicted state
   */
  void findBallFromPredState(Mat& predState);

  /**
   * @brief findBallUpperCam Scans the ball in upper camera image
   * @param roi Region of interest
   */
  void findBallUpperCam(const Rect& roi);

  /**
   * @brief findBallLowerCam Scans the ball in lower camera image
   * @param roi Region of interest
   */
  void findBallLowerCam(const Rect& roi);

  /**
   * @brief scanRandom Scans an area by randomly choosing from the given
   *   bounding boxes in the area
   * @param boundRects Bounding boxes that are resulting from scan
   * @param pairIndices Indices for grid bounding boxes
   * @param iters Number of iterations to randomly scan the area
   */
  void scanRandom(vector<RectPtr>& boundRects, const vector<int>& pairIndices, const int& iters);

  /**
   * @brief scanRandom Scans the given roi
   * @param boundRects Bounding boxes that are resulting from scan
   * @param roi Region of interest
   */
  void scanRoi(vector<RectPtr>& boundRects, const Rect& roi);

  /**
   * @brief filterRegions Filters out the scanned ball regions
   * @param boundRects Possible ball regions
   * @param threshold dist threshold for considering two regions as one
   */
  void filterRegions(vector<RectPtr>& boundRects, const float& threshold);

  /**
   * @brief classifyRegions Classifies all the regions based on ball classifier
   * @param boundRects Possible ball regions
   */
  void classifyRegions(vector<RectPtr>& boundRects);

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

  ///< Balls found in current iteration
  vector<Rect> foundBall;

  ///< Ball radius in xyz frame
  float ballRadius;

  ///< Upper radius threshold.
  float ballRadiusMax;

  ///< Lower radius threshold.
  float ballRadiusMin;

  ///< OpenCv Cascade classifier for ball.
  CascadeClassifier cascade;

  ///< Field Extraction module object.
  boost::shared_ptr<FieldExtraction> fieldExt;

  ///< Field Extraction module object.
  boost::shared_ptr<RegionSegmentation> regionSeg;

  ///< Ball tracker class object.
  boost::shared_ptr<BallTracker> ballTracker;

  ///< Distance threshold for combining two regions
  vector<int> regionsDist;

  ///< Processing times
  float processTime;
  float resetBallTrackerTime;
  float ballDetectionTime;
  float scanTime;
  float regionFilterTime;
  float regionClassificationTime;
  float updateBallInfoTime;
  float findBallRegionsTime;

  ///< Ball scan step for lower cam
  int scanStepLow;
  int scanStepHigh;

  ///< Type of the ball
  unsigned ballType;

  ///< Friction coefficients
  float coeffSF;
  float coeffRF;

  ///< Ball scanning parameters
  vector<Point> topXYPairs;
  vector<Point> bottomXYPairs;
  vector<int> xySeen;
  Point gridSizeTop;
  Point gridSizeBottom;

  ///< Tflite mode interpreter
  //std::unique_ptr<FlatBufferModel> model;
  //ops::builtin::BuiltinOpResolver resolver;
  //std::unique_ptr<Interpreter> interpreter;
};
