/**
 * @file FeatureExtraction/RegionSegmentation.h
 *
 * This file declares the class for segmenting the image into different
 * regions for further processing.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author AbdulRehman
 * @date 5 Mar 2018
 */

#pragma once

#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"

class FieldExtraction;
typedef boost::shared_ptr<FieldExtraction> FieldExtractionPtr;
//class BallExtraction;
//class GoalExtraction;

/**
 * @class RegionSegmentation
 * @brief The class for extracting field from the input image.
 */
class RegionSegmentation : public FeatureExtraction, public DebugBase
{
  INIT_DEBUG_BASE_(
    //! Option to send total module time
    (int, sendTime, 0),
    //! Option to draw vertical lines
    (int, drawVerticalLines, 0),
    //! Option to draw horizontal lines
    (int, drawHorizontalLines, 0),
    //! Option to draw points
    (int, drawPoints, 0),
    //! Option to display detailed info on results of this module
    (int, displayInfo, 0),
    //! Option to display image output
    (int, displayOutput, 0),
  )

public:
  /**
   * Constructor
   *
   * @param visionModule: pointer to parent VisionModule
   */
  RegionSegmentation(VisionModule* visionModule);

  /**
   * Destructor
   */
  ~RegionSegmentation()
  {
  }

  /**
   * @brief processImage Derived from FeatureExtraction
   */
  void  processImage();

  /**
   * @brief setFieldExtraction Sets the pointer to field extraction module
   * @param fieldExt pointer to field extraction module
   */
  void setFieldExtraction(const boost::shared_ptr<FieldExtraction>& fieldExt)
  {
    this->fieldExt = fieldExt;
  }

  /**
   * @brief setBallExtraction Sets the pointer to ball extraction module
   * @param ballExt pointer to ball extraction module
   */
  //void setBallExtraction(const boost::shared_ptr<BallExtraction>& ballExt)
  //{
  //  this->ballExt = ballExt;
  //}

  /**
   * @brief setGoalExtraction Sets the pointer to goal extraction module
   * @param goalExt pointer to goal extraction module
   */
  //void setGoalExtraction(const boost::shared_ptr<GoalExtraction>& goalExt)
  //{
  //  this->goalExt = goalExt;
  //}

  /**
   * @brief getBorderPoints Returns the extracted field border points
   * @return vector<cv::Point>
   */
  vector<Point>& getBorderPoints() { return borderPoints; }

  /**
   * @brief getFieldAvgHeight Returns the field average height
   * @return int
   */
  int& getFieldAvgHeight() { return avgHeight; }

  /**
   * @brief getFieldMinBestHeight Returns the field mininimum best height
   * @return int
   */
  int& getFieldMinBestHeight() { return minBestHeight; }

  //vector<ScannedLinePtr>& getVerGoalLines() { return verGoalLines; }
  //vector<ScannedLinePtr>& getVerBallLines() { return verBallLines; }
  //vector<ScannedLinePtr>& getHorBallLines() { return horBallLines; }
  /**
   * @brief getVerRobotLines Returns the vertial scanned lines for robot regions
   * @return vector<ScannedLinePtr
   */
  vector<ScannedLinePtr>& getVerRobotLines() { return verRobotLines; }

  /**
   * @brief getHorRobotLines Returns the horizontal scanned lines for robot regions
   * @return vector<ScannedLinePtr
   */
  vector<ScannedLinePtr>& getHorRobotLines() { return horRobotLines; }

  /**
   * @brief getVerJerseyLinesOurs Returns the vertial scanned lines for team jerseys
   * @return
   */
  vector<ScannedLinePtr>& getVerJerseyLinesOurs() { return verJerseyLinesOurs; }

  /**
   * @brief getHorJerseyLinesOurs Returns the horizontal scanned lines for team jerseys
   * @return
   */
  vector<ScannedLinePtr>& getHorJerseyLinesOurs() { return horJerseyLinesOurs; }

  /**
   * @brief getVerJerseyLinesOpps Returns the vertial scanned lines for opponent jerseys
   * @return
   */
  vector<ScannedLinePtr>& getVerJerseyLinesOpps() { return verJerseyLinesOpps; }
  /**
   * @brief getHorJerseyLinesOpps Returns the horizontal scanned lines for opponent jerseys
   * @return
   */
  vector<ScannedLinePtr>& getHorJerseyLinesOpps() { return horJerseyLinesOpps; }

private:
  /**
   * Loads debug variables from config file
   */
  void getDebugVars();

  /**
   * @brief horizontaScan Performs a horizontal scan of the image
   */
  void horizontalScan();

  /**
   * @brief verticalScan Performs a vertical scan of the image
   */
  void verticalScan();

  /**
   * @brief drawResults Draws extracted scanned lines on bgr image matrix
   */
  void drawResults();

  //! Step size for direction parallel to scan
  int scanStepLow;

  //! Step size for direction perpendicular to scan
  int scanStepHigh;

  //! Vertical scan limiting image index
  int vScanLimitIdx;

  //! Horizontal scan limiting image index
  int hScanLimitIdx;

  //! Horizontal and vertical scan lines for robot regions
  vector<ScannedLinePtr> horRobotLines;
  vector<ScannedLinePtr> verRobotLines;

  //! Horizontal and vertical scan lines for teammate jerseys
  vector<ScannedLinePtr> horJerseyLinesOurs;
  vector<ScannedLinePtr> verJerseyLinesOurs;

  //! Horizontal and vertical scan lines for opponent jerseys
  vector<ScannedLinePtr> horJerseyLinesOpps;
  vector<ScannedLinePtr> verJerseyLinesOpps;

  //! Time taken by horizontal scan
  double horizontalScanTime;

  //! Time taken by vertical scan
  double verticalScanTime;

  //! Time taken by overall processing
  double processTime;

  //vector<ScannedLinePtr> horBallLines;
  //vector<ScannedLinePtr> verGoalLines;
  //vector<ScannedLinePtr> verBallLines;

  //! A vector of extracted field border points
  vector<Point> borderPoints;

  //! Average field height
  int avgHeight;

  //! Minimum best field height
  int minBestHeight;

  //! Field Extraction module object
  FieldExtractionPtr fieldExt;

  //! Ball Extraction module object
  //BallExtractionPtr ballExt;

  //! Goal Extraction module object
  //GoalExtractionPtr goalExt;
};
typedef boost::shared_ptr<RegionSegmentation> RegionSegmentationPtr;
