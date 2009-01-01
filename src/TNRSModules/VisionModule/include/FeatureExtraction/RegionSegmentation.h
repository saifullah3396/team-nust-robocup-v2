/**
 * @file VisionModule/include/FeatureExtraction/RegionSegmentation.h
 *
 * This file declares the class for segmenting the image into different
 * regions for further processing.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author AbdulRehman
 * @date 5 Mar 2018
 */

#pragma once

#include "TNRSBase/include/DebugBase.h"
#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"

struct Scan;
class FieldExtraction;
typedef boost::shared_ptr<FieldExtraction> FieldExtractionPtr;

enum class ScanTypes : unsigned int {
  field,
  robot,
  ball,
  oppJersey,
  ourJersey,
  lines,
  goal,
  count
};

/**
 * @class RegionSegmentation
 * @brief The class for extracting field from the input image.
 */
class RegionSegmentation : public FeatureExtraction, public DebugBase
{
  INIT_DEBUG_BASE_(
    ///< Option to send total module time
    (int, sendTime, 0),
    ///< Option to draw vertical lines
    (int, drawVerticalLines, 0),
    ///< Option to draw horizontal lines
    (int, drawHorizontalLines, 0),
    ///< Option to draw points
    (int, drawPoints, 0),
    ///< Option to display detailed info on results of this module
    (int, displayInfo, 0),
    ///< Option to display image output
    (int, displayOutput, 0),
  );
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
  ~RegionSegmentation();

  /**
   * @brief processImage Derived from FeatureExtraction
   */
  void  processImage();

  /**
   * @brief reset Clears up the previously extracted features
   */
  void reset();

  /**
   * @brief setScanSettings Sets which scanners should be used based on
   *   current image
   */
  void setScanSettings();

  /**
   * @brief setFieldExtraction Sets the pointer to field extraction module
   * @param fieldExt pointer to field extraction module
   */
  void setFieldExtraction(const boost::shared_ptr<FieldExtraction>& fieldExt)
  {
    this->fieldExt = fieldExt;
  }

  /**
   * @brief getBorderPoints Returns the extracted field border points
   * @return vector<cv::Point>
   */
  const vector<Point>& getBorderPoints() const { return borderPoints; }

  /**
   * @brief getFieldAvgHeight Returns the field average height
   * @return int
   */
  const int& getFieldAvgHeight() const { return avgHeight; }

  /**
   * @brief getFieldMinBestHeight Returns the field mininimum best height
   * @return int
   */
  const int& getFieldMinBestHeight() const { return minBestHeight; }

  Scan* getHorizontalScan(const ScanTypes& type);
  Scan* getVerticalScan(const ScanTypes& type);

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

  FieldExtractionPtr fieldExt; ///< Field Extraction module object
  int vScanLimitIdx; ///< Vertical scan limiting image index
  int hScanLimitIdx; ///< Horizontal scan limiting image index
  double horizontalScanTime = {0.0}; ///< Time taken by horizontal scan
  double verticalScanTime = {0.0}; ///< Time taken by vertical scan
  vector<Point> borderPoints; ///< A vector of extracted field border points
  int avgHeight; ///< Average field height
  int minBestHeight; ///< Minimum best field height
  vector<Scan*> vScans; ///< Horizontal scanners
  vector<Scan*> hScans; ///< Vertical scanners
  vector<int> hMinScanStepLow; ///< Minimum step size for direction parallel to scan
  vector<int> vMinScanStepLow; ///< Minimum step size for direction perpendicular to scan
  vector<int> hScanStepHigh; ///< Step size for direction perpendicular to scan
  vector<int> vScanStepHigh; ///< Step size for direction perpendicular to scan
  vector<vector<int> > hScanStepSizes; ///< Step size for direction parallel to scan
  vector<vector<int> > vScanStepSizes; ///< Step size for direction parallel to scan
};
typedef boost::shared_ptr<RegionSegmentation> RegionSegmentationPtr;
