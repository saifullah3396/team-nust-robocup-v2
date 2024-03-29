/**
 * @file VisionModule/include/FeatureExtraction/FieldExtraction.h
 *
 * This file declares the class FieldExtraction
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author AbdulRehman
 * @date 22 Aug 2017
 */

#pragma once

#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"

class FittedLine;
typedef boost::shared_ptr<FittedLine> FittedLinePtr;
class RegionSegmentation;
typedef boost::shared_ptr<RegionSegmentation> RegionSegmentationPtr;

/**
 * @class FieldExtraction
 * @brief The class for extracting field from the input image.
 */
class FieldExtraction : public FeatureExtraction, public DebugBase
{
  INIT_DEBUG_BASE_(
    //! Option to send total module time.
    (int, sendTime, 0),
    //! Option to draw filtered points
    (int, drawFiltPoints, 0),
    //! Option to draw border on the output image.
    (int, drawBorder, 0),
    //! Option to draw border lines on the output image.
    (int, drawBorderLines, 0),
    //! Option to display detailed information about the results
    (int, displayInfo, 0),
    //! Option to display image output
    (int, displayOutput, 0),
  )

public:
  /**
   * Default constructor for this class.
   *
   * @param visionModule: Pointer to parent VisionModule.
   */
  FieldExtraction(VisionModule* visionModule);

  /**
   * Default destructor for this class.
   */
  ~FieldExtraction() {}

  /**
   * @brief processImage Derived from FeatureExtraction
   */
  void processImage();

  /**
   * Returns the vector of extracted field border points.
   *
   * @return vector<Point>
   */
  vector<Point>& getBorder() { return border; }

  /**
   * Returns the extracted field border lines.
   *
   * @return vector<Vec4i>
   */
  vector<Vec4f>& getBorderLines() { return borderLines; }

  /**
   * Returns the extracted field border lines in world.
   *
   * @return vector<FittedLinePtr>
   */
  vector<FittedLinePtr>& getBorderLinesWorld() { return borderLinesWorld; }

  /**
   * Returns the minimum field height.
   *
   * @return int
   */
  int& getFieldHeight() { return fieldHeight; }

  /**
   * Returns the rectangle of the extracted field.
   *
   * @return Rect
   */
  Rect& getFieldRect() { return fieldRect; }

  /**
   * @brief isFound Returns true if the field is extracted successfully
   * @return bool
   */
  bool& isFound() { return fieldFound; }

private:
  /**
   * @brief filterBorderPoints Filters field border points extracted in region segmentation
   * @return vector<Point>
   */
  vector<Point> filterBorderPoints();

  /**
   * @brief fitLines Fits lines on border points
   * @param points border points vector
   */
  void fitLines(const vector<Point>& points);

  /**
   * @brief makeFieldRect Makes a simple opencv rect for field
   */
  void makeFieldRect();

  /**
   * @brief transformBorderLines Transforms border lines to world frame
   */
  void transformBorderLines();

  /**
   * @brief drawResults Draws border and border lines on bgr image matrix
   */
  void drawResults();

  //! Field minimum height
  int fieldHeight;

  //! Whether the field is found or not
  bool fieldFound;

  //! Field border.
  vector<Point> border;

  //! Field border lines in image.
  vector<Vec4f> borderLines;

  //! Field border lines in world.
  vector<FittedLinePtr> borderLinesWorld;

  //! Rect for the extracted field
  Rect fieldRect;

  //! Total number of ransac iterations for fitting lines
  int maxRANSACIterations;

  //! Processing times
  float processTime;
  float pointsFilterTime;
  float fitLinesTime;
  float borderTransformTime;

  //! A pointer to region segmentation class
  RegionSegmentationPtr regionSeg;
};
typedef boost::shared_ptr<FieldExtraction> FieldExtractionPtr;
