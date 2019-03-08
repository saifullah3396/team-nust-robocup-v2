/**
 * @file FeatureExtraction/ScannedRegion.h
 *
 * This file declares the struct ScannedRegion
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>
#include <opencv2/core/core.hpp>

/**
 * @struct ScannedRegion
 * @brief Holds information about the regions resulting from scanning 
 *   the image.
 */
struct ScannedRegion
{
  ScannedRegion(const ScannedRegion&) = default;
  ScannedRegion(ScannedRegion&&) = default;
  ScannedRegion& operator=(const ScannedRegion&) & = default;
  ScannedRegion& operator=(ScannedRegion&&) & = default;
  virtual ~ScannedRegion() {}

  /**
   * @brief ScannedRegion Constructor
   * @param pts Points defining the bounding box of the region
   */
  ScannedRegion(const std::vector<cv::Point>& pts);

  /**
   * @brief ScannedRegion Constructor
   * @param rect Rect defining the bounding box of the region
   */
  ScannedRegion(const cv::Rect& rect);

  /**
   * @brief ScannedRegion::join Performs a union with another region
   * @param other Other region
   */
  void join(const ScannedRegion& other);

  /**
   * @brief drawRegions Draws a vector of regions on the input image
   * @param img
   * @param regions
   * @param color
   */
  static void drawRegions(
    cv::Mat& img,
    const std::vector<boost::shared_ptr<ScannedRegion>>& regions,
    const cv::Scalar& color = cv::Scalar(0, 0, 0),
    const int& thickness = 1);

  /**
   * @brief draw Draws the region rect on the input image
   * @param img Image
   * @param color Color
   * @param thickness Thickness
   */
  void draw(
    cv::Mat& img,
    const cv::Scalar& color = cv::Scalar(0, 0, 0),
    const int& thickness = 1);

  /**
   * @brief filterRegionsBasedOnSize Removes the regions smaller
   *   than given size in x and y
   * @param regions Regions
   * @param x Size in x
   * @param y Size in y
   */
  static void filterRegionsBasedOnSize(
    std::vector<boost::shared_ptr<ScannedRegion>>& regions,
    const size_t& x,
    const size_t& y);

  /**
   * @brief linkRegions Links scanned regions based on x-y dist threshold
   * @param resultRegions Output regions
   * @param regions Input regions
   * @param xTol Tolerance in X
   * @param yTol Tolerance in Y
   */
  static void linkRegions(
    std::vector<boost::shared_ptr<ScannedRegion>>& resultRegions,
    std::vector<boost::shared_ptr<ScannedRegion>>& regions,
    const unsigned& xTol,
    const unsigned& yTol);

  cv::Rect rect; //! Rect defining the bounding box of the region
  cv::Point center; //! Region center
  cv::Point leftBase; //! Left base corner
  cv::Point rightBase; //! Right base corner
  bool assigned = {false}; //! Whether this region has already been assigned as best neighbor
  bool horizontal = {false}; //! Whether this region has width / height > 1
  boost::shared_ptr<ScannedRegion> pred; //! Preceding ScannedRegion in search
  boost::shared_ptr<ScannedRegion> bestNeighbor; //! Best neighbouring ScannedRegion
};

typedef boost::shared_ptr<ScannedRegion> ScannedRegionPtr;
typedef cv::vector<ScannedRegionPtr>::iterator SRIter; //! Iterator
