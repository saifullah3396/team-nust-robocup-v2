/**
 * @file VisionModule/include/FeatureExtraction/ScannedEdge.h
 *
 * This file defines the struct ScannedEdge.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>
#include <opencv2/core/core.hpp>

/**
 * @struct ScannedEdge
 * @brief Holds information about the line edge points resulting from
 *   scanning the image.
 */
struct ScannedEdge
{
  ScannedEdge(const ScannedEdge&) = default;
  ScannedEdge(ScannedEdge&&) = default;
  ScannedEdge& operator=(const ScannedEdge&) & = default;
  ScannedEdge& operator=(ScannedEdge&&) & = default;
  virtual ~ScannedEdge() {}

  /**
   * @brief ScannedEdge Constructor
   * @param p: Point defining region edge in the image
   */
  ScannedEdge(const cv::Point& pI, const cv::Point2f& pW) :
    pI(pI), pW(pW)
  {
  }

  /**
   * @brief drawEdges Draws a vector of edges on the input image
   * @param img
   * @param edges
   * @param color
   */
  static void drawEdges(
    cv::Mat& img,
    const std::vector<boost::shared_ptr<ScannedEdge>>& edges,
    const cv::Scalar& color = cv::Scalar(0, 0, 0));

  /**
   * @brief draw Draws the edge rect on the input image
   * @param img Image
   * @param color Color
   */
  void draw(
    cv::Mat& img,
    const cv::Scalar& color = cv::Scalar(0, 0, 0));

  cv::Point pI; ///< Point defining region edge in the image
  cv::Point2f pW; ///< Point defining region edge in the world
  float angleW = {0.f}; ///< Angle of line in world
  bool searched = {false};
  boost::shared_ptr<ScannedEdge> pred; ///< Preceding ScannedEdge in search
  boost::shared_ptr<ScannedEdge> bestNeighbor; ///< Best neighbouring ScannedEdge
  boost::shared_ptr<ScannedEdge> bestTo; ///< The edge to which this is the best neighbor
};

typedef boost::shared_ptr<ScannedEdge> ScannedEdgePtr;

