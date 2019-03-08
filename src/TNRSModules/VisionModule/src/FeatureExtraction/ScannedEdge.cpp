/**
 * @file FeatureExtraction/ScannedEdge.cpp
 *
 * This file implements the struct ScannedEdge
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#include "FeatureExtraction/ScannedEdge.h"
#include "Utils/include/VisionUtils.h"

/**
 * @brief drawEdges Draws a vector of edges on the input image
 * @param img
 * @param edges
 * @param color
 */
void ScannedEdge::drawEdges(
  cv::Mat& img,
  const std::vector<boost::shared_ptr<ScannedEdge>>& edges,
  const cv::Scalar& color)
{
  for (const auto& edge : edges)
    edge->draw(img, color);
}

/**
 * @brief draw Draws the edge rect on the input image
 * @param img Image
 * @param color Color
 * @param thickness Thickness
 */
void ScannedEdge::draw(
  cv::Mat& img,
  const cv::Scalar& color)
{
  VisionUtils::drawPoint(this->pI, img, color);
}
