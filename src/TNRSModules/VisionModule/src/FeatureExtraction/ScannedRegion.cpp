/**
 * @file FeatureExtraction/ScannedRegion.cpp
 *
 * This file implements the struct ScannedRegion
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#include "FeatureExtraction/ScannedRegion.h"
#include "Utils/include/VisionUtils.h"

ScannedRegion::ScannedRegion(const std::vector<cv::Point>& pts) {
  rect = boundingRect(pts);
  int yBase = rect.y + rect.height;
  center =
    cv::Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
  leftBase = cv::Point(rect.x, yBase);
  rightBase = cv::Point(rect.x + rect.width, yBase);
}

ScannedRegion::ScannedRegion(const cv::Rect& rect) : rect(rect) {
  int yBase = rect.y + rect.height;
  center =
    cv::Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
  leftBase = cv::Point(rect.x, yBase);
  rightBase = cv::Point(rect.x + rect.width, yBase);
}

void ScannedRegion::drawRegions(
  cv::Mat& img,
  const std::vector<ScannedRegionPtr>& regions,
  const cv::Scalar& color,
  const int& thickness)
{
  for (const auto& region : regions)
    region->draw(img, color, thickness);
}

void ScannedRegion::draw(
  cv::Mat& img,
  const cv::Scalar& color,
  const int& thickness)
{
  cv::rectangle(img, rect, color, thickness);
}

void ScannedRegion::join(
  const ScannedRegion& other)
{
  rect = rect | other.rect;
}

void ScannedRegion::filterRegionsBasedOnSize(
  std::vector<ScannedRegionPtr>& regions,
  const size_t& x,
  const size_t& y)
{
  SRIter iter = regions.begin();
  //! Remove small regions
  while (iter != regions.end()) {
    if ((*iter)->rect.width > x && (*iter)->rect.height > y) ++iter;
    else iter = regions.erase(iter);
  }
}

void ScannedRegion::linkRegions(
  std::vector<ScannedRegionPtr>& linked,
  std::vector<ScannedRegionPtr>& regions,
  const unsigned& xTol,
  const unsigned& yTol)
{
  //! Remove small regions
  filterRegionsBasedOnSize(regions, 5, 5);

  //! Sort regions
  sort(regions.begin(), regions.end(), [](
    const ScannedRegionPtr& sr1,
    const ScannedRegionPtr& sr2)
  { return sr1->rect.x < sr2->rect.x;});

  //! Preceding regions
  ScannedRegionPtr rPred;
  for (const auto& sr : regions) {
    sr->bestNeighbor.reset();
    if (rPred)
      sr->pred = rPred;
    rPred = sr;
  }

  for (const auto& sr : regions) {
    auto neighbor = sr->pred;
    int minX = 1000;
    int minY = 1000;
    while (neighbor != 0) { //! If neighbor exists
      auto diff = neighbor->center - sr->center;
      diff -= cv::Point(sr->rect.width / 2 + neighbor->rect.width / 2,
                        sr->rect.height / 2 + neighbor->rect.height / 2);
      if (diff.x < xTol && diff.y < yTol) { //! < 0 means overlap
        if (diff.x < minX && diff.y < minY) {
          minX = diff.x;
          minY = diff.y;
          sr->bestNeighbor = neighbor;
          neighbor->assigned = true;
        } else {
          break;
        }
      }
      neighbor = neighbor->pred;
    }
  }

  reverse(regions.begin(), regions.end());
  for (const auto& sr : regions) {
    if (sr->assigned) continue; //! Not a chain parent
    auto neighbor = sr->bestNeighbor;
    while (neighbor != 0) {
      sr->join(*neighbor);
      neighbor = neighbor->bestNeighbor;
    }
    int yBase = sr->rect.y + sr->rect.height;
    sr->center =
      cv::Point(sr->rect.x + sr->rect.width / 2, sr->rect.y + sr->rect.height / 2);
    sr->leftBase = cv::Point(sr->rect.x, yBase);
    sr->rightBase = cv::Point(sr->rect.x + sr->rect.width, yBase);
    linked.push_back(sr);
  }
}
