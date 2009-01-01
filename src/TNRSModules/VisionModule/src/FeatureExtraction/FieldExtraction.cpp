/**
 * @file VisionModule/src/FeatureExtraction/FieldExtraction.cpp
 *
 * This file implements the class for field extraction from the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author AbdulRehman
 * @date 22 Aug 2017
 */

#include <Eigen/Dense>
#include "VisionModule/include/CameraTransform.h"
#include "VisionModule/include/ColorHandler.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/RegionSegmentation.h"
#include "VisionModule/include/FeatureExtraction/FittedLine.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/DataUtils.h"

FieldExtraction::FieldExtraction(VisionModule* visionModule) :
  FeatureExtraction(visionModule, "FieldExtraction"),
  DebugBase("FieldExtraction", this)
{
  initDebugBase();
  GET_DEBUG_CONFIG(
    VisionConfig,
    FieldExtraction,
    (int, sendTime),
    (int, drawFiltPoints),
    (int, drawBorder),
    (int, drawBorderLines),
    (int, displayInfo),
    (int, displayOutput),
  );

  GET_CLASS_CONFIG(
    VisionConfig,
    FieldExtraction,
    maxRANSACIterations,
    minRANSACPoints,
    minMatchedRANSACPoints,
    minRANSACLineLength,
    minRANSACPointDist,
    borderPointsImageHeightTol,
  );
  regionSeg = GET_FEATURE_EXT_CLASS(RegionSegmentation, FeatureExtractionIds::segmentation);
  fieldHeight = 0;
  fieldFound = false;

  ///< Initializing processing times
  processTime = 0.0;
  pointsFilterTime = 0.0;
  fitLinesTime = 0.0;
  borderTransformTime = 0.0;
}

void FieldExtraction::processImage()
{
  ///< If current image is not the top image,
  ///< the border points and border lines will stay the same as for
  ///< the previous run carried out with top image
  if (activeCamera != CameraId::headTop)
    return;
  auto tStart = high_resolution_clock::now();
  auto filteredPoints = filterBorderPoints();
  fitLines(filteredPoints);
  makeFieldRect();
  transformBorderLines();
  drawResults();
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  processTime = timeSpan.count();
  if (GET_DVAR(int, displayOutput)) {
    VisionUtils::displayImage(name, bgrMat[toUType(activeCamera)]);
    //waitKey(0);
  }
  if (GET_DVAR(int, displayInfo)) {
    LOG_INFO("FieldExtraction Results:");
    LOG_INFO("Time taken by filtering points: " << pointsFilterTime);
    LOG_INFO("Time taken by fitting lines on points: " << fitLinesTime);
    LOG_INFO("Time taken by transforming border lines " << borderTransformTime);
    LOG_INFO("Time taken by overall processing: " << processTime);
    LOG_INFO("Filtered points: " << filteredPoints.size());
  }
}

vector<Point> FieldExtraction::filterBorderPoints()
{
  auto tStart = high_resolution_clock::now();
  auto borderPoints = regionSeg->getBorderPoints();
  vector<Point> filtered;
  if (!borderPoints.empty()) {
    auto avgHeight = regionSeg->getFieldAvgHeight();
    auto minBestHeight = regionSeg->getFieldMinBestHeight();
    ///< Sort extracted field points
    sort(
      borderPoints.begin(),
      borderPoints.end(),
      [](const Point& p1, const Point& p2) {
        return (p1.x < p2.x) ||
        ((p1.x == p2.x) && (p1.y < p2.y));
      });

    float avgToMin = abs(minBestHeight - avgHeight);
    float newAvgY = 0.f;
    float newMaxY = 0.f;
    ///< Filter out points by getting points lying on minimum y and same x
    vector<Point> filterBelow;
    filterBelow.push_back(borderPoints[0]);
    for (size_t i = 1; i < borderPoints.size(); ++i) {
      if (filterBelow.back().x == borderPoints[i].x) {
        continue;
      } else {
        ///< Add points lying avgToMin pixels above the avg height
        if (borderPoints[i].y < avgHeight + avgToMin + borderPointsImageHeightTol) {
          filterBelow.push_back(borderPoints[i]);
          newAvgY += borderPoints[i].y;
          newMaxY = borderPoints[i].y > newMaxY ? borderPoints[i].y : newMaxY;
        }
      }
    }
    newAvgY /= filterBelow.size();

    ///< Filter out points that lie way above the possible field
    for (size_t i = 0; i < filterBelow.size(); ++i) {
      //VisionUtils::drawPoint(filterBelow[i], bgrMat[toUType(activeCamera)], Scalar(0,255,255));
      if (filterBelow[i].y > newAvgY - 2 * abs(newMaxY - newAvgY)) {
        filtered.push_back(filterBelow[i]);
        //VisionUtils::drawPoint(filterBelow[i], bgrMat[toUType(activeCamera)], Scalar(255,0,255));
      }
    }
  }
  if (GET_DVAR(int, drawFiltPoints)) {
    for (size_t i = 0; i < filtered.size(); ++i) {
      VisionUtils::drawPoint(filtered[i], bgrMat[toUType(activeCamera)], Scalar(255, 0, 0));
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  pointsFilterTime = timeSpan.count();
  return filtered;
}

void FieldExtraction::fitLines(const vector<Point>& points)
{
  auto tStart = high_resolution_clock::now();
  fieldHeight = getImageHeight();
  //vector<Point> maskPoints;
  border.clear();
  borderLines.clear();
  borderLinesWorld.clear();
  fieldFound = false;
  mt19937 rng;
  uniform_real_distribution<> dist { 0, 1 };
  if (points.size() >= minRANSACPoints) {
    double bestNx = 0.0;
    double bestNy = 0.0;
    double bestD = 0.0;
    auto maxCnt = 0;
    for (int i = 0; i < maxRANSACIterations; i++) {
      Point2d p1, p2;
      do {
        p1 =
          points[(int) (dist(rng) * points.size())];
        p2 =
          points[(int) (dist(rng) * points.size())];
      } while(p1.x == p2.x && p1.y == p2.y);
      if (p1.x != p2.x || p1.y != p2.y) {
        if (p1.x > p2.x) {
          std::swap(p1, p2);
        }
        Point2d diff = p1 - p2;
        double len = cv::norm(diff);
        if (len < minRANSACLineLength) continue;
        double nx = -diff.y / len;
        double ny = diff.x / len;
        double d = nx * p1.x + ny * p1.y;
        auto cnt = 0;
        for (const auto& p : points) {
          auto dist = fabsf(nx * p.x + ny * p.y - d);
          if (dist < minRANSACPointDist) {
            cnt++;
            //sum += 1;
          } //else if (dist > 4.f && dist < 32) {
            //sum -= 1;
          //}
        }
        if (cnt > maxCnt) {
          maxCnt = cnt;
          bestNx = nx;
          bestNy = ny;
          bestD = d;
        }
      }
    }

    // if enough points left, search for the best second line matching for the
    // rest of the border points with RANSAC algorithm
    if (maxCnt >= minMatchedRANSACPoints) {
      auto best =
        Vec4f(
          0, bestD / bestNy,
          getImageWidth() - 1,
          (bestD - bestNx * (getImageWidth() - 1)) / bestNy);
      //line(bgrMat[toUType(activeCamera)],
//        Point(best[0],best[1]),
//        Point(best[2],best[3]),
//        Scalar(255,255,0),
//        1
//      );
      vector<Point2d> pointsLeft;
      vector<Point2d> pointsDone;
      for (const auto& p : points) {
        auto dist = fabsf(bestNx * p.x + bestNy * p.y - bestD);
        if (dist <= minRANSACPointDist) pointsDone.push_back(p);
        else pointsLeft.push_back(p);
      }
      //VisionUtils::drawPoints(pointsDone, bgrMat[toUType(activeCamera)], Scalar(255,0,0));
      //VisionUtils::drawPoints(pointsLeft, bgrMat[toUType(activeCamera)], Scalar(0,255,0));
      //VisionUtils::displayImage("bgrMat", bgrMat[toUType(activeCamera)]);
      //waitKey(0);
      double bestNx2 = 0.0;
      double bestNy2 = 0.0;
      double bestD2 = 0.0;
      auto maxCnt2 = 0;
      if (pointsLeft.size() >= minRANSACPoints) {
        for (int i = 0; i < maxRANSACIterations; i++) {
          Point2d p1, p2;
          do {
            p1 =
              pointsLeft[(int) (dist(rng) * pointsLeft.size())];
            p2 =
              pointsLeft[(int) (dist(rng) * pointsLeft.size())];
          } while(p1.x == p2.x && p1.y == p2.y);
          if (p1.x != p2.x || p1.y != p2.y) {
            if (p1.x > p2.x) {
              std::swap(p1, p2);
            }
            Point2d diff = p1 - p2;
            double len = cv::norm(diff);
            if (len < minRANSACLineLength) continue;
            double nx = -diff.y / len;
            double ny = diff.x / len;
            double d = nx * p1.x + ny * p1.y;
            auto cnt = 0;
            for (const auto& p : pointsLeft) {
              double dist = nx * p.x + ny * p.y - d;
              if (fabsf(dist) < minRANSACPointDist) {
                cnt++;
              }// else if (dist > minRANSACPointDist && dist < 32) {
              // cnt -= 1;
              //}
            }
            for (const auto& p : pointsDone) {
              double dist = nx * p.x + ny * p.y - d;
              if (fabsf(dist) < minRANSACPointDist) {
                cnt -= 2;
              }
            }
            if (cnt > maxCnt2) {
              //maxSum2 = sum;
              maxCnt2 = cnt;
              bestNx2 = nx;
              bestNy2 = ny;
              bestD2 = d;
            }
          }
        }
      }
      if (bestNy2 == 0) {
        borderLines.push_back(best);
      } else {
        if (maxCnt2 >= minMatchedRANSACPoints) {
          auto best2 =
            Vec4f(
              0, bestD2 / bestNy2,
              getImageWidth() - 1,
              (bestD2 - bestNx2 * (getImageWidth() - 1)) / bestNy2);
          Point2f intersection;
          VisionUtils::findIntersection(best, best2, intersection);
          if (intersection.x > 0 && intersection.x < getImageWidth() &&
              intersection.y > 0 && intersection.y < getImageHeight()) {
            //VisionUtils::drawPoint(intersection, bgrMat[toUType(activeCamera)]);
            if (best[1] > best2[1]) {
              borderLines.push_back(
                Vec4f(best[0], best[1], intersection.x, intersection.y));
              borderLines.push_back(
                Vec4f(intersection.x, intersection.y, best2[2], best2[3]));
            } else {
              borderLines.push_back(
                Vec4f(best2[0], best2[1], intersection.x, intersection.y));
              borderLines.push_back(
                Vec4f(intersection.x, intersection.y, best[2], best[3]));
            }
          } else {
            borderLines.push_back(best);
          }
        }
      }
      //interpolate the field-border from one or two lines
      for (int x = 0; x < getImageWidth(); x++) {
        if (bestNy == 0) continue;
        int y1 = (int) ((bestD - bestNx * x) / bestNy);
        if (borderLines.size() >= 2) {
          int y2 = (int) ((bestD2 - bestNx2 * x) / bestNy2);
          if (y2 > y1) y1 = y2;
        }
        if (y1 < 0) y1 = 0;
        if (y1 >= getImageHeight() - 2) y1 = getImageHeight() - 2;
        border.push_back(y1);
        //maskPoints.push_back(Point(x, y1 - 10));
        fieldHeight = y1 < fieldHeight ? y1 : fieldHeight;
      }
    }
  }
  //maskPoints.insert(maskPoints.begin(), Point(0, getImageHeight()));
  //maskPoints.push_back(Point(getImageWidth(), getImageHeight()));
  //fillConvexPoly(mask, &maskPoints[0], (int) maskPoints.size(), 255, 8, 0);
  if (fieldHeight >= getImageHeight()) fieldHeight = 0;
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  fitLinesTime = timeSpan.count();
}

void FieldExtraction::makeFieldRect() {
  if (fieldHeight != 0) {
    fieldRect.x = 0;
    fieldRect.y = fieldHeight;
    fieldRect.width = getImageWidth();
    fieldRect.height = getImageHeight() - fieldHeight;
    ///< Four points are converted to create perspective transform matrix for faster computation later on
    Point2f f1, f2, f3, f4;
    cameraTransforms[toUType(activeCamera)]->imageToWorld(f1, Point2f(fieldRect.x, fieldRect.y), 0.f);
    cameraTransforms[toUType(activeCamera)]->imageToWorld(f2, Point2f(fieldRect.width, fieldRect.y), 0.f);
    cameraTransforms[toUType(activeCamera)]->imageToWorld(f3, Point2f(fieldRect.width, getImageHeight()), 0.f);
    cameraTransforms[toUType(activeCamera)]->imageToWorld(f4, Point2f(fieldRect.x, getImageHeight()), 0.f);
    if (borderLines.empty()) // If not line is found, make a straight horizontal line
      borderLines.push_back(Vec4f(0, 0, getImageWidth(), 0));
    fieldFound = true;
  }
}

void FieldExtraction::transformBorderLines() {
  auto tStart = high_resolution_clock::now();
  for (const auto& l : borderLines) {
    if (l[0] < 0 && l[2] < 0) continue;
    if (l[0] > getImageWidth() && l[2] > getImageWidth()) continue;
    Point2f wp1, wp2;
    cameraTransforms[toUType(activeCamera)]->imageToWorld(wp1, Point2f(l[0], l[1]), 0.f);
    cameraTransforms[toUType(activeCamera)]->imageToWorld(wp2, Point2f(l[2], l[3]), 0.f);
    auto diff = wp2 - wp1;
    float d = norm(diff);
    auto unit = Point2f(diff.x / d, diff.y / d);
    auto fl = boost::make_shared<FittedLine>(wp1, wp2);
    fl->unit = unit;
    fl->d = d;
    fl->diff = diff;
    fl->perp.x = -fl->diff.y / fl->d;
    fl->perp.y = fl->diff.x / fl->d;
    fl->perpDist = fl->perp.x * fl->p1.x + fl->perp.y * fl->p1.y;
    borderLinesWorld.push_back(fl);
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  borderTransformTime = timeSpan.count();
}

void FieldExtraction::drawResults()
{
  if (GET_DVAR(int, drawBorder)) {
    for (size_t x = 0; x < border.size(); ++x)
      VisionUtils::drawPoint(
        Point(x, border[x]),
        bgrMat[toUType(activeCamera)],
        Scalar(255, 0, 0));
  }
  if (GET_DVAR(int, drawBorderLines)) {
    for (size_t i = 0; i < borderLines.size(); ++i) {
      auto l = borderLines[i];
      line(bgrMat[toUType(activeCamera)],
        Point(l[0],l[1]),
        Point(l[2],l[3]),
        Scalar(255,255,0),
        2
      );
    }
  }
}
