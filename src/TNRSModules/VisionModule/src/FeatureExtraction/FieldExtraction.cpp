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
  FeatureExtraction(visionModule),
  DebugBase("FieldExtraction", this)
{
  initDebugBase();
  int tempSendTime;
  int tempDrawFiltPoints;
  int tempDrawBorder;
  int tempDrawBorderLines;
  int tempDisplayInfo;
  int tempDisplayOutput;
  GET_CONFIG(
    "VisionConfig",
    (int, FieldExtraction.sendTime, tempSendTime),
    (int, FieldExtraction.drawFiltPoints, tempDrawFiltPoints),
    (int, FieldExtraction.drawBorder, tempDrawBorder),
    (int, FieldExtraction.drawBorderLines, tempDrawBorderLines),
    (int, FieldExtraction.displayInfo, tempDisplayInfo),
    (int, FieldExtraction.displayOutput, tempDisplayOutput),
    (int, FieldExtraction.maxRANSACIterations, maxRANSACIterations),
  );
  SET_DVAR(int, sendTime, tempSendTime);
  SET_DVAR(int, drawBorder, tempDrawBorder);
  SET_DVAR(int, drawBorderLines, tempDrawBorderLines);
  SET_DVAR(int, displayInfo, tempDisplayInfo);
  SET_DVAR(int, displayOutput, tempDisplayOutput);
  SET_DVAR(int, drawFiltPoints, tempDrawFiltPoints);
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
  if (GET_DVAR(int, displayOutput))
    VisionUtils::displayImage("FieldExtraction", bgrMat[toUType(activeCamera)]);
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
        if (borderPoints[i].y < avgHeight + avgToMin + 10) { // Tolerance of 10 pixels
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
  if (points.size() >= 6) {
    float bestNx = 0;
    float bestNy = 0;
    float bestD = 0;
    int maxSum = 0;
    int pointsCnt = 0;
    for (int i = 0; i < maxRANSACIterations; i++) {
      Point2d p1 =
        points[(int) (dist(rng) * points.size())];
      Point2d p2 =
        points[(int) (dist(rng) * points.size())];
      if (p1.x != p2.x || p1.y != p2.y) {
        if (p1.x > p2.x) {
          Point2d tmp = p1;
          p1 = p2;
          p2 = tmp;
        }
        int dx = p1.x - p2.x;
        int dy = p1.y - p2.y;
        float len = sqrt(dx * dx + dy * dy);
        if (len < 16) continue;
        float nx = -dy / len;
        float ny = dx / len;
        float d = nx * p1.x + ny * p1.y;
        int sum = 0;
        int cnt = 0;
        for (int j = 0; j < points.size(); ++j) {
          Point2d p = points[j];
          float dist = fabsf(nx * p.x + ny * p.y - d);
          if (dist < 4.f) {
            cnt++;
            sum += 1;
          } //else if (dist > 4.f && dist < 32) {
            //sum -= 1;
          //}
        }
        if (sum > maxSum) {
          maxSum = sum;
          pointsCnt = cnt;
          bestNx = nx;
          bestNy = ny;
          bestD = d;
        }
      }
    }

    // if enough points left, search for the best second line matching for the
    // rest of the border points with RANSAC algorithm
    if (pointsCnt >= 5) {
      vector < Point2d > pointsLeft;
      vector < Point2d > pointsDone;
      for (int j = 0; j < points.size(); ++j) {
        Point2d p = points[j];
        float dist = fabsf(bestNx * p.x + bestNy * p.y - bestD);
        if (dist >= 4.f) {
          pointsLeft.push_back(p);
        } else {
          pointsDone.push_back(p);
        }
      }
      float bestNx2 = 0;
      float bestNy2 = 0;
      float bestD2 = 0;
      int pointsCnt2 = 0;
      if (pointsLeft.size() >= 4) {
        int maxSum2 = 0;
        for (int i = 0; i < maxRANSACIterations; i++) {
          Point2d p1 = pointsLeft[(int) (dist(rng) * pointsLeft.size())];
          Point2d p2 = pointsLeft[(int) (dist(rng) * pointsLeft.size())];
          if (p1.x != p2.x || p1.y != p2.y) {
            if (p1.x > p2.x) {
              Point2d tmp = p1;
              p1 = p2;
              p2 = tmp;
            }
            int dx = p1.x - p2.x;
            int dy = p1.y - p2.y;
            float len = sqrtf(dx * dx + dy * dy);
            if (len < 16) continue;
            float nx = -dy / len;
            float ny = dx / len;
            float d = nx * p1.x + ny * p1.y;
            int sum = 0;
            int cnt = 0;
            for (int j = 0; j < pointsLeft.size(); ++j) {
              Point2d p = pointsLeft[j];
              float dist = nx * p.x + ny * p.y - d;
              if (fabsf(dist) < 4) {
                sum += 1;
                cnt++;
              }
              //} else if (dist > 4 && dist < 32) {
              //  sum -= 1;
              //}
            }
            for (int j = 0; j < pointsDone.size(); ++j) {
              Point2d p = pointsDone[j];
              float dist = nx * p.x + ny * p.y - d;
              if (dist > 4) {
                sum -= 2;
              }
            }
            if (sum > maxSum2) {
              maxSum2 = sum;
              pointsCnt2 = cnt;
              bestNx2 = nx;
              bestNy2 = ny;
              bestD2 = d;
            }
          }
        }
      }
      Vec4f best =
        Vec4f(
          0, bestD / bestNy, getImageWidth() - 1, (bestD - bestNx * (getImageWidth() - 1)) / bestNy);
      if (bestNy2 == 0) {
        borderLines.push_back(best);
      } else {
        Vec4f best2 =
          Vec4f(
            0, bestD2 / bestNy2, getImageWidth() - 1, (bestD2 - bestNx2 * (getImageWidth() - 1)) / bestNy2);
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
      //interpolate the field-border from one or two lines
      for (int x = 0; x < getImageWidth(); x++) {
        if (bestNy == 0) continue;
        int y1 = (int) ((bestD - bestNx * x) / bestNy);
        if (pointsCnt2 >= 4 && bestNy2 != 0) {
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
    fieldFound = true;
  }
}

void FieldExtraction::transformBorderLines() {
  auto tStart = high_resolution_clock::now();
  for (const auto& l : borderLines) {
    if (l[0] < 10 && l[2] < 10) continue;
    if (l[0] > getImageWidth() - 10 && l[2] > getImageWidth() - 10) continue;
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
