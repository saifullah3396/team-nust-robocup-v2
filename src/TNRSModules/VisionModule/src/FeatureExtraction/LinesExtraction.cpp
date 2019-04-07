/**
 * @file VisionModule/src/FeatureExtraction/LinesExtraction.cpp
 *
 * This file declares the class for field corners extraction from
 * the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017
 */

#include <chrono>
#include "LocalizationModule/include/FieldLandmarkIds.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/AngleDefinitions.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/Landmark.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/TNRSLine.h"
#include "VisionModule/include/VisionModule.h"
#include "VisionModule/include/CameraTransform.h"
#include "VisionModule/include/ColorHandler.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/LinesExtraction.h"
#include "VisionModule/include/FeatureExtraction/RegionSegmentation.h"
#include "VisionModule/include/FeatureExtraction/RegionScanners.h"
#include "VisionModule/include/FeatureExtraction/RobotExtraction.h"
#include "VisionModule/include/FeatureExtraction/RobotRegion.h"
#include "VisionModule/include/FeatureExtraction/Circle.h"
#include "VisionModule/include/FeatureExtraction/FittedLine.h"
#include "VisionModule/include/FeatureExtraction/ScannedEdge.h"
#include "VisionModule/include/FeatureExtraction/ScannedRegion.h"

LinesExtraction::LinesExtraction(VisionModule* visionModule) :
  FeatureExtraction(visionModule),
  DebugBase("LinesExtraction", this)
{
  ///< Initialize debug variables
  initDebugBase();
  int tempSendTime;
  int tempDrawScannedEdges;
  int tempDrawBorderLines;
  int tempDrawWorldLines;
  int tempDrawFiltWorldLines;
  int tempDrawCircle;
  int tempDrawUnknownLandmarks;
  int tempDrawCorners;
  int tempDisplayInfo;
  int tempDisplayOutput;
  GET_CONFIG(
    "VisionConfig",
    (int, LinesExtraction.scanStepHighUpperCam, scanStepHighUpperCam),
    (int, LinesExtraction.scanStepHighLowerCam, scanStepHighLowerCam),
    (int, LinesExtraction.scanStepLowUpperCam, scanStepLowUpperCam),
    (int, LinesExtraction.scanStepLowLowerCam, scanStepLowLowerCam),
    (int, LinesExtraction.sendTime, tempSendTime),
    (int, LinesExtraction.drawScannedEdges, tempDrawScannedEdges),
    (int, LinesExtraction.drawBorderLines, tempDrawBorderLines),
    (int, LinesExtraction.drawWorldLines, tempDrawWorldLines),
    (int, LinesExtraction.drawFiltWorldLines, tempDrawFiltWorldLines),
    (int, LinesExtraction.drawCircle, tempDrawCircle),
    (int, LinesExtraction.drawCorners, tempDrawCorners),
    (int, LinesExtraction.displayInfo, tempDisplayInfo),
    (int, LinesExtraction.displayOutput, tempDisplayOutput),
    (int, LinesExtraction.drawUnknownLandmarks, tempDrawUnknownLandmarks),
  )
  SET_DVAR(int, sendTime, tempSendTime);
  SET_DVAR(int, drawScannedEdges, tempDrawScannedEdges);
  SET_DVAR(int, drawBorderLines, tempDrawBorderLines);
  SET_DVAR(int, drawWorldLines, tempDrawWorldLines);
  SET_DVAR(int, drawFiltWorldLines, tempDrawFiltWorldLines);
  SET_DVAR(int, drawCircle, tempDrawCircle);
  SET_DVAR(int, drawCorners, tempDrawCorners);
  SET_DVAR(int, drawUnknownLandmarks, tempDrawUnknownLandmarks);
  SET_DVAR(int, displayInfo, tempDisplayInfo);
  SET_DVAR(int, displayOutput, tempDisplayOutput);

  ///< Get other feature extraction classes
  fieldExt = GET_FEATURE_EXT_CLASS(FieldExtraction, FeatureExtractionIds::field);
  regionSeg = GET_FEATURE_EXT_CLASS(RegionSegmentation, FeatureExtractionIds::segmentation);
  robotExt = GET_FEATURE_EXT_CLASS(RobotExtraction, FeatureExtractionIds::robot );

  worldImage = Mat(Size(1000, 700), CV_8UC3, Scalar(0));
  int width = 1000;
  int height = 700;
  mapDrawing = Mat(Size(1000, 700), CV_8UC3, Scalar(0));
  mapDrawing = Scalar(0, 100, 0); //BGR
  int borderDiff = 50;
  rectangle(
    mapDrawing,
    Point(borderDiff, borderDiff),
    Point(width - borderDiff, height - borderDiff),
    Scalar(255, 255, 255),
    5);
  rectangle(
    mapDrawing,
    Point(borderDiff, height / 2 - 220 / 2),
    Point(borderDiff + 60, height / 2 + 220 / 2),
    Scalar(255, 255, 255),
    5);
  rectangle(
    mapDrawing,
    Point(width - borderDiff, height / 2 - 220 / 2),
    Point(width - borderDiff - 60, height / 2 + 220 / 2),
    Scalar(255, 255, 255),
    5);
  rectangle(
    mapDrawing,
    Point(borderDiff + 130 - 6, height / 2 + 3),
    Point(borderDiff + 130 + 6, height / 2 - 3),
    Scalar(255, 255, 255),
    -1);
  rectangle(
    mapDrawing,
    Point(borderDiff + 130 - 3, height / 2 + 6),
    Point(borderDiff + 130 + 3, height / 2 - 6),
    Scalar(255, 255, 255),
    -1);
  rectangle(
    mapDrawing,
    Point(width - borderDiff - 130 + 6, height / 2 + 3),
    Point(width - borderDiff - 130 - 6, height / 2 - 3),
    Scalar(255, 255, 255),
    -1);
  rectangle(
    mapDrawing,
    Point(width - borderDiff - 130 + 3, height / 2 + 6),
    Point(width - borderDiff - 130 - 3, height / 2 - 6),
    Scalar(255, 255, 255),
    -1);
  rectangle(
    mapDrawing,
    Point(width / 2 - 6, height / 2 + 3),
    Point(width / 2 + 6, height / 2 - 3),
    Scalar(255, 255, 255),
    -1);
  line(
    mapDrawing,
    Point(width / 2, 5),
    Point(width / 2, height - 5),
    Scalar(255, 255, 255),
    5);
  circle(
    mapDrawing,
    Point(width / 2, height / 2),
    75,
    Scalar(255, 255, 255),
    5);

  ///< Initializing processing times
  edgeScanTime = 0.f;
  fitLinesTime = 0.f;
  filterLinesTime = 0.f;
  findFeaturesTime = 0.f;
  findCircleTime = 0.f;
  addLandmarksTime = 0.f;
  processTime = 0.f;

  //activeCamera = static_cast<unsigned>(CameraId::headBottom);
}

Point LinesExtraction::worldToImage(const Point& point)
{
  auto robotPose2D = ROBOT_POSE_2D_IN(VisionModule);
  robotPose2D.x() = 2.5;
  robotPose2D.y() = -1.60;
  robotPose2D.theta() = Angle::DEG_45;
  auto pointT = robotPose2D.transform(point);
  return Point(pointT.x * 100 + 500, 350 - pointT.y * 100);
}

Point2f LinesExtraction::worldToImage(const Point2f& point)
{
  auto robotPose2D = ROBOT_POSE_2D_IN(VisionModule);
  robotPose2D.x() = 2.5;
  robotPose2D.y() = -1.60;
  robotPose2D.theta() = Angle::DEG_45;
  auto pointT = robotPose2D.transform(point);
  return Point2f(pointT.x * 100 + 500, 350 - pointT.y * 100);
}

void LinesExtraction::processImage()
{
  auto tStart = high_resolution_clock::now();
  if (fieldExt->isFound()) {
    if (GET_DVAR(int, displayOutput))
      worldImage = mapDrawing.clone();//Scalar(0);
    if (activeCamera == CameraId::headBottom) {
      Point2f tempP;
      cameraTransforms[toUType(activeCamera)]->imageToWorld(tempP, Point2f(0, 0), 0.f);
      cameraTransforms[toUType(activeCamera)]->imageToWorld(tempP, Point2f(getImageWidth(), 0), 0.f);
      cameraTransforms[toUType(activeCamera)]->imageToWorld(tempP, Point2f(getImageWidth(), getImageHeight()), 0.f);
      cameraTransforms[toUType(activeCamera)]->imageToWorld(tempP, Point2f(0, getImageHeight()), 0.f);
    }
    vector<vector<ScannedEdgePtr> > connectedEdges;
    if (scanForEdges(connectedEdges)) {
      vector<FittedLinePtr> worldLines;
      findLinesFromEdges(connectedEdges, worldLines);
      if (activeCamera != CameraId::headBottom) {
        vector<Point2f> circlePoints;
        filterLines(worldLines, circlePoints);
        findFeatures(worldLines);
        Circle circleOutput;
        if (findCircle(circleOutput, circlePoints)) {
          computeCircleLandmark(circleOutput, worldLines);
          if (GET_DVAR(int, drawCircle)) {
            circle(
              worldImage,
              worldToImage(circleOutput.center),
              circleOutput.radius * 100,
              Scalar(255, 0, 0),
              2
            );
          }
        }
      }
      addLineLandmarks(worldLines);
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  processTime = timeSpan.count();
  if (GET_DVAR(int, displayInfo)) {
    LOG_INFO("LinesExtraction Results:");
    LOG_INFO("Time taken by scanning for edges: " << edgeScanTime);
    LOG_INFO("Time taken by fitting lines to edges: " << fitLinesTime);
    LOG_INFO("Time taken by filtering lines: " << filterLinesTime);
    LOG_INFO("Time taken by finding features: " << findFeaturesTime);
    LOG_INFO("Time taken by finding circle: " << findCircleTime);
    LOG_INFO("Time taken by adding landmarks: " << addLandmarksTime);
    LOG_INFO("Time taken by overall processing: " << processTime);
  }
  if (GET_DVAR(int, displayOutput)) {
    VisionUtils::displayImage("LinesExtraction Image", bgrMat[toUType(activeCamera)]);
    VisionUtils::displayImage("LinesExtraction World", worldImage);
  }
}

bool LinesExtraction::scanForEdges(
  vector<vector<ScannedEdgePtr> >& connectedEdges)
{
  auto tStart = high_resolution_clock::now();
  auto verImagePoints = static_cast<LinesScan*>(regionSeg->getVerticalScan(ScanTypes::lines))->edgePoints;
  auto horImagePoints = static_cast<LinesScan*>(regionSeg->getHorizontalScan(ScanTypes::lines))->edgePoints;
  auto border = fieldExt->getBorder();
  auto robotRegions = robotExt->getRobotRegions();
  if (border.empty()) {
    duration<double> timeSpan = high_resolution_clock::now() - tStart;
    edgeScanTime = timeSpan.count();
    return false;
  }
  auto iter = verImagePoints.begin();
  while (iter != verImagePoints.end()) {
    if ((*iter).y > border[(*iter).x]) {
      bool inRobotRegion = false;
      for (const auto& rr : robotRegions) {
        if (rr->sr->rect.contains((*iter))) {
          inRobotRegion = true;
          break;
        }
      }
      if (!inRobotRegion)
        ++iter;
      else
        iter = verImagePoints.erase(iter);
    } else {
      iter = verImagePoints.erase(iter);
    }
  }

  auto scanStepHigh = regionSeg->getVerticalScan(ScanTypes::lines)->highStep;
  vector<Point2f> verWorldPoints;
  vector<Point2f> horWorldPoints;
  vector<ScannedEdgePtr> verLineEdges;
  vector<ScannedEdgePtr> horLineEdges;
  if (!verImagePoints.empty()) {
    cameraTransforms[toUType(activeCamera)]->imageToWorld(
      verWorldPoints,
      verImagePoints,
      0.0);
    for (size_t i = 0; i < verWorldPoints.size(); ++i) {
      auto se =
        boost::make_shared<ScannedEdge>(
          verImagePoints[i], verWorldPoints[i]);
      verLineEdges.push_back(se);
    }
    findConnectedEdges(
      connectedEdges, verLineEdges, scanStepHigh * 2.5, scanStepHigh, true);
  }
  if (!horImagePoints.empty()) {
    cameraTransforms[toUType(activeCamera)]->imageToWorld(
      horWorldPoints,
      horImagePoints,
      0.0);
    for (size_t i = 0; i < horWorldPoints.size(); ++i) {
      auto se =
        boost::make_shared<ScannedEdge>(
          horImagePoints[i], horWorldPoints[i]);
      horLineEdges.push_back(se);
    }
    findConnectedEdges(
      connectedEdges, horLineEdges, scanStepHigh, scanStepHigh * 2.5, false);
  }
  if (GET_DVAR(int, drawScannedEdges)) {
    VisionUtils::drawPoints(horImagePoints, bgrMat[toUType(activeCamera)]);
    VisionUtils::drawPoints(verImagePoints, bgrMat[toUType(activeCamera)]);
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  edgeScanTime = timeSpan.count();
  if (connectedEdges.empty()) return false;
  return true;
}

void LinesExtraction::findConnectedEdges(
  vector<vector<ScannedEdgePtr> >& connectedEdges,
  vector<ScannedEdgePtr>& scannedEdges, const unsigned& xTol,
  const unsigned& yTol, const bool& verticalScan)
{
  typedef vector<ScannedEdgePtr>::iterator sEIter;
  sEIter iter = scannedEdges.begin();
  while (iter != scannedEdges.end()) {
    if (*iter) ++iter;
    else iter = scannedEdges.erase(iter);
  }

  if (verticalScan) {
    sort(scannedEdges.begin(), scannedEdges.end(), [](
      const ScannedEdgePtr& se1,
      const ScannedEdgePtr& se2)
    { return se1->pI.x < se2->pI.x;});
  } else {
    sort(scannedEdges.begin(), scannedEdges.end(), [](
      const ScannedEdgePtr& se1,
      const ScannedEdgePtr& se2)
    { return se1->pI.y < se2->pI.y;});
  }

  ScannedEdgePtr pred;
  for (const auto& se : scannedEdges) {
    se->angleW = 1000;
    se->bestNeighbor.reset();
    se->bestTo.reset();
    if (pred) se->pred = pred;
    pred = se;
  }

  //cout << "xTol: " << xTol << endl;
  //cout << "yTol: " << yTol << endl;
  auto maxX = xTol * xTol;
  auto maxY = yTol * yTol;
  auto maxDist = maxX + maxY;
  for (const auto& se : scannedEdges) {
    if (se->bestTo) continue;
    auto neighbor = se->pred;
    auto minDist = 1000;
    //se->draw(bgrMat[toUType(activeCamera)], Scalar(255, 0, 0));
    while (neighbor) {
      auto diffX = se->pI.x - neighbor->pI.x;
      diffX *= diffX;
      if (diffX >= maxX) { neighbor = neighbor->pred; continue; }
      auto diffY = se->pI.y - neighbor->pI.y;
      diffY *= diffY;
      if (diffY >= maxY) { neighbor = neighbor->pred; continue; }
      auto sqrdDist = diffX + diffY;
      if (sqrdDist < minDist && sqrdDist < maxDist)
      {
        se->bestNeighbor = neighbor;
        //cout << "bestNeighbor asigned..." << endl;
        //neighbor->draw(bgrMat[toUType(activeCamera)], Scalar(0, 0, 255));
      } else if (se->bestNeighbor) {
        break;
      }
      minDist = std::min(sqrdDist, minDist);
      neighbor = neighbor->pred;
      //imshow("bgr", bgrMat[toUType(activeCamera)]);
      //waitKey(0);
    }
    if (se->bestNeighbor) {
      se->angleW = atan2(
        se->bestNeighbor->pW.y - se->pW.y,
        se->bestNeighbor->pW.x - se->pW.x);
      se->bestNeighbor->bestTo = se;
    }
  }

  for (auto& se : scannedEdges) {
    if (!se->bestNeighbor) {
      if (se->bestTo)
        se->angleW = se->bestTo->angleW;
      else se.reset();
    }
  }
  /*for (const auto& se : scannedEdges) {
    if (se) {
    se->draw(bgrMat[toUType(activeCamera)], Scalar(255, 0, 0));
    if (se->bestNeighbor)
      se->bestNeighbor->draw(bgrMat[toUType(activeCamera)], Scalar(0, 0, 255));
    cout << "bestTo:" << se->bestTo << endl;
    imshow("bgr", bgrMat[toUType(activeCamera)]);
    waitKey(0);
    }
  }*/
  reverse(scannedEdges.begin(), scannedEdges.end());
  const int minChainLength = 3;
  for (const auto& se : scannedEdges) {
    if (se) {
      if (se->searched || se->bestTo) continue;
      vector<ScannedEdgePtr> groupedEdges;
      groupedEdges.push_back(se);
      auto neighbor = se->bestNeighbor;
      auto prevAngle = se->angleW;
      //se->draw(bgrMat[toUType(activeCamera)], Scalar(255,0,0));
      while (neighbor && !neighbor->searched) {
        neighbor->searched = true;
        //neighbor->draw(bgrMat[toUType(activeCamera)], Scalar(255,255,0));
        //imshow("bgr", bgrMat[toUType(activeCamera)]);
        //waitKey(0);
        //cout << "neighbor->angleW: " << neighbor->angleW * MathsUtils::RAD_TO_DEG<< endl;
        //cout << "prevAngle: " << prevAngle * MathsUtils::RAD_TO_DEG << endl;
        if (fabsf(neighbor->angleW - prevAngle) > Angle::DEG_60) {
          if (groupedEdges.size() >= minChainLength) {
            connectedEdges.push_back(groupedEdges);
            groupedEdges.clear();
          }
          //neighbor->draw(bgrMat[toUType(activeCamera)], Scalar(255,0,0));
        }
        groupedEdges.push_back(neighbor);
        prevAngle = neighbor->angleW;
        neighbor = neighbor->bestNeighbor;
        //imshow("bgr", bgrMat[toUType(activeCamera)]);
        //waitKey(0);
      }
      if (groupedEdges.size() >= minChainLength) {
        connectedEdges.push_back(groupedEdges);
      }
    }
  }
  /*
  Mat test = bgrMat[toUType(activeCamera)].clone();
  for (size_t i = 0; i < connectedEdges.size(); ++i) {
    cout << "i: " << i << endl;
    bgrMat[toUType(activeCamera)] = test.clone();
    ScannedEdge::drawEdges(bgrMat[toUType(activeCamera)], connectedEdges[i], Scalar(255, 0,0));
    imshow("bgr", bgrMat[toUType(activeCamera)]);
    waitKey(0);
  }*/
}

void LinesExtraction::findLinesFromEdges(
  const vector<vector<ScannedEdgePtr> >& connectedEdges,
  vector<FittedLinePtr>& lines)
{
  auto tStart = high_resolution_clock::now();
  mt19937 rng;
  uniform_real_distribution<> dist { 0, 1 };
  const int maxIters = 5;
  for (const auto& ce : connectedEdges) {
    float bestNx = 0;
    float bestNy = 0;
    float bestD = 0;
    int pointsCnt = 0;
    Point2f p1, p2;
    p1 = ce.front()->pW;
    p2 = ce.back()->pW;
    for (size_t n = 0; n < maxIters; ++n) {
      if (p1.x > p2.x) {
        auto tmp = p1;
        p1 = p2;
        p2 = tmp;
      }
      auto diff = p1 - p2;
      float len = norm(diff);
      float nx = -diff.y / len;
      float ny = diff.x / len;
      float perpDist = nx * p1.x + ny * p1.y;
      int cnt = 0;
      for (const auto& edge : ce) {
        auto& p = edge->pW;
        auto dist = fabsf(nx * p.x + ny * p.y - perpDist);
        if (dist < 0.05) cnt++;
      }
      if (cnt > pointsCnt) {
        pointsCnt = cnt;
        bestNx = nx;
        bestNy = ny;
        bestD = perpDist;
        if (pointsCnt >= 0.95 * ce.size())
          break;
      }
      if (ce.size() < 3)
        break;
      do {
        p1 =
          ce[(int) (dist(rng) * ce.size())]->pW;
        p2 =
          ce[(int) (dist(rng) * ce.size())]->pW;
      } while(p1.x == p2.x && p1.y == p2.y);
    }

    if (pointsCnt < 0.5 * ce.size()) {
      auto fl = boost::make_shared<FittedLine>();
      fl->circleLine = true;
      vector<Point2f> points;
      for (const auto& edge : ce)
        points.push_back(edge->pW);
      fl->points = boost::make_shared<vector<Point2f> >(points);
    } else {
      if (pointsCnt >= 3) {
        vector<Point2f> collinearPoints;
        for (const auto& edge : ce) {
          auto& p = edge->pW;
          float dist = fabsf(bestNx * p.x + bestNy * p.y - bestD);
          if (dist <= 0.1f) collinearPoints.push_back(p);
        }
        Point2f minP = collinearPoints[0];
        Point2f maxP = collinearPoints.back();
        auto fl = boost::make_shared<FittedLine> ();
        fl->p1 = minP;
        fl->p2 = maxP;
        fl->diff = maxP - minP;
        fl->d = norm(fl->diff);
        fl->perp = Point2f(bestNx, bestNy);
        fl->perpDist = bestD;
        fl->unit = Point2f(fl->diff.x / fl->d, fl->diff.y / fl->d);
        fl->points = boost::make_shared < vector<Point2f> > (collinearPoints);
        lines.push_back(fl);
        /*if (GET_DVAR(int, drawWorldLines)) {
          Point2f p11 = worldToImage(fl->p1);
          Point2f p12 = worldToImage(fl->p2);
          line(worldImage, p11, p12, Scalar(0,255,255), 2);
        }*/
      }
    }
  }
  /*for (const auto& fl : lines) {
    cout << "diff:" << fl->diff << endl;
    cout << "d:" << fl->d << endl;
    cout << "perp:" << fl->perp << endl;
    cout << "perpDist:" << fl->perpDist << endl;
    cout << "unit:" << fl->unit << endl;
    cout << "points:" << fl->points->size() << endl;
    Point2f p11 = worldToImage(fl->p1);
    Point2f p12 = worldToImage(fl->p2);
    line(worldImage, p11, p12, Scalar(0,255,255), 2);
    imshow("worldImge",  worldImage);
    waitKey(0);
  }*/
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  fitLinesTime = timeSpan.count();
}

void LinesExtraction::filterLines(
  vector<FittedLinePtr>& worldLines,
  vector<Point2f>& circlePoints)
{
  auto tStart = high_resolution_clock::now();
  auto borderLines = fieldExt->getBorderLinesWorld();
  FittedLinePtr borderLine;
  auto maxLen = 0;
  for (const auto& line : borderLines) {
    if (line->d > maxLen) {
      borderLine = line;
      maxLen = line->d;
    }
  }
  if(GET_DVAR(int, drawBorderLines)) {
    auto p11 = worldToImage(borderLine->p1);
    auto p12 = worldToImage(borderLine->p2);
    line(worldImage, p11, p12, Scalar(0,255,255), 1);
  }
  auto angleL = atan2(borderLine->unit.y, borderLine->unit.x);
  angleL = angleL > M_PI_2 ? M_PI - angleL : angleL;
  angleL = angleL < 0 ? M_PI + angleL : angleL;
  const auto minDistFromBorder = 0.1f;

  ///< Filter lines based on relationship with border line
  for (auto& wl : worldLines) {
    if (!wl) continue;
    ///< Find perpendicular distance from border line
    auto dist = borderLine->findShortest(*wl);
    ///< Reject line if it is too close to border perpendicularly
    if (fabsf(dist) < minDistFromBorder) {
      //cout << "Too close to border... rejected" << endl;
      wl.reset();
      continue;
    }

    ///< Find intersection with border line
    Point2f inter;
    if (wl->findIntersection(*borderLine, inter)) {
      auto diff1 = norm(inter - wl->p1);
      auto diff2 = norm(inter - wl->p2);
      auto ratio = diff1 / wl->d + diff2 / wl->d;
      ///< Reject line if it is intersecting with the border
      if (diff1 < 0.35 || diff2 < 0.35 || (ratio > 0.9 && ratio < 1.1)) {
        //cout << "Intersecting with border... rejected" << endl;
        wl.reset();
        continue;
      }
    }

    ///< Find line angle and compare with borderline angle
    wl->angle = atan2(wl->unit.y, wl->unit.x);
    wl->angle = wl->angle < 0 ? M_PI + wl->angle : wl->angle;
    //cout << "wl->angle: " << wl->angle * 180 / 3.14 << endl;
    auto angleD = abs(wl->angle - angleL);
    ///< 15-75 degrees the difference between border line angle and the estimated world line angle
    ///< Might have to increase the tolerance in real world
    if (angleD >= Angle::DEG_15 && angleD <= Angle::DEG_75) {
      ///< If line is far from border, add it as circle line
      if (wl->d < 1.0f && fabsf(dist) > 2.5f) {
        auto points = *(wl->points);
        //line(worldImage, worldToImage(wl->p1),  worldToImage(wl->p2), Scalar(255,0,255), 1);
        //cout << "circleLine: "<< endl;
        circlePoints.insert(circlePoints.begin(), points.begin(), points.end());
        wl->circleLine = true;
      } else { ///< Reject line if it is not far yet too far in angles from border
        wl.reset();
        //cout << "Not parallel or perpendicular to border... rejected" << endl;
        continue;
      }
    }
  }

  const auto minLineFilterDist = 0.1f;
  ///< Filter lines by removing overlapping parts and joining them into one line
  for (auto& wl1 : worldLines) {
    if (!wl1 || wl1->circleLine) continue;
    vector<Point2f> collinearPoints;
    for (auto& wl2 : worldLines) {
      if (!wl2 || wl2 == wl1 || wl2->circleLine) continue;
      ///< Find the angle between the lines
      if (fabsf(wl2->angle - wl1->angle) < Angle::DEG_15) {
        ///< Find the perpendicular distance between the lines
        auto dist = wl2->findShortest(*wl1);
        if (fabsf(dist) < minLineFilterDist) {
          collinearPoints.push_back(wl1->p1);
          collinearPoints.push_back(wl1->p2);
          collinearPoints.push_back(wl2->p1);
          collinearPoints.push_back(wl2->p2);
          (*wl1->points).insert((*wl1->points).begin(), (*wl2->points).begin(), (*wl2->points).end());
          wl2.reset();
        }
      }
    }
    if (!collinearPoints.empty()) {
      Point2f p1, p2;
      float maxDist = 0;
      for (int j = 0; j < collinearPoints.size(); ++j) {
        for (int k = j; k < collinearPoints.size(); ++k) {
          float dist = norm(collinearPoints[j] - collinearPoints[k]);
          if (dist > maxDist) {
            maxDist = dist;
            p1 = collinearPoints[j];
            p2 = collinearPoints[k];
          }
        }
      }
      wl1->p1 = p1;
      wl1->p2 = p2;
      wl1->diff = p2 - p1;
      wl1->d = norm(wl1->diff);
      wl1->unit = Point2f(wl1->diff.x / wl1->d, wl1->diff.y / wl1->d);
      wl1->perp = Point2f(-wl1->diff.y / wl1->d, wl1->diff.x / wl1->d);
      wl1->perpDist = wl1->perp.x * wl1->p1.x + wl1->perp.y * wl1->p1.y;
      wl1->angle = atan2(wl1->unit.y, wl1->unit.x);
      wl1->angle = wl1->angle < 0 ? M_PI + wl1->angle : wl1->angle;
    }
    if (GET_DVAR(int, drawFiltWorldLines)) {
      auto p1 = worldToImage(wl1->p1);
      auto p2 = worldToImage(wl1->p2);
      line(worldImage, p1, p2, Scalar(0, 0, 255), 1);
    }
    //imshow("worldImage", worldImage);
    //waitKey(0);
  }

  /**
   * Once again remove new lines that are far from border
   * and add them as parts of circle
   */
  for (auto& wl : worldLines) {
    if (!wl || wl->circleLine) continue;
    auto dist = borderLine->findShortest(*wl);
    if (wl->d < 0.65f && fabsf(dist) > 2.5f) {
      wl->circleLine = true;
    }
  }

  /*
   * Filtering again checking if two lines are perpendicular
   * for (size_t i = 0; i < worldLines.size(); ++i) {
    if (!worldLines[i]) continue;
    if (!worldLines[i]->circleLine) continue;
    auto wl = worldLines[i];
    //Point2f p11 = worldToImage(wl->p1);
    //Point2f p12 = worldToImage(wl->p2);
    //line(worldImage, p11, p12, Scalar(0,255,0), 2);
    for (size_t j = i; j < worldLines.size(); ++j) {
      if (!worldLines[j]) continue;
      if (!worldLines[j]->circleLine) continue;
      if (i != j) {
        auto wl2 = worldLines[j];
        //Point2f p11 = worldToImage(wl2->p1);
        //Point2f p12 = worldToImage(wl2->p2);
        //line(worldImage, p11, p12, Scalar(255,0,0), 2);
        Point2f inter;
        if (wl->findIntersection(*wl2, inter)) {
          float diff1 = norm(inter - wl->p1);
          float diff2 = norm(inter - wl->p2);
          float ratio = diff1 / wl->d + diff2 / wl->d;
          if (diff1 < 0.35 || diff2 < 0.35 || (ratio > 0.9 && ratio < 1.1)) {
            float angleD = fabsf(wl->angle - wl2->angle);
            angleD = angleD > M_PI_2 ? M_PI - angleD : angleD;
            //cout << "angleD : " << angleD * 180 / M_PI  << endl;
            //cout << "wl->angle : " << wl->angle * 180 / M_PI << endl;
            //cout << "wl2->angle : " << wl2->angle * 180 / M_PI  << endl;
            if (angleD >= 75 * M_PI / 180 ||
                angleD <= 15 * M_PI / 180)
            {
              //cout << "circleLine removed" << endl;
              wl->circleLine = false;
              wl2->circleLine = false;
            }
          }
        }
      }
    }
  }*/

  size_t nCircleLines = 0;
  for (auto& wl : worldLines) {
    if (wl && wl->circleLine) {
      auto& points = wl->points;
      circlePoints.insert(circlePoints.begin(), (*points).begin(), (*points).end());
      nCircleLines++;
    }
  }

  //if (nCircleLines < 2) {
  //    for (auto& wl : worldLines) {
  //      if (wl && wl->circleLine) wl->circleLine = false;
  //    }
  //  }

  //for (size_t i = 0; i < circlePoints.size(); ++i) {
  //  VisionUtils::drawPoint(worldToImage(circlePoints[i]), worldImage);
  //}
  //imshow("worldImage", worldImage);
  //waitKey(0);

  FlIter iter = worldLines.begin();
  while (iter != worldLines.end()) {
    if (*iter) ++iter;
    else iter = worldLines.erase(iter);
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  filterLinesTime = timeSpan.count();
}

void LinesExtraction::findFeatures(vector<FittedLinePtr>& worldLines)
{
  //cout << "Finding features..." << endl;
  auto tStart = high_resolution_clock::now();
  //for (const auto& wl : worldLines) {
  //  if (!wl || wl->circleLine) continue;
  //  Point2f p1 = worldToImage(wl->p1);
  //  Point2f p2 = worldToImage(wl->p2);
    //line(worldImage, p1, p2, Scalar(255,0,0), 2);
    //imshow("worldImage test", worldImage);
    //waitKey(0);
  //}
  static auto padding = 20;
  static auto interImageBox = cv::Rect(padding, padding, getImageWidth() - padding * 2, getImageHeight() - padding * 2);
  for (size_t i = 0; i < worldLines.size(); ++i) {
    const auto& wl1 = worldLines[i];
    if (!wl1 || wl1->circleLine) continue;
    for (size_t j = i; j < worldLines.size(); ++j) {
      const auto& wl2 = worldLines[j];
      if (!wl2 || wl2 == wl1 || wl2->circleLine) continue;
      Point2f inter;
      if (wl1->findIntersection(*wl2, inter)) {
        Point2f imageP;
        cameraTransforms[toUType(activeCamera)]->worldToImage(
          Point3f(inter.x, inter.y, 0.f), imageP);
        if (!interImageBox.contains(imageP))
          continue;
      } else {
        continue;
      }
      bool intersection1 = false;
      bool intersection2 = false;
      bool center1 = false;
      bool center2 = false;
      auto diff1 = inter - wl1->p1;
      auto d1 = norm(diff1);
      auto unit1 = Point2f(diff1.x / d1, diff1.y / d1);
      if (unit1.x / wl1->unit.x < 0 && d1 < 0.2) {
        intersection1 = true;
      } else if (d1 - wl1->d < 0.3) { ///< Within 30 centimeters ahead of p2
        auto r = d1 / wl1->d;
        if (r > 0.05 && r < 0.95)
          center1 = true;
        intersection1 = true;
      }
      if (intersection1) {
        auto diff2 = inter - wl2->p1;
        auto d2 = norm(diff2);
        auto unit2 = Point2f(diff2.x / d2, diff2.y / d2);
        if (unit2.x / wl2->unit.x < 0 && d2 < 0.2) {
          intersection2 = true;
        } else if (d2 - wl2->d < 0.3) { ///< Within 30 centimeters ahead of p2
          auto r = d2 / wl2->d;
          if (r > 0.05 && r < 0.95)
            center2 = true;
          intersection2 = true;
        }
        // if (d1 / d2 > 3.f || d1 > d2 < 1/3.f)
        //  continue;
        if (intersection2) {
          float angle = acos(wl1->unit.dot(wl2->unit));
          angle = angle > M_PI_2 ? M_PI - angle : angle;
          if (angle > 1.308333333) // 75 degrees
          {
            if (center1) {
              if (center2) {
                if (GET_DVAR(int, drawCorners))
                  putText(worldImage, "X", worldToImage(inter), CV_FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, CV_AA);
                //VisionUtils::drawPoint(Point(inter.x * 100 + 500, 350 - 100 * inter.y), worldImage, Scalar(0,0,255));
              } else {
                // Line 2 is perpendicular
                // T joint
                float norm1, norm2;
                Point2f dist1, dist2;
                dist1 = wl2->p1 - inter;
                dist2 = wl2->p2 - inter;
                norm1 = norm(dist1);
                norm2 = norm(dist2);
                Point2f unitT =
                  norm1 > norm2 ?
                    Point2f(dist1.x / norm1, dist1.y / norm1) :
                    Point2f(dist2.x / norm2, dist2.y / norm2);
                computeLandmark(inter, unitT, FL_TYPE_T_CORNER);
                //cout << "T joint 1" << endl;
                if (GET_DVAR(int, drawCorners))
                  putText(worldImage, "T", worldToImage(inter), CV_FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, CV_AA);
                //imshow("worldImage", worldImage);
                //waitKey(0);
              }
            } else if (center2) {
              // Line 1 is perpendicular
              // T joint
              float norm1, norm2;
              Point2f dist1, dist2;
              dist1 = wl1->p1 - inter;
              dist2 = wl1->p2 - inter;
              norm1 = norm(dist1);
              norm2 = norm(dist2);
              Point2f unitT =
                norm1 > norm2 ?
                  Point2f(dist1.x / norm1, dist1.y / norm1) :
                  Point2f(dist2.x / norm2, dist2.y / norm2);
              computeLandmark(inter, unitT, FL_TYPE_T_CORNER);
              //cout << "T joint 2" << endl;
              if (GET_DVAR(int, drawCorners))
                putText(worldImage, "T", worldToImage(inter), CV_FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, CV_AA);
              //imshow("worldImage", worldImage);
              //waitKey(0);
            } else {
              // L joint
              float norm11, norm12;
              Point2f dist11, dist12;
              dist11 = wl1->p1 - inter;
              dist12 = wl1->p2 - inter;
              norm11 = norm(dist11);
              norm12 = norm(dist12);
              Point2f unitL1 =
                norm11 > norm12 ?
                  Point2f(dist11.x / norm11, dist11.y / norm11) :
                  Point2f(dist12.x / norm12, dist12.y / norm12);
              float norm21, norm22;
              Point2f dist21, dist22;
              dist21 = wl2->p1 - inter;
              dist22 = wl2->p2 - inter;
              norm21 = norm(dist21);
              norm22 = norm(dist22);
              Point2f unitL2 =
                norm21 > norm22 ?
                  Point2f(dist21.x / norm21, dist21.y / norm21) :
                  Point2f(dist22.x / norm22, dist22.y / norm22);
              Point2f unitL = unitL1 + unitL2;
              float normL = norm(unitL);
              unitL.x = unitL.x / normL;
              unitL.y = unitL.y / normL;
              computeLandmark(inter, unitL, FL_TYPE_L_CORNER);
              //cout << "L joint " << endl;
              if (GET_DVAR(int, drawCorners))
                putText(worldImage, "L", worldToImage(inter), CV_FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, CV_AA);
            }
          }
        }
      }
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  findFeaturesTime = timeSpan.count();
}

void LinesExtraction::computeLandmark(
  const Point2f& inter,
  const Point2f& unitToBaseLine,
  const unsigned& type)
{
  float len = norm(inter);
  Point2f interUnit = Point2f(inter.x / len, inter.y / len);
  float cross = interUnit.x * unitToBaseLine.y - interUnit.y * unitToBaseLine.x;
  float dot = interUnit.dot(unitToBaseLine);
  float perpAngle = atan2(cross, dot) + atan2(inter.y, inter.x);
  float cosa = cos(perpAngle);
  float sina = sin(perpAngle);
  auto poseFromLandmark =
    RobotPose2D<float>(
      -cosa * inter.x - sina * inter.y,
      +sina * inter.x - cosa * inter.y,
      -perpAngle);
  auto l =
    boost::make_shared<KnownLandmark<float> >(
      type,
      inter,
      poseFromLandmark);
  knownLandmarks.push_back(l);
  //cout << "inter: " << inter << endl;
  //cout << "robotX: "<< l.poseFromLandmark.x << endl;
  //cout << "robotY: "<< l.poseFromLandmark.y << endl;
  //cout << "robotOrientation: "<< l.poseFromLandmark.theta * 180 / 3.14 << endl;
  //VisionUtils::drawPoint(Point(inter.x * 100 + 500, 350 - 100 * inter.y), worldImage, Scalar(0,255,0));
  //imshow("worldImage", worldImage);
  //waitKey(0);
}

bool LinesExtraction::findCircle(Circle& circleOutput, vector<Point2f>& circlePoints)
{
  auto tStart = high_resolution_clock::now();
  if (circlePoints.empty()) {
    duration<double> timeSpan = high_resolution_clock::now() - tStart;
    findCircleTime = timeSpan.count();
    return false;
  }
  if (circlePoints.size() > 15) {
    float radius = 0.75;
    Mat meanMat;
    reduce(circlePoints, meanMat, 01, CV_REDUCE_AVG);
    Point2f meanP = Point2f(meanMat.at<float>(0, 0), meanMat.at<float>(0, 1));
    float maxDist = 0.f;
    for (int i = 0; i < circlePoints.size(); ++i) {
      auto dist = norm(meanP - circlePoints[i]);
      maxDist = dist > maxDist ? dist : maxDist;
    }
    //cout << "meanP: " << meanP << endl;
    //cout << "maxDist: " << maxDist << endl;
    if (maxDist < radius - 0.1)
      return false;
    //for (size_t i = 0 ; i < circlePoints.size(); ++i)
    //  VisionUtils::drawPoint(worldToImage(circlePoints[i]), worldImage);
    int maxCount = circlePoints.size() > 20 ? 20 : circlePoints.size();
    unsigned RANSACiterations = 10;
    int bestPoints = 0;
    Circle bestCircle;
    mt19937 rng;
    uniform_real_distribution<> dist
      { 0, 1 };
    bool circleFound = false;
    for (size_t n = 0; n < RANSACiterations; ++n) {
      if (circleFound)
        break;
      Point2f p1 = circlePoints[(int) (dist(rng) * circlePoints.size())];
      Point2f p2 = circlePoints[(int) (dist(rng) * circlePoints.size())];
      ///< Make a circle for two points
      auto circles = Circle::pointsToCircle(p1, p2, radius);
      for (size_t c = 0; c < circles.size(); ++c) {
        int numPoints = 0;
        //cout << "center: " << circles[c].center << endl;
        //cout << "radius: " << radius << endl;
        for (size_t i = 0; i < maxCount; ++i) {
          float dist = norm(circles[c].center - circlePoints[i]) - radius;
          //cout << "dist: " << norm(circles[c].center - circlePoints[i]) << endl;
          if (fabsf(dist) < 0.05) {
            ++numPoints;
          } //else {
          //  --numPoints;
          //}
        }
        //cout << "numPoints: " << numPoints << endl;
        if (numPoints > bestPoints) {
          bestPoints = numPoints;
          bestCircle = circles[c];
        }
        /*if (GET_DVAR(int, drawCircle)) {
          circle(
            worldImage,
            worldToImage(circles[c].center),
            circles[c].radius * 100,
            Scalar(0, 255, 255 - c * 100),
            1
          );
        }*/
        //cout << "bestPoints:"  << bestPoints << endl;
        //cout << "maxCount:"  << maxCount << endl;
        if (bestPoints >= maxCount * 0.65)
          circleFound = true; break;
      }
      //if (n > RANSACiterations / 4.0 && bestPoints <= maxCount / 4.0)
      //  return false;
      //cout << "bestPoints: " << bestPoints << endl;
      //cout << "maxCount: " << maxCount << endl;
      //VisionUtils::displayImage(worldImage, "circle");
      //waitKey(0);
    }
    duration<double> timeSpan = high_resolution_clock::now() - tStart;
    findCircleTime = timeSpan.count();
    //cout << "maxCount:" << maxCount << endl;
    if (bestPoints >= maxCount * 0.65) {
      circleOutput = bestCircle;
      return true;
    } else {
      return false;
    }
  }
  return false;
}

void LinesExtraction::computeCircleLandmark(const Circle& c,
  const vector<FittedLinePtr>& worldLines)
{
  for (size_t i = 0; i < worldLines.size(); ++i) {
    if (!worldLines[i]) continue;
    vector<Point2f> inters;
    auto wl = worldLines[i];
    if (wl->d > 0.6) {
      if (findCircleLineIntersection(c, wl, inters)) {
        if (!inters.empty()) {
          for (int i = 0; i < inters.size(); ++i) {
            Point2f diff;
            diff = inters[i] - c.center;
            float dist = norm(diff);
            Point2f unit = Point2f(diff.x / dist, diff.y / dist);
            computeLandmark(c.center, unit, FL_TYPE_CIRCLE);
          }
          break;
        }
        //VisionUtils::drawPoint(Point(inters[0].x * 100 + 500, 350 - 100 * inters[0].y), worldImage, Scalar(0,255,0));
        //VisionUtils::drawPoint(Point(inters[1].x * 100 + 500, 350 - 100 * inters[1].y), worldImage, Scalar(0,255,0));
        //imshow("worldImage", worldImage);
        //waitKey(0);
      }
    }
  }
}

bool LinesExtraction::findCircleLineIntersection(const Circle& c,
  const FittedLinePtr& wl, vector<Point2f>& intersections)
{
  float t =
    wl->unit.x * (c.center.x - wl->p1.x) + wl->unit.y * (c.center.y - wl->p1.y);
  Point2f bisectorP;
  bisectorP.x = t * wl->unit.x + wl->p1.x;
  bisectorP.y = t * wl->unit.y + wl->p1.y;
  float perpDist = norm(bisectorP - c.center);
  if (perpDist < 0.2) {
    float dt = sqrt(c.radius * c.radius - perpDist * perpDist);
    Point2f inter1, inter2;
    inter1.x = bisectorP.x + dt * wl->unit.x;
    inter1.y = bisectorP.y + dt * wl->unit.y;

    inter2.x = bisectorP.x - dt * wl->unit.x;
    inter2.y = bisectorP.y - dt * wl->unit.y;
    intersections.push_back(inter1);
    intersections.push_back(inter2);
    return true;
  } else {
    return false;
  }
}

void LinesExtraction::addLineLandmarks(vector<FittedLinePtr>& worldLines)
{
  auto tStart = high_resolution_clock::now();
  int count = 0;
  float nDiv;
  nDiv = activeCamera == CameraId::headTop ? 0.2 : 0.05;
  for (size_t i = 0; i < worldLines.size(); ++i) {
    if (!worldLines[i]) continue;
    auto wl = worldLines[i];
    int numPoints = ceil(wl->d / nDiv);
    for (int i = 0; i < numPoints; ++i) {
      float r = i / (float) numPoints;
      auto l =
        boost::make_shared<UnknownLandmark<float> >(
          cv::Point_<float>(
            wl->p1.x + wl->unit.x * wl->d * r,
            wl->p1.y + wl->unit.y * wl->d * r)
          );
      l->type = FL_TYPE_LINES;
      if (GET_DVAR(int, drawUnknownLandmarks))
        VisionUtils::drawPoint(worldToImage(l->pos), worldImage, Scalar(255, 255, 255));
      unknownLandmarks.push_back(l);
      count++;
    }
    if (count >= 20) break;
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  addLandmarksTime = timeSpan.count();
  //cout << "Number of landmarks observerd2: " << OVAR(ObsLandmarks, VisionModule::unknownLandmarksObs).data.size() << endl;
}

/*bool LinesExtraction::checkEllipse(Ellipse& e)
{
  float ratio = e.rs / e.rl;
  if (ratio < 0.8 && ratio > 1.2) return false;
  if (e.rl < 0.45 || e.rs < 0.45 || e.rl > 0.9 || e.rs > 0.9) return false;
  return true;
}*/
