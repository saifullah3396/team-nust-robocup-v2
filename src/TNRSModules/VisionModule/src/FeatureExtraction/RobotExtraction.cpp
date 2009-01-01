/**
 * @file VisionModule/src/FeatureExtraction/RobotExtraction.cpp
 *
 * This file declares the class for robots extraction from
 * the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author AbdulRehman
 * @date 22 Aug 2017
 */

#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/DataHolders/ObstacleType.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/TNRSLine.h"
#include "VisionModule/include/CameraTransform.h"
#include "VisionModule/include/ColorHandler.h"
#include "VisionModule/include/FeatureExtraction/RegionSegmentation.h"
#include "VisionModule/include/FeatureExtraction/RegionScanners.h"
#include "VisionModule/include/FeatureExtraction/RobotExtraction.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/RobotRegion.h"
#include "VisionModule/include/FeatureExtraction/ScannedLine.h"
#include "VisionModule/include/FeatureExtraction/ScannedRegion.h"
#include "VisionModule/include/VisionModule.h"
#include "VisionModule/include/VisionExceptions.h"

RobotExtraction::RobotExtraction(VisionModule* visionModule) :
  FeatureExtraction(visionModule, "RobotExtraction"),
  DebugBase("RobotExtraction", this)
{
  ///< Set up debugging variables
  initDebugBase();
  GET_DEBUG_CONFIG(
    VisionConfig,
    RobotExtraction,
    (int, sendTime),
    (int, drawScannedLines),
    (int, drawJerseyRegions),
    (int, drawRobotRegions),
    (int, drawLowerBodyRegions),
    (int, displayInfo),
    (int, displayOutput),
  );

  GET_CLASS_CONFIG(
    VisionConfig,
    RobotExtraction,
    refreshTime,
    maxRobotTrackers,
    maxRobotWorldWidth,
    fallenRobotWidth,
    robotMatchMaxDistance,
    jerseyApproxHeight,
    maxJerseyWidthRatio,
    jerseyBelowBorderMin,
    jerseyAboveBorderRelCutoff,
    lineLinkHorXTolRatio,
    lineLinkHorYTolRatio,
    lineLinkVerXTolRatio,
    lineLinkVerYTolRatio,
    maxLineLengthDiffRatio,
    regionsXDiffTol,
    regionsYDiffTol,
    maxRegionSizeDiffRatio,
    lowerBodyRegionsXDiffTol,
    lowerBodyRegionsYDiffTol,
    lowerBodyMaxRegionSizeDiffRatio,
    lowerBodyLineLinkHorXTolRatio,
    lowerBodyLineLinkHorYTolRatio,
    lowerBodyLineLinkVerXTolRatio,
    lowerBodyLineLinkVerYTolRatio,
    lowerBodyMaxLineLengthDiffRatio,
  )
  ///< Get other feature extraction classes
  fieldExt = GET_FEATURE_EXT_CLASS(FieldExtraction, FeatureExtractionIds::field);
  regionSeg = GET_FEATURE_EXT_CLASS(RegionSegmentation, FeatureExtractionIds::segmentation);

  ///< Initializing processing times
  processTime = 0.f;
  linesFilterTime = 0.f;
  findJerseysTime = 0.f;
  classifyRobotsTime = 0.f;
  findStratRegionsTime = 0.f;
  updateRobotsInfoTime = 0.f;
}

void RobotExtraction::processImage()
{
  auto tStart = high_resolution_clock::now();
  if (activeCamera == CameraId::headTop) {
    if (fieldExt->isFound()) {
      refreshRobotRegions();
      filterRobotLines();
      vector<RobotRegionPtr> jerseyRegions;
      findJerseys(jerseyRegions);
      vector<RobotRegionPtr> robotRegions;
      classifyRobots(robotRegions, jerseyRegions);
      updateRobotTrackers(robotRegions);
      updateRobotsInfo();
      drawResults();
    }
  } else {
    refreshRobotRegions();
    filterRobotLines();
    vector<RobotRegionPtr> robotRegions;
    findLowerBodyRegions(robotRegions);
    updateRobotTrackers(robotRegions);
    updateRobotsInfo();
    drawResults();
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  processTime = timeSpan.count();
  if (GET_DVAR(int, displayOutput)) {
    if (activeCamera == CameraId::headTop)
      VisionUtils::displayImage(name, bgrMat[toUType(activeCamera)]);
  }
  if (GET_DVAR(int, displayInfo)) {
    LOG_INFO("RobotExtraction Results:")
    LOG_INFO("Time taken by overall process: " << processTime);
    LOG_INFO("Time taken by robot scanned lines filter: " << linesFilterTime);
    LOG_INFO("Time taken by finding robot jerseys: " << findJerseysTime);
    LOG_INFO("Time taken by robots classification: " << classifyRobotsTime);
    //LOG_INFO("Time taken by finding lower body regions: " << findLowerBodyRegionsTime);
    LOG_INFO("Time taken by updating robots in memory: " << updateRobotsInfoTime);
    LOG_INFO("Number of robot regions tracked: " << trackedRobots.size())
  }
}

void RobotExtraction::refreshRobotRegions()
{
  trackedRobots.resize(maxRobotTrackers);
  float currentTime = visionModule->getModuleTime();
  auto iter = trackedRobots.begin();
  while (iter != trackedRobots.end()) {
    if (*iter) {
      if (currentTime - (*iter)->getLastUpdateTime() < refreshTime) {
        ++iter;
      } else {
        iter = trackedRobots.erase(iter);
      }
    } else {
      iter = trackedRobots.erase(iter);
    }
  }
}

void RobotExtraction::filterRobotLines()
{
  auto tStart = high_resolution_clock::now();
  auto& verRobotLines = regionSeg->getVerticalScan(ScanTypes::robot)->scanLines;
  auto& horRobotLines = regionSeg->getHorizontalScan(ScanTypes::robot)->scanLines;
  //regionSeg->getHorizontalScan(ScanTypes::robot)->draw(bgrMat[toUType(activeCamera)]);
  //regionSeg->getVerticalScan(ScanTypes::robot)->draw(bgrMat[toUType(activeCamera)]);
  if (activeCamera == CameraId::headTop) {
    const auto& border = fieldExt->getBorder();
    ///< Filter with border
    for (auto& rl : verRobotLines) {
      if (!rl) continue;
      int yBorder = border[rl->baseIndex];
      if (rl->start < yBorder) {
        if (rl->end - yBorder > jerseyBelowBorderMin) {
          rl->start = std::max(0, yBorder - jerseyAboveBorderRelCutoff);
        } else {
          rl.reset();
        }
      }
    }
  }

  if (GET_DVAR(int, drawScannedLines)) {
    for (auto& rl : horRobotLines) {
      if (rl)
        line(
          bgrMat[toUType(activeCamera)],
          Point(rl->start, rl->baseIndex),
          Point(rl->end, rl->baseIndex),
          colorToBgr[toUType(TNColors::black)],
          1);
    }

    for (auto& rl : verRobotLines) {
      if (rl)
        line(
          bgrMat[toUType(activeCamera)],
          Point(rl->baseIndex, rl->start),
          Point(rl->baseIndex, rl->end),
          colorToBgr[toUType(TNColors::black)],
          1);
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  linesFilterTime = timeSpan.count();
}

void RobotExtraction::findJerseys(vector<RobotRegionPtr>& outputRegions)
{
  auto tStart = high_resolution_clock::now();
  auto verJerseyLinesOurs = regionSeg->getVerticalScan(ScanTypes::ourJersey)->scanLines;
  auto horJerseyLinesOurs = regionSeg->getHorizontalScan(ScanTypes::ourJersey)->scanLines;
  auto verJerseyLinesOpps = regionSeg->getVerticalScan(ScanTypes::oppJersey)->scanLines;
  auto horJerseyLinesOpps = regionSeg->getHorizontalScan(ScanTypes::oppJersey)->scanLines;
  vector<ScannedRegionPtr> verJerseyRegionsOurs;
  vector<ScannedRegionPtr> horJerseyRegionsOurs;
  vector<ScannedRegionPtr> verJerseyRegionsOpps;
  vector<ScannedRegionPtr> horJerseyRegionsOpps;
  auto verLineLinkXTol =
    regionSeg->getVerticalScan(ScanTypes::ourJersey)->highStep * lineLinkVerXTolRatio; // pixels
  auto verLineLinkYTol =
    regionSeg->getVerticalScan(ScanTypes::ourJersey)->highStep * lineLinkVerYTolRatio; // pixels
  auto horLineLinkXTol =
    regionSeg->getHorizontalScan(ScanTypes::ourJersey)->highStep * lineLinkHorXTolRatio; // pixels
  auto horLineLinkYTol =
    regionSeg->getHorizontalScan(ScanTypes::ourJersey)->highStep * lineLinkHorYTolRatio; // pixels
  findRegions(
    verJerseyRegionsOurs,
    verJerseyLinesOurs,
    verLineLinkXTol,
    verLineLinkYTol,
    maxLineLengthDiffRatio,
    bgrMat[toUType(activeCamera)]);
  findRegions(horJerseyRegionsOurs,
    horJerseyLinesOurs,
    horLineLinkXTol,
    horLineLinkYTol,
    maxLineLengthDiffRatio,
    bgrMat[toUType(activeCamera)]);
  findRegions(verJerseyRegionsOpps,
    verJerseyLinesOpps,
    verLineLinkXTol,
    verLineLinkYTol,
    maxLineLengthDiffRatio,
    bgrMat[toUType(activeCamera)]);
  findRegions(horJerseyRegionsOpps,
    horJerseyLinesOpps,
    horLineLinkXTol,
    horLineLinkYTol,
    maxLineLengthDiffRatio,
    bgrMat[toUType(activeCamera)]);
  ScannedRegion::drawRegions(
    bgrMat[toUType(activeCamera)], verJerseyRegionsOurs, Scalar(255,0,0), 2);
  ScannedRegion::drawRegions(
    bgrMat[toUType(activeCamera)], horJerseyRegionsOurs, Scalar(0,0,255), 2);
  regionSeg->getVerticalScan(ScanTypes::ourJersey)->draw(bgrMat[toUType(activeCamera)]);
  regionSeg->getHorizontalScan(ScanTypes::ourJersey)->draw(bgrMat[toUType(activeCamera)]);
  horJerseyRegionsOurs.insert(
    horJerseyRegionsOurs.end(),
    verJerseyRegionsOurs.begin(),
    verJerseyRegionsOurs.end());
  horJerseyRegionsOpps.insert(
    horJerseyRegionsOpps.end(),
    verJerseyRegionsOpps.begin(),
    verJerseyRegionsOpps.end());
  vector<ScannedRegionPtr> oursFiltered;
  vector<ScannedRegionPtr> oppsFiltered;
  ScannedRegion::linkRegions(
    oursFiltered,
    horJerseyRegionsOurs,
    regionsXDiffTol,
    regionsYDiffTol,
    maxRegionSizeDiffRatio,
    bgrMat[toUType(activeCamera)]);
  ScannedRegion::linkRegions(
    oppsFiltered,
    horJerseyRegionsOpps,
    regionsXDiffTol,
    regionsYDiffTol,
    maxRegionSizeDiffRatio,
    bgrMat[toUType(activeCamera)]);
  ScannedRegion::drawRegions(
    bgrMat[toUType(activeCamera)], oursFiltered, Scalar(255,0,0), 2);
  ScannedRegion::drawRegions(
    bgrMat[toUType(activeCamera)], oppsFiltered, Scalar(0,0,255), 2);
  for (const auto& jr : oursFiltered) {
    if (jr) {
      if (jr->rect.width > 5 * jr->rect.height ||
          jr->rect.height > 5 * jr->rect.width)
        continue;
      outputRegions.push_back(boost::make_shared<RobotRegion>(jr, ObstacleType::teammate));
      if (GET_DVAR(int, drawJerseyRegions))
        jr->draw(bgrMat[toUType(activeCamera)], colorToBgr[toUType(ourColor)]);
    }
  }
  for (const auto& jr : oppsFiltered) {
    if (jr) {
      if (jr->rect.width > 5 * jr->rect.height ||
          jr->rect.height > 5 * jr->rect.width)
        continue;
      outputRegions.push_back(boost::make_shared<RobotRegion>(jr, ObstacleType::opponent));
      if (GET_DVAR(int, drawJerseyRegions))
        jr->draw(bgrMat[toUType(activeCamera)], colorToBgr[toUType(oppColor)]);
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  findJerseysTime = timeSpan.count();
}

void RobotExtraction::classifyRobots(vector<RobotRegionPtr>& robotRegions, vector<RobotRegionPtr>& jerseyRegions)
{
  auto tStart = high_resolution_clock::now();
  auto& verRobotLines = regionSeg->getVerticalScan(ScanTypes::robot)->scanLines;
  auto& horRobotLines = regionSeg->getHorizontalScan(ScanTypes::robot)->scanLines;

  sort(jerseyRegions.begin(), jerseyRegions.end(), [](
    const RobotRegionPtr& rr1,
    const RobotRegionPtr& rr2)
  { return rr1->sr->center.x < rr2->sr->center.x;});

  for (auto& rr : jerseyRegions) {
    if (!rr) continue;

    bool fallen = false;
    bool posFromJersey = false;

    ScannedRegionPtr robotFound;
    if (rr->sr->rect.area() > 1000) {
      static const auto bodyToJerseyMinHeightRatio = 1.0;
      static const auto bodyToJerseyMinWidthRatio = 1.0;
      // Large area
      if (rr->sr->rect.height >= rr->sr->rect.width) { // If it is vertical
        auto halfHeight = rr->sr->rect.height / 2;
        auto minY = rr->sr->rect.y - halfHeight;
        auto maxY = rr->sr->leftBase.y + halfHeight;
        auto minX = rr->sr->leftBase.x - rr->sr->rect.width;
        auto maxX = rr->sr->rightBase.x + rr->sr->rect.width;
        vector<LinearScannedLinePtr> bodyLines;
        for (const auto& vl : verRobotLines) {
          if (vl && vl->baseIndex > minX && vl->baseIndex < maxX) {
            if (vl->start > minY && vl->start < maxY)
              bodyLines.push_back(vl);
          }
        }
        vector<Point> verPts;
        if (bodyLines.empty()) continue;
        for (const auto& l : bodyLines) {
          verPts.push_back(Point(l->baseIndex, l->start));
          verPts.push_back(Point(l->baseIndex, l->end));
        }
        auto bodyRect = boundingRect(verPts) | rr->sr->rect;
        auto hRatio = bodyRect.height / ((float) rr->sr->rect.height);
        if (hRatio > bodyToJerseyMinHeightRatio) {
          robotFound = boost::make_shared <ScannedRegion> (bodyRect);
          //region found
        }
      } else { // if the rectangle is greater in width
        //cout << "width based" << endl;
        auto minY = rr->sr->rect.y - rr->sr->rect.height;
        auto maxY = rr->sr->leftBase.y + rr->sr->rect.height;
        auto minX = rr->sr->leftBase.x - rr->sr->rect.width;
        auto maxX = rr->sr->rightBase.x + rr->sr->rect.width;
        vector<LinearScannedLinePtr> bodyLines;
        for (const auto& hl : horRobotLines) {
          if (!hl) continue;
          if (hl->start > minX && hl->start < maxX) {
            if (hl->baseIndex > minY && hl->baseIndex < maxY) bodyLines.push_back(hl);
          } else if (hl->end > minX && hl->end < maxX) {
            if (hl->baseIndex > minY && hl->baseIndex < maxY) bodyLines.push_back(hl);
          }
        }
        vector<Point> horPts;
        if (bodyLines.empty()) continue;
        for (const auto& l : bodyLines) {
          horPts.push_back(Point(l->start, l->baseIndex));
          horPts.push_back(Point(l->end, l->baseIndex));
        }
        auto bodyRect = boundingRect(horPts) | rr->sr->rect;
        auto wRatio = bodyRect.width / ((float) rr->sr->rect.width);
        if (wRatio > bodyToJerseyMinWidthRatio) {
          fallen = true;
          robotFound = boost::make_shared<ScannedRegion> (bodyRect);
          //region found
        }
      }
    } else {
      // Small area
      float bodyToJerseyMinHeightRatio = 1.5;
      float bodyToJerseyMinWidthRatio = 1.5;
      auto doubleWidth = rr->sr->rect.width * 2;
      auto minY = rr->sr->rect.y - rr->sr->rect.height;
      auto maxY = rr->sr->leftBase.y + rr->sr->rect.height;
      auto minX = rr->sr->leftBase.x - doubleWidth;
      auto maxX = rr->sr->rightBase.x + doubleWidth;
      vector<LinearScannedLinePtr> bodyLines;
      for (const auto& vl : verRobotLines) {
        if (vl && vl->baseIndex > minX && vl->baseIndex < maxX) {
          if (vl->start > minY && vl->start < maxY)
            bodyLines.push_back(vl);
        }
      }
      vector<Point> verPts;
      if (bodyLines.empty()) continue;
      for (const auto& l : bodyLines) {
        verPts.push_back(Point(l->baseIndex, l->start));
        verPts.push_back(Point(l->baseIndex, l->end));
      }
      auto bodyRect = boundingRect(verPts) | rr->sr->rect;
      auto hRatio = bodyRect.height / ((float) rr->sr->rect.height);
      if (hRatio > bodyToJerseyMinHeightRatio) {
        robotFound = boost::make_shared<ScannedRegion> (bodyRect);
        // region found
      } else {
        minX = rr->sr->leftBase.x - doubleWidth;
        maxX = rr->sr->rightBase.x + doubleWidth;
        vector<LinearScannedLinePtr> bodyLines;
        for (const auto& hl : horRobotLines) {
          if (!hl) continue;
          if (hl->start > minX && hl->start < maxX) {
            if (hl->baseIndex > minY && hl->baseIndex < maxY)
              bodyLines.push_back(hl);
          } else if (hl->end > minX && hl->end < maxX) {
            if (hl->baseIndex > minY && hl->baseIndex < maxY) bodyLines.push_back(hl);
          }
        }
        vector<Point> horPts;
        if (bodyLines.empty()) continue;
        for (const auto& l : bodyLines) {
          horPts.push_back(Point(l->start, l->baseIndex));
          horPts.push_back(Point(l->end, l->baseIndex));
        }
        auto bodyRect = boundingRect(horPts) | rr->sr->rect;
        auto wRatio = bodyRect.width / ((float) rr->sr->rect.width);
        if (wRatio > bodyToJerseyMinWidthRatio) {
          fallen = true;
          robotFound = boost::make_shared<ScannedRegion> (bodyRect);
          // region found
        }
      }
    }

    if (robotFound) {
      // If robot is too close and its legs are not showing
      auto robotMaxY = robotFound->rect.y + robotFound->rect.height;
      //! Jersey center
      Point2f jerseyCenterWorld;
      cameraTransforms[toUType(activeCamera)]->imageToWorld(
        jerseyCenterWorld, rr->sr->center, jerseyApproxHeight);
      //VisionUtils::drawPoint(rr->sr->center, bgrMat[toUType(activeCamera)]);
      if (getImageHeight() - robotMaxY < 10 && !fallen) {
        posFromJersey = true;
      }
      rr->bodySr = robotFound;

      if (!fallen) {
        //! Make a horizontal box for upper body
        Rect upperRect;
        upperRect.y = rr->sr->rect.y;

        //! Maximum of body width or ratio of jersey width
        upperRect.width = std::min(float(rr->bodySr->rect.width), rr->sr->rect.width * maxJerseyWidthRatio);
        upperRect.height = rr->sr->rect.height;
        upperRect.x = rr->sr->center.x - upperRect.width / 2;

        //! Make a vertical box for lower body
        Rect lowerRect;
        lowerRect.x = rr->sr->rect.x;
        lowerRect.y = rr->bodySr->rect.y;
        lowerRect.width = rr->sr->rect.width;
        lowerRect.height = rr->bodySr->rect.height;
        rr->sr = boost::make_shared<ScannedRegion>(upperRect);
        rr->bodySr = boost::make_shared<ScannedRegion>(lowerRect);
      }

      ///< Find world position and verify robot width in world
      if (posFromJersey) {
        // Jersey center with approximate height of 35 cms
        cameraTransforms[toUType(activeCamera)]->imageToWorld(
          rr->frontLeft, Point2f(rr->sr->leftBase.x, rr->sr->leftBase.y), jerseyApproxHeight);
        cameraTransforms[toUType(activeCamera)]->imageToWorld(
          rr->frontRight, Point2f(rr->sr->rightBase.x, rr->sr->rightBase.y), jerseyApproxHeight);
        rr->world = jerseyCenterWorld;
      } else {
        cameraTransforms[toUType(activeCamera)]->imageToWorld(
          rr->frontLeft, Point2f(rr->bodySr->leftBase.x, rr->bodySr->leftBase.y), 0.0);
        cameraTransforms[toUType(activeCamera)]->imageToWorld(
          rr->frontRight, Point2f(rr->bodySr->rightBase.x, rr->bodySr->rightBase.y), 0.0);
        cameraTransforms[toUType(activeCamera)]->imageToWorld(
          rr->world, Point2f(robotFound->center.x, rr->bodySr->leftBase.y), 0.0);
      }

      auto baseWidth = cv::norm(rr->frontRight - rr->frontLeft);
      if (!posFromJersey) {
        if (baseWidth > maxRobotWorldWidth && !fallen) {
          continue;
        } else if (baseWidth > fallenRobotWidth && fallen) {
          continue;
        }
      }

      if (fallen) {
        if (rr->obstacleType == ObstacleType::teammate)
          rr->obstacleType == ObstacleType::teammateFallen;
        else if (rr->obstacleType == ObstacleType::opponent)
          rr->obstacleType == ObstacleType::opponentFallen;
      }

      robotRegions.push_back(rr);
      /*rectangle(bgrMat[toUType(activeCamera)], rr->sr->rect, Scalar(255,0,0), 3);
      rectangle(bgrMat[toUType(activeCamera)], rr->bodySr->rect, Scalar(255,0,255), 3);
      VisionUtils::drawPoint(Point(rr->sr->leftBase.x, rr->sr->leftBase.y), bgrMat[toUType(activeCamera)]);
      VisionUtils::drawPoint(Point(rr->sr->rightBase.x, rr->sr->rightBase.y), bgrMat[toUType(activeCamera)]);
      VisionUtils::drawPoint(Point2f(rr->bodySr->center.x, rr->bodySr->leftBase.y), bgrMat[toUType(activeCamera)]);
     VisionUtils::displayImage("Region", bgrMat[toUType(activeCamera)]);
      waitKey(0);*/
    }
  }

  for (auto& r1 : robotRegions) {
    for (auto& r2 : robotRegions) {
      if (!r1) continue;

      if (!r2) continue;
      if (r1 != r2) {
        if ((r1->sr->rect & r2->sr->rect).area() > 0) { // overlap top
          if (r1->sr->rect.area() > r2->sr->rect.area()) {
            r1->sr = boost::make_shared<ScannedRegion>(r1->sr->rect | r2->sr->rect);
            r1->bodySr = boost::make_shared<ScannedRegion>(r1->bodySr->rect | r2->bodySr->rect);
            r2.reset();
            continue;
          } else {
            r2->sr = boost::make_shared<ScannedRegion>(r1->sr->rect | r2->sr->rect);
            r2->bodySr = boost::make_shared<ScannedRegion>(r1->bodySr->rect | r2->bodySr->rect);
            r1.reset();
            continue;
          }
        }

        if ((r1->bodySr->rect & r2->bodySr->rect).area() > 0) { // overlap bottom
          if (r1->bodySr->rect.area() > r2->bodySr->rect.area()) {
            r1->sr = boost::make_shared<ScannedRegion>(r1->sr->rect | r2->sr->rect);
            r1->bodySr = boost::make_shared<ScannedRegion>(r1->bodySr->rect | r2->bodySr->rect);
            r2.reset();
            continue;
          } else {
            r2->sr = boost::make_shared<ScannedRegion>(r1->sr->rect | r2->sr->rect);
            r2->bodySr = boost::make_shared<ScannedRegion>(r1->bodySr->rect | r2->bodySr->rect);
            r1.reset();
            continue;
          }
        }
      }
    }
  }

  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  classifyRobotsTime = timeSpan.count();
}

void RobotExtraction::updateRobotTrackers(
  const vector<boost::shared_ptr<RobotRegion>>& robotRegions)
{
  auto currentTime = visionModule->getModuleTime();
  for (const auto& rr : robotRegions) {
    if (!rr) continue;
    // Update new regions and check if a robot already exists in the region
    auto closest = 1000;
    boost::shared_ptr<RobotTracker> bestMatch;
    //rectangle(bgrMat[toUType(activeCamera)], rr->bodySr->rect, Scalar(255,0,0), 3);
    //VisionUtils::displayImage("Region", bgrMat[toUType(activeCamera)]);
    //waitKey(0);
    for (auto& tr : trackedRobots) {
      auto dx = rr->world.x - tr->getState()[0];
      auto dy = rr->world.y - tr->getState()[1];
      auto dist = sqrt(dx * dx + dy * dy);
      if (dist < robotMatchMaxDistance && dist < closest) {
        closest = dist;
        bestMatch = tr;
      }
    }

    if (bestMatch &&
        (rr->obstacleType == bestMatch->getObstacleType() ||
        rr->obstacleType == ObstacleType::unknown))
    {
      Matrix<float, 10, 1> meas;
      if (activeCamera == CameraId::headBottom) {
        meas[0] = rr->world.x;
        meas[1] = rr->world.y;
        for (int i = 2; i < 10; ++i) {
          meas[i] = NAN;
        }
      } else {
        meas[0] = rr->world.x;
        meas[1] = rr->world.y;
        meas[2] = rr->sr->rect.x;
        meas[3] = rr->sr->rect.y;
        meas[4] = rr->sr->rect.width;
        meas[5] = rr->sr->rect.height;
        meas[6] = rr->bodySr->rect.x;
        meas[7] = rr->bodySr->rect.y;
        meas[8] = rr->bodySr->rect.width;
        meas[9] = rr->bodySr->rect.height;
      }
      bestMatch->update(meas, currentTime);
    } else {
      if (activeCamera == CameraId::headTop) {
        bool addNewTracker = false;
        if (trackedRobots.size() < maxRobotTrackers) {
          addNewTracker = true;
        } else if (norm(rr->world) < trackedRobots.back()->getDist()) {
          addNewTracker = true;
        }
        if (addNewTracker) {
          Matrix<float, 12, 1> state;
          state[0] = rr->world.x;
          state[1] = rr->world.y;
          state[2] = 0.0;
          state[3] = 0.0;
          state[4] = rr->sr->rect.x;
          state[5] = rr->sr->rect.y;
          state[6] = rr->sr->rect.width;
          state[7] = rr->sr->rect.height;
          state[8] = rr->bodySr->rect.x;
          state[9] = rr->bodySr->rect.y;
          state[10] = rr->bodySr->rect.width;
          state[11] = rr->bodySr->rect.height;
          auto newTracked = boost::shared_ptr<RobotTracker>(new RobotTracker(rr->obstacleType));
          newTracked->init(state, this->cycleTime, visionModule->getModuleTime());
          trackedRobots.push_back(newTracked);
        }
      }
    }
  }
  sort(trackedRobots.begin(), trackedRobots.end(), [](
    const boost::shared_ptr<RobotTracker>& tr1,
    const boost::shared_ptr<RobotTracker>& tr2)
  { return tr1->getDist() < tr2->getDist();});
}

void RobotExtraction::findLowerBodyRegions(vector<RobotRegionPtr>& robotRegions)
{
  auto tStart = high_resolution_clock::now();
  auto& verRobotLines = regionSeg->getVerticalScan(ScanTypes::robot)->scanLines;
  auto& horRobotLines = regionSeg->getHorizontalScan(ScanTypes::robot)->scanLines;

  vector<ScannedRegionPtr> verRobotRegions;
  vector<ScannedRegionPtr> horRobotRegions;
  auto verLineLinkXTol =
    regionSeg->getVerticalScan(ScanTypes::ourJersey)->highStep * lowerBodyLineLinkVerXTolRatio; // pixels
  auto verLineLinkYTol =
    regionSeg->getVerticalScan(ScanTypes::ourJersey)->highStep * lowerBodyLineLinkVerYTolRatio; // pixels
  auto horLineLinkXTol =
    regionSeg->getHorizontalScan(ScanTypes::ourJersey)->highStep * lowerBodyLineLinkHorXTolRatio; // pixels
  auto horLineLinkYTol =
    regionSeg->getHorizontalScan(ScanTypes::ourJersey)->highStep * lowerBodyLineLinkHorYTolRatio; // pixels
  findRegions(
    verRobotRegions,
    verRobotLines,
    verLineLinkXTol,
    verLineLinkYTol,
    lowerBodyMaxLineLengthDiffRatio,
    bgrMat[toUType(activeCamera)]);
  findRegions(
    horRobotRegions,
    horRobotLines,
    horLineLinkXTol,
    horLineLinkYTol,
    lowerBodyMaxLineLengthDiffRatio,
    bgrMat[toUType(activeCamera)]);
  //ScannedRegion::drawRegions(
  //  bgrMat[toUType(activeCamera)], verRobotRegions, Scalar(255,0,0));
  //ScannedRegion::drawRegions(
  //  bgrMat[toUType(activeCamera)], horRobotRegions, Scalar(0,255,0));
  horRobotRegions.insert(
    horRobotRegions.end(),
    verRobotRegions.begin(),
    verRobotRegions.end());

  this->lowerCamRobotRegions.clear();
  ScannedRegion::linkRegions(
    this->lowerCamRobotRegions,
    horRobotRegions,
    lowerBodyRegionsXDiffTol,
    lowerBodyRegionsYDiffTol,
    lowerBodyMaxRegionSizeDiffRatio,
    bgrMat[toUType(activeCamera)]);

  if (GET_DVAR(int, drawLowerBodyRegions))
    ScannedRegion::drawRegions(
      bgrMat[toUType(activeCamera)], this->lowerCamRobotRegions, Scalar(0,0,255));

  for (const auto& lbr : this->lowerCamRobotRegions) {
    robotRegions.push_back(boost::make_shared<RobotRegion>(lbr, ObstacleType::unknown));
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  findStratRegionsTime = timeSpan.count();
}

void RobotExtraction::updateRobotsInfo()
{
  auto tStart = high_resolution_clock::now();
  classifiedRobotRegions.clear();
  for (const auto& rt : trackedRobots) {
    const Matrix<float, 12, 1>& state = rt->getState();

    //! Make a rect for upper region
    if (activeCamera == CameraId::headTop) {
      auto top = Rect(state[4], state[5], state[6], state[7]);
      auto bottom = Rect(state[8], state[9], state[10], state[11]);
      auto rr = boost::make_shared<RobotRegion>(boost::make_shared<ScannedRegion>(top), rt->getObstacleType());
      rr->bodySr = boost::make_shared<ScannedRegion>(bottom);
      rr->world = Point2f(state[0], state[1]);
      classifiedRobotRegions.push_back(rr);
    }

    Obstacle<float> obstacle(rt->getObstacleType());
    obstacle.center = Point2f(state[0] + obstacle.depth, state[1]);
    const auto& robotPose2D = ROBOT_POSE_2D_IN(VisionModule);
    obstacle.centerT = robotPose2D.transform(obstacle.center);

    OBSTACLES_OBS_OUT(VisionModule).data.push_back(obstacle);
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  updateRobotsInfoTime = timeSpan.count();
}

void RobotExtraction::drawResults()
{
  if (GET_DVAR(int, drawRobotRegions)) {
    for (const auto& tr : trackedRobots) {
      if (tr->getObstacleType() == ObstacleType::teammate ||
          tr->getObstacleType() == ObstacleType::teammateFallen)
      {
        auto state = tr->getState();
        rectangle(
          bgrMat[toUType(activeCamera)],
          Rect(state[4], state[5], state[6], state[7]),
          colorToBgr[toUType(ourColor)]);
        rectangle(
          bgrMat[toUType(activeCamera)],
          Rect(state[8], state[9], state[10], state[11]),
          colorToBgr[toUType(ourColor)]);
      } else if (
        tr->getObstacleType() == ObstacleType::opponent ||
        tr->getObstacleType() == ObstacleType::opponentFallen)
      {
        auto state = tr->getState();
        rectangle(
          bgrMat[toUType(activeCamera)],
          Rect(state[4], state[5], state[6], state[7]),
          colorToBgr[toUType(oppColor)]);
        rectangle(
          bgrMat[toUType(activeCamera)],
          Rect(state[8], state[9], state[10], state[11]),
          colorToBgr[toUType(oppColor)]);
      }
    }
  }
}
