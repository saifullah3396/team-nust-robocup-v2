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
#include "VisionModule/include/FeatureExtraction/BallExtraction.h"
#include "VisionModule/include/FeatureExtraction/RegionSegmentation.h"
#include "VisionModule/include/FeatureExtraction/RegionScanners.h"
#include "VisionModule/include/FeatureExtraction/RobotExtraction.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/RobotRegion.h"
#include "VisionModule/include/FeatureExtraction/ScannedLine.h"
#include "VisionModule/include/FeatureExtraction/ScannedRegion.h"
#include "VisionModule/include/VisionModule.h"
#include "VisionModule/include/VisionExceptions.h"

#define ROBOT_WIDTH 0.50
#define FALLEN_ROBOT_WIDTH 0.75
#define ROBOT_MATCH_MAX_DISTANCE 0.35
#define JERSEY_APPROX_HEIGHT 0.35

RobotExtraction::RobotExtraction(VisionModule* visionModule) :
  FeatureExtraction(visionModule),
  DebugBase("RobotExtraction", this)
{
  ///< Set up debugging variables
  initDebugBase();
  int tempSendTime;
  int tempDrawScannedLines;
  int tempDrawRobotRegions;
  int tempDrawJerseyRegions;
  int tempDrawStrayRegions;
  int tempDisplayInfo;
  int tempDisplayOutput;
  GET_CONFIG(
    "VisionConfig",
    (int, RobotExtraction.sendTime, tempSendTime),
    (int, RobotExtraction.drawScannedLines, tempDrawScannedLines),
    (int, RobotExtraction.drawJerseyRegions, tempDrawJerseyRegions),
    (int, RobotExtraction.drawRobotRegions, tempDrawRobotRegions),
    (int, RobotExtraction.drawStrayRegions, tempDrawStrayRegions),
    (int, RobotExtraction.displayInfo, tempDisplayInfo),
    (int, RobotExtraction.displayOutput, tempDisplayOutput),
  );
  SET_DVAR(int, sendTime, tempSendTime);
  SET_DVAR(int, drawScannedLines, tempDrawScannedLines);
  SET_DVAR(int, drawJerseyRegions, tempDrawJerseyRegions);
  SET_DVAR(int, drawRobotRegions, tempDrawRobotRegions);
  SET_DVAR(int, drawStrayRegions, tempDrawStrayRegions);
  SET_DVAR(int, displayInfo, tempDisplayInfo);
  SET_DVAR(int, displayOutput, tempDisplayOutput);

  ///< Load ball classifier
  // Not used anymore
  //try {
  //  loadRobotClassifier();
  //} catch (VisionException& e) {
  //  LOG_EXCEPTION(e.what());
  //}

  ///< Get other feature extraction classes
  fieldExt = GET_FEATURE_EXT_CLASS(FieldExtraction, FeatureExtractionIds::field);
  ballExt = GET_FEATURE_EXT_CLASS(BallExtraction, FeatureExtractionIds::ball);
  regionSeg = GET_FEATURE_EXT_CLASS(RegionSegmentation, FeatureExtractionIds::segmentation);

  ///< Initializing processing times
  processTime = 0.f;
  linesFilterTime = 0.f;
  findJerseysTime = 0.f;
  classifyRobotsTime = 0.f;
  findStratRegionsTime = 0.f;
  updateRobotsInfoTime = 0.f;
}

/*void RobotExtraction::loadRobotClassifier()
{
  string classifierFile =
    ConfigManager::getConfigDirPath() +
    "/Classifiers/ballClassifier.xml";

  if (!classifier.load(classifierFile)) {
    throw
      VisionException(
        "RobotExtraction",
        "Robot classifier not found.",
        true
      );
  }
}
*/

void RobotExtraction::processImage()
{
  auto tStart = high_resolution_clock::now();
  if (fieldExt->isFound()) {
    strayRegions.clear();
    refreshRobotRegions();
    filterRobotLines();
    findJerseys();
    classifyRobots();
    findStrayRegions();
    updateRobotsInfo();
    drawResults();
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  processTime = timeSpan.count();
  if (GET_DVAR(int, displayOutput))
    VisionUtils::displayImage("RobotExtraction", bgrMat[toUType(activeCamera)]);
  if (GET_DVAR(int, displayInfo)) {
    LOG_INFO("RobotExtraction Results:")
    LOG_INFO("Time taken by overall process: " << processTime);
    LOG_INFO("Time taken by robot scanned lines filter: " << linesFilterTime);
    LOG_INFO("Time taken by finding robot jerseys: " << findJerseysTime);
    LOG_INFO("Time taken by robots classification: " << classifyRobotsTime);
    LOG_INFO("Time taken by finding stray regions: " << findStratRegionsTime);
    LOG_INFO("Time taken by updating robots in memory: " << updateRobotsInfoTime);
    LOG_INFO("Number of robot regions found: " << filtRobotRegions.size())
  }
}

void RobotExtraction::refreshRobotRegions()
{
  float currentTime = visionModule->getModuleTime();
  RRIter iter = filtRobotRegions.begin();
  while (iter != filtRobotRegions.end()) {
    if (*iter) {
      if (currentTime - (*iter)->timeDetected < refreshTime) {
        ++iter;
      } else {
        iter = filtRobotRegions.erase(iter);
      }
    } else {
      iter = filtRobotRegions.erase(iter);
    }
  }
  jerseyRegions.clear();
}

void RobotExtraction::filterBallRegions(
  vector<boost::shared_ptr<LinearScannedLine> >& robotLines, const bool& direction)
{
  auto& ballInfo = BALL_INFO_OUT(VisionModule);
  int ballMin, ballMax;
  if (direction) {
    ballMin = (ballInfo.posImage.y - ballInfo.bboxHeight / 2);
    ballMax = (ballInfo.posImage.y + ballInfo.bboxHeight / 2);
  } else {
    ballMin = (ballInfo.posImage.x - ballInfo.bboxWidth / 2);
    ballMax = (ballInfo.posImage.x + ballInfo.bboxWidth / 2);
  }
  vector<boost::shared_ptr<LinearScannedLine> > newRobotLines;
  for (auto& rl : robotLines) {
    if (!rl) continue;
    if (rl->start <= ballMin) {
      if (rl->end <= ballMax) {
        rl->end = ballMin;
      } else {
        newRobotLines.push_back(
          boost::make_shared<LinearScannedLine> (
            ballMax, rl->end, rl->baseIndex, rl->end - ballMax, rl->direction));
        rl->end = ballMin;
      }
    } else {
      if (rl->end <= ballMax) {
        rl.reset(); ///< completely in ball region
      } else if (rl->start <= ballMax) { ///< start in ball region
        rl->start = ballMax;
      }
    }
  }
  robotLines.insert(robotLines.begin(), newRobotLines.begin(), newRobotLines.end());
}

void RobotExtraction::filterRobotLines()
{
  auto tStart = high_resolution_clock::now();
  verRobotLines = regionSeg->getVerticalScan(ScanTypes::robot)->scanLines;
  horRobotLines = regionSeg->getHorizontalScan(ScanTypes::robot)->scanLines;
  if (activeCamera == CameraId::headTop) {
    border = fieldExt->getBorder();
    ///< Filter with border
    static const auto jerseyBelowBorderMin = 5;
    static const auto jerseyAboveBorderRelCutoff = 50;
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

  ///< Filter with ball regions
  filterBallRegions(verRobotLines, true);
  filterBallRegions(horRobotLines, false);

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

void RobotExtraction::findJerseys()
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
    regionSeg->getVerticalScan(ScanTypes::ourJersey)->highStep * 1.5; // pixels
  auto verLineLinkYTol = verLineLinkXTol;
  auto horLineLinkXTol =
    regionSeg->getHorizontalScan(ScanTypes::ourJersey)->highStep * 1.5; // pixels
  auto horLineLinkYTol = horLineLinkXTol;
  auto maxLineLengthDiffRatio = 2.5;
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
  //ScannedRegion::drawRegions(
  //  bgrMat[toUType(activeCamera)], verJerseyRegionsOurs, Scalar(255,0,0), 2);
  //ScannedRegion::drawRegions(
  //  bgrMat[toUType(activeCamera)], horJerseyRegionsOurs, Scalar(0,0,255), 2);
  //regionSeg->getVerticalScan(ScanTypes::ourJersey)->draw(bgrMat[toUType(activeCamera)]);
  //regionSeg->getHorizontalScan(ScanTypes::ourJersey)->draw(bgrMat[toUType(activeCamera)]);
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
  static const auto regionsXDiffTol = 36; // pixels
  static const auto regionsYDiffTol = 36; // pixels
  static const auto maxRegionSizeDiffRatio = 2.5;
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
  //ScannedRegion::drawRegions(
  //  bgrMat[toUType(activeCamera)], oursFiltered, Scalar(255,0,0), 2);
  //ScannedRegion::drawRegions(
  //  bgrMat[toUType(activeCamera)], oppsFiltered, Scalar(0,0,255), 2);
  for (const auto& jr : oursFiltered) {
    if (jr) {
      if (jr->rect.width > 5 * jr->rect.height ||
          jr->rect.height > 5 * jr->rect.width)
        continue;
      jerseyRegions.push_back(boost::make_shared<RobotRegion>(jr, true));
      if (GET_DVAR(int, drawJerseyRegions))
        jr->draw(bgrMat[toUType(activeCamera)], colorToBgr[toUType(ourColor)]);
    }
  }
  for (const auto& jr : oppsFiltered) {
    if (jr) {
      if (jr->rect.width > 5 * jr->rect.height ||
          jr->rect.height > 5 * jr->rect.width)
        continue;
      jerseyRegions.push_back(boost::make_shared<RobotRegion>(jr, false));
      if (GET_DVAR(int, drawJerseyRegions))
        jr->draw(bgrMat[toUType(activeCamera)], colorToBgr[toUType(oppColor)]);
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  findJerseysTime = timeSpan.count();
}

void RobotExtraction::classifyRobots()
{
  auto tStart = high_resolution_clock::now();
  for (const auto& frr : filtRobotRegions) {
    if (frr) frr->refresh = false;
  }

  auto iter = jerseyRegions.begin();
  ///< Remove deleted regions
  while (iter != jerseyRegions.end()) {
    if ((*iter)) ++iter;
    else iter = jerseyRegions.erase(iter);
  }

  sort(jerseyRegions.begin(), jerseyRegions.end(), [](
    const RobotRegionPtr& rr1,
    const RobotRegionPtr& rr2)
  { return rr1->sr->center.x < rr2->sr->center.x;});

  static const auto bodyToJerseyMinHeightRatio = 1.5;
  static const auto bodyToJerseyMinWidthRatio = 1.5;
  vector<RobotRegionPtr> robotRegions;
  for (auto& rr : jerseyRegions) {
    if (!rr) continue;
    ScannedRegionPtr robotFound;
    //if (rr->sr->rect.area() > 1e9) {
    //  robotFound = rr->sr;
    //  rr->posFromJersey = true;
    //} else if (rr->sr->rect.area() > 1000) {
    if (rr->sr->rect.area() > 1000) {
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
          rr->fallen = true;
          robotFound = boost::make_shared<ScannedRegion> (bodyRect);
          //region found
        }
      }
    } else {
      // Small area
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
          rr->fallen = true;
          robotFound = boost::make_shared<ScannedRegion> (bodyRect);
          // region found
        }
      }
    }

    if (robotFound) {
      // If robot is too close and its legs are not showing
      auto robotMaxY = robotFound->rect.y + robotFound->rect.height;
      //cout << "robotMaxY:" << robotMaxY << endl;
      //cout << "fallen: " << rr->fallen << endl;
      if (getImageHeight() - robotMaxY < 10 && !rr->fallen) {
        Rect updatedJersey;
        updatedJersey.x = robotFound->leftBase.x;
        updatedJersey.width = robotFound->rightBase.x - updatedJersey.x;
        updatedJersey.y = rr->sr->rect.y;
        updatedJersey.height = rr->sr->rect.height;
        rr->sr = boost::make_shared<ScannedRegion> (updatedJersey);
        rr->posFromJersey = true;
      } else {
        rr->sr = robotFound;
      }

      ///< Find world position and verify robot width in world
      if (rr->posFromJersey) {
        // Jersey center with approximate height of 35 cms
        cameraTransforms[toUType(activeCamera)]->imageToWorld(
          rr->frontLeft, Point2f(rr->sr->leftBase.x, rr->sr->leftBase.y), JERSEY_APPROX_HEIGHT);
        cameraTransforms[toUType(activeCamera)]->imageToWorld(
          rr->frontRight, Point2f(rr->sr->rightBase.x, rr->sr->rightBase.y), JERSEY_APPROX_HEIGHT);
        cameraTransforms[toUType(activeCamera)]->imageToWorld(
          rr->world, rr->sr->center, JERSEY_APPROX_HEIGHT);
      } else {
        cameraTransforms[toUType(activeCamera)]->imageToWorld(
          rr->frontLeft, Point2f(rr->sr->leftBase.x, rr->sr->leftBase.y), 0.0);
        cameraTransforms[toUType(activeCamera)]->imageToWorld(
          rr->frontRight, Point2f(rr->sr->rightBase.x, rr->sr->rightBase.y), 0.0);
        cameraTransforms[toUType(activeCamera)]->imageToWorld(
          rr->world,
          Point2f(robotFound->center.x, robotFound->leftBase.y),
          0.0);
      }
      auto baseWidth = cv::norm(rr->frontRight - rr->frontLeft);
      //cout << "baseWidth: " << baseWidth << endl;
      if (baseWidth > ROBOT_WIDTH && !rr->fallen) {
        rr.reset();
      } else if (baseWidth > FALLEN_ROBOT_WIDTH && rr->fallen) {
        rr.reset();
      }

      if (rr) {
        //cout << "rr->world:" << rr->world << endl;
        //rr->sr->draw(bgrMat[toUType(activeCamera)], Scalar(255,255,255));
        //robotFound->draw(bgrMat[toUType(activeCamera)], Scalar(0,255,255));
        // Update new regions and check if a robot already exists in the region
        auto closest = 1000;
        RobotRegionPtr prevRegionMatch;
        for (const auto& frl : filtRobotRegions) {
          if (frl &&
              rr->ourTeam == frl->ourTeam &&
              rr->fallen == frl->fallen)
          {
            auto dist = cv::norm(rr->world - frl->world);
            //cout << "dist:" << dist << endl;
            if (dist < ROBOT_MATCH_MAX_DISTANCE && dist < closest) {
              closest = dist;
              prevRegionMatch = frl;
            }
          }
        }
        if (prevRegionMatch) {
          // Update previous detected region
          //prevRegionMatch->sr->draw(bgrMat[toUType(activeCamera)], Scalar(255,0,0), 3);
          prevRegionMatch = rr;
          prevRegionMatch->refresh = true;
        } else {
          // Add newly detected region
          robotRegions.push_back(rr);
        }
        //VisionUtils::displayImage("Region", bgrMat[toUType(activeCamera)]);
        //waitKey(0);
      }
    }
  }
  if (!robotRegions.empty()) {
    filtRobotRegions.insert(
      filtRobotRegions.end(),
      robotRegions.begin(),
      robotRegions.end());
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  classifyRobotsTime = timeSpan.count();
}

void RobotExtraction::findStrayRegions()
{
  auto tStart = high_resolution_clock::now();
  if (activeCamera == CameraId::headTop) {
    for (auto& rl : verRobotLines) {
      if (rl) {
        auto borderY = border[rl->baseIndex];
        if (rl->start < borderY) {
          if (rl->end - borderY > 5) rl->start = borderY;
          else rl.reset();
        }
      }
    }
  }
  vector<ScannedRegionPtr> verStrayRegions;
  vector<ScannedRegionPtr> verStrayRegionsF;
  findRegions(verStrayRegions, verRobotLines, 24, 24, 2, bgrMat[toUType(activeCamera)]);
  ScannedRegion::linkRegions(verStrayRegionsF, verStrayRegions, 24, 12, 2.5, bgrMat[toUType(activeCamera)]);
  for (const auto& r : verStrayRegionsF)
    strayRegions.push_back(r->rect);
  if (GET_DVAR(int, drawStrayRegions))
    ScannedRegion::drawRegions(bgrMat[toUType(activeCamera)], verStrayRegionsF, Scalar(0, 0, 0));
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  findStratRegionsTime = timeSpan.count();
}

void RobotExtraction::updateRobotsInfo()
{
  auto tStart = high_resolution_clock::now();
  float currentTime = visionModule->getModuleTime();
  for (const auto& frl : filtRobotRegions) {
    if (!frl) continue;
    if (frl->refresh) frl->timeDetected = currentTime;
    ObstacleType type = ObstacleType::unknown;
    if (frl->ourTeam) {
      if (!frl->fallen) type = ObstacleType::teammate;
      else type = ObstacleType::teammateFallen;
    } else {
      if (!frl->fallen) type = ObstacleType::opponent;
      else type = ObstacleType::opponentFallen;
    }
    Obstacle<float> obstacle(type);
    obstacle.center = frl->world;
    obstacle.front.p1 = frl->frontLeft;
    obstacle.front.p2 = frl->frontRight;
    obstacle.front.setup();
    obstacle.back =
      TNRSLine<float>(
        obstacle.front.p1 + obstacle.front.perp * obstacle.depth,
        obstacle.front.p2 + obstacle.front.perp * obstacle.depth
      );
    const auto& robotPose2D = ROBOT_POSE_2D_IN(VisionModule);
    obstacle.frontT =
      TNRSLine<float>(
        robotPose2D.transform(obstacle.front.p1),
        robotPose2D.transform(obstacle.front.p2)
      );
    obstacle.backT =
      TNRSLine<float>(
        robotPose2D.transform(obstacle.back.p1),
        robotPose2D.transform(obstacle.back.p2)
      );
    OBSTACLES_OBS_OUT(VisionModule).data.push_back(obstacle);
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  updateRobotsInfoTime = timeSpan.count();
}

void RobotExtraction::drawResults()
{
  if (GET_DVAR(int, drawRobotRegions)) {
    for (const auto& frl : filtRobotRegions) {
      if (frl && frl->ourTeam)
        frl->sr->draw(bgrMat[toUType(activeCamera)], colorToBgr[toUType(ourColor)]);
      else
        frl->sr->draw(bgrMat[toUType(activeCamera)], colorToBgr[toUType(oppColor)]);
    }
  }
}
