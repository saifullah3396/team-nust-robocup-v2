/**
 * @file FeatureExtraction/RobotExtraction.h
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
#include "VisionModule/include/FeatureExtraction/RegionSegmentation.h"
#include "VisionModule/include/FeatureExtraction/RobotExtraction.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/RobotRegion.h"
#include "VisionModule/include/VisionExceptions.h"

RobotExtraction::RobotExtraction(VisionModule* visionModule) :
  FeatureExtraction(visionModule),
  DebugBase("RobotExtraction", this)
{
  //! Set up debugging variables
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

  //! Load ball classifier
  // Not used anymore
  //try {
  //  loadRobotClassifier();
  //} catch (VisionException& e) {
  //  LOG_EXCEPTION(e.what());
  //}

  //! Get other feature extraction classes
  fieldExt = GET_FEATURE_EXT_CLASS(FieldExtraction, FeatureExtractionIds::field);
  regionSeg = GET_FEATURE_EXT_CLASS(RegionSegmentation, FeatureExtractionIds::segmentation);

  //! Initializing processing times
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
        true,
        EXC_ROBOT_CLASSIFIER_NOT_FOUND
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
    VisionUtils::displayImage("RobotExtraction", bgrMat[currentImage]);
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
  /*cout << "OBSTACLES_OBS: " << OBSTACLES_OBS.data.size() << endl;
  for(size_t i = 0; i < OBSTACLES_OBS.data.size(); ++i) {
    cout << (int)OBSTACLES_OBS.data[i].type << endl;
    cout << OBSTACLES_OBS.data[i].center << endl;
    cout << OBSTACLES_OBS.data[i].leftBound << endl;
    cout << OBSTACLES_OBS.data[i].rightBound << endl;
    cout << OBSTACLES_OBS.data[i].depth << endl;
  }*/
  //VisionUtils::displayImage(bgrMat[currentImage], "robots");
  /*#ifdef DEBUG_BUILD
   if (GET_DVAR(int, sendTime)) {
   high_resolution_clock::time_point tEnd =
   high_resolution_clock::now();
   duration<double> timeSpan = tEnd - tStart;
   CommModule::addToLogMsgQueue("RobotExtraction time: " +
   DataUtils::varToString(timeSpan.count()) + " seconds.");
   }
   #endif*/
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

void RobotExtraction::filterRobotLines()
{
  auto tStart = high_resolution_clock::now();
  border = fieldExt->getBorder();
  verRobotLines = regionSeg->getVerRobotLines();
  horRobotLines = regionSeg->getHorRobotLines();

  for (int i = 0; i < verRobotLines.size(); ++i) {
    auto rl = verRobotLines[i];
    if (rl->p1.y < border[rl->p1.x].y) {
      if (rl->p2.y - border[rl->p2.x].y > 5) {
        rl->p1.y = border[rl->p2.x].y - 50;
      } else {
        verRobotLines[i].reset();
      }
    }
  }

  /*for (int i = 0; i < horRobotLines.size(); ++i) {
   auto rl = horRobotLines[i];
   if(rl->p1.y < border[rl->p1.x].y) {
   if (rl->p2.y - border[rl->p2.x].y < 10) {
   horRobotLines[i].reset();
   } else {
   for (int x = 0; x < border.size(); ++x) {
   if (border[x].y == rl->p1.y)
   rl->p1 = border[x];
   }
   }
   } else if (rl->p2.y < border[rl->p2.x].y) {
   if (rl->p1.y - border[rl->p1.x].y < 10) {
   horRobotLines[i].reset();
   } else {
   for (int x = 0; x < border.size(); ++x) {
   if (border[x].y == rl->p2.y)
   rl->p2 = border[x];
   }
   }
   }
   }*/

  if (GET_DVAR(int, drawScannedLines)) {
    for (int i = 0; i < horRobotLines.size(); ++i) {
      if (horRobotLines[i])
        line(
          bgrMat[currentImage],
          horRobotLines[i]->p1,
          horRobotLines[i]->p2,
          Scalar(0, 0, 0),
          1);
    }

     for (int i = 0; i < verRobotLines.size(); ++i) {
      if (verRobotLines[i])
        line(
          bgrMat[currentImage],
          verRobotLines[i]->p1,
          verRobotLines[i]->p2,
          Scalar(0, 0, 0),
          1);
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  linesFilterTime = timeSpan.count();
}

void RobotExtraction::findJerseys()
{
  auto tStart = high_resolution_clock::now();
  auto verJerseyLinesOurs = regionSeg->getVerJerseyLinesOurs();
  auto horJerseyLinesOurs = regionSeg->getHorJerseyLinesOurs();
  auto verJerseyLinesOpps = regionSeg->getVerJerseyLinesOpps();
  auto horJerseyLinesOpps = regionSeg->getHorJerseyLinesOpps();

  filterJerseys(border, verJerseyLinesOurs);
  filterJerseys(border, horJerseyLinesOurs);
  filterJerseys(border, verJerseyLinesOpps);
  filterJerseys(border, horJerseyLinesOpps);

  vector<ScannedRegionPtr> verJerseyRegionsOurs;
  vector<ScannedRegionPtr> horJerseyRegionsOurs;
  vector<ScannedRegionPtr> verJerseyRegionsOpps;
  vector<ScannedRegionPtr> horJerseyRegionsOpps;
  findRegions(verJerseyRegionsOurs, verJerseyLinesOurs, 36, 36, false);
  findRegions(horJerseyRegionsOurs, horJerseyLinesOurs, 36, 36, true);
  findRegions(verJerseyRegionsOpps, verJerseyLinesOpps, 36, 36, false);
  findRegions(horJerseyRegionsOpps, horJerseyLinesOpps, 36, 36, true);
  vector<ScannedRegionPtr> verJOursFiltered;
  vector<ScannedRegionPtr> horJOursFiltered;
  vector<ScannedRegionPtr> verJOppsFiltered;
  vector<ScannedRegionPtr> horJOppsFiltered;
  ScannedRegion::linkRegions(verJOursFiltered, verJerseyRegionsOurs, 36, 36);
  ScannedRegion::linkRegions(horJOursFiltered, horJerseyRegionsOurs, 36, 36);
  ScannedRegion::linkRegions(verJOppsFiltered, verJerseyRegionsOpps, 36, 36);
  ScannedRegion::linkRegions(horJOppsFiltered, horJerseyRegionsOpps, 36, 36);
  horJOursFiltered.insert(
    horJOursFiltered.end(),
    verJOursFiltered.begin(),
    verJOursFiltered.end());
  horJOppsFiltered.insert(
    horJOppsFiltered.end(),
    verJOppsFiltered.begin(),
    verJOppsFiltered.end());
  filterScannedRegions(horJOursFiltered, 36);
  filterScannedRegions(horJOppsFiltered, 36);
  for (int i = 0; i < horJOursFiltered.size(); ++i) {
    if (horJOursFiltered[i]) {
      auto jr = horJOursFiltered[i];
      int w = jr->rect.width;
      int h = jr->rect.height;
      if (w > 5 * h || h > 5 * w) continue;
      jerseyRegions.push_back(boost::make_shared < RobotRegion > (jr, true));
      if (GET_DVAR(int, drawJerseyRegions))
        jr->draw(bgrMat[currentImage], Scalar(0, 255, 255), 2);
    }
  }
  for (int i = 0; i < horJOppsFiltered.size(); ++i) {
    if (horJOppsFiltered[i]) {
      auto jr = horJOppsFiltered[i];
      int w = jr->rect.width;
      int h = jr->rect.height;
      if (w > 5 * h || h > 5 * w) continue;
      jerseyRegions.push_back(boost::make_shared <RobotRegion> (jr, false));
      if (GET_DVAR(int, drawJerseyRegions))
        jr->draw(bgrMat[currentImage], Scalar(255, 0, 0), 2);
    }
  }
  sort(jerseyRegions.begin(), jerseyRegions.end(), [](
    const RobotRegionPtr& rr1,
    const RobotRegionPtr& rr2)
  { return rr1->sr->center.x < rr2->sr->center.x;});
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  findJerseysTime = timeSpan.count();
}

void RobotExtraction::filterJerseys(const vector<Point>& border,
  vector<ScannedLinePtr>& jerseyLines)
{
  for (int i = 0; i < jerseyLines.size(); ++i) {
    auto jl = jerseyLines[i];
    if (jl->p1.y < border[jl->p1.x].y) {
      if (border[jl->p1.x].y - jl->p1.y > 200) {
        jerseyLines[i].reset();
      }
    }
  }
}

void RobotExtraction::classifyRobots()
{
  auto tStart = high_resolution_clock::now();
  for (int i = 0; i < filtRobotRegions.size(); ++i) {
    if (filtRobotRegions[i])
      filtRobotRegions[i]->refresh = false;
  }

  vector<RobotRegionPtr> robotRegions;
  for (int i = 0; i < jerseyRegions.size(); ++i) {
    auto rr = jerseyRegions[i];
    ScannedRegionPtr robotFound;
    //drawRegion(bgrMat[currentImage], rr->sr, Scalar(255, 255, 255));
    //cout << "Area: " << rr->sr->rect.area() << endl;
    if (rr->sr->rect.area() > 10000) {
      robotFound = rr->sr;
      rr->posFromJersey = true;
    } else if (rr->sr->rect.area() > 1000) {
      // Large area
      if (rr->sr->rect.height >= rr->sr->rect.width) {
        int halfHeight = rr->sr->rect.height / 2;
        int minY = rr->sr->rect.y - halfHeight;
        int maxY = rr->sr->leftBase.y + halfHeight;
        int minX = rr->sr->leftBase.x - rr->sr->rect.width;
        int maxX = rr->sr->rightBase.x + rr->sr->rect.width;
        vector<ScannedLinePtr> bodyLines;
        for (int j = 0; j < verRobotLines.size(); ++j) {
          if (!verRobotLines[j]) continue;
          auto vl = verRobotLines[j];
          if (vl->p1.x > minX && vl->p1.x < maxX) {
            if (vl->p1.y > minY && vl->p1.y < maxY)
              bodyLines.push_back(vl);
          }
        }
        vector < Point > verPts;
        if (bodyLines.empty()) continue;
        for (int j = 0; j < bodyLines.size(); ++j) {
          verPts.push_back(bodyLines[j]->p1);
          verPts.push_back(bodyLines[j]->p2);
        }
        Rect bodyRect = boundingRect(verPts) | rr->sr->rect;
        float hRatio = bodyRect.height / ((float) rr->sr->rect.height);
        if (hRatio > 1.5) {
          robotFound = boost::make_shared < ScannedRegion > (bodyRect);
          //drawRegion(bgrMat[currentImage], robotFound, Scalar(255,0,0));
          //region found
        }
      } else {
        int minY = rr->sr->rect.y - rr->sr->rect.height;
        int maxY = rr->sr->leftBase.y + rr->sr->rect.height;
        int minX = rr->sr->leftBase.x - rr->sr->rect.width;
        int maxX = rr->sr->rightBase.x + rr->sr->rect.width;
        vector<ScannedLinePtr> bodyLines;
        for (int j = 0; j < horRobotLines.size(); ++j) {
          if (!horRobotLines[j]) continue;
          auto hl = horRobotLines[j];
          if (hl->p1.x > minX && hl->p1.x < maxX) {
            if (hl->p1.y > minY && hl->p1.y < maxY) bodyLines.push_back(hl);
          } else if (hl->p2.x > minX && hl->p2.x < maxX) {
            if (hl->p2.y > minY && hl->p2.y < maxY) bodyLines.push_back(hl);
          }
        }
        vector < Point > horPts;
        if (bodyLines.empty()) continue;
        for (int j = 0; j < bodyLines.size(); ++j) {
          horPts.push_back(bodyLines[j]->p1);
          horPts.push_back(bodyLines[j]->p2);
        }
        Rect bodyRect = boundingRect(horPts) | rr->sr->rect;
        float wRatio = bodyRect.width / ((float) rr->sr->rect.width);
        if (wRatio > 1.5) {
          rr->fallen = true;
          robotFound = boost::make_shared < ScannedRegion > (bodyRect);
          //drawRegion(bgrMat[currentImage], robotFound, Scalar(0,255,0));
          //region found
        }
      }
    } else {
      // Small area
      int doubleWidth = rr->sr->rect.width * 2;
      int minY = rr->sr->rect.y - rr->sr->rect.height;
      int maxY = rr->sr->leftBase.y + rr->sr->rect.height;
      int minX = rr->sr->leftBase.x - doubleWidth;
      int maxX = rr->sr->rightBase.x + doubleWidth;
      vector<ScannedLinePtr> bodyLines;
      for (int j = 0; j < verRobotLines.size(); ++j) {
        if (!verRobotLines[j]) continue;
        auto vl = verRobotLines[j];
        if (vl->p1.x > minX && vl->p1.x < maxX) {
          if (vl->p1.y > minY && vl->p1.y < maxY) bodyLines.push_back(vl);
        }
      }
      vector < Point > verPts;
      if (bodyLines.empty()) continue;
      for (int j = 0; j < bodyLines.size(); ++j) {
        verPts.push_back(bodyLines[j]->p1);
        verPts.push_back(bodyLines[j]->p2);
      }
      Rect bodyRect = boundingRect(verPts) | rr->sr->rect;
      float hRatio = bodyRect.height / ((float) rr->sr->rect.height);
      if (hRatio > 1.5) {
        robotFound = boost::make_shared < ScannedRegion > (bodyRect);
        //drawRegion(bgrMat[currentImage], robotFound, Scalar(0,0,255));
        // region found
      } else {
        minX = rr->sr->leftBase.x - doubleWidth;
        maxX = rr->sr->rightBase.x + doubleWidth;
        vector < ScannedLinePtr > bodyLines;
        for (int j = 0; j < horRobotLines.size(); ++j) {
          if (!horRobotLines[j]) continue;
          auto hl = horRobotLines[j];
          if (hl->p1.x > minX && hl->p1.x < maxX) {
            if (hl->p1.y > minY && hl->p1.y < maxY) bodyLines.push_back(hl);
          } else if (hl->p2.x > minX && hl->p2.x < maxX) {
            if (hl->p2.y > minY && hl->p2.y < maxY) bodyLines.push_back(hl);
          }
        }
        vector < Point > horPts;
        if (bodyLines.empty()) continue;
        for (int j = 0; j < bodyLines.size(); ++j) {
          horPts.push_back(bodyLines[j]->p1);
          horPts.push_back(bodyLines[j]->p2);
        }
        Rect bodyRect = boundingRect(horPts) | rr->sr->rect;
        float wRatio = bodyRect.width / ((float) rr->sr->rect.width);
        if (wRatio > 1.5f) {
          rr->fallen = true;
          robotFound = boost::make_shared < ScannedRegion > (bodyRect);
          //drawRegion(bgrMat[currentImage], robotFound, Scalar(255,255,0));
          // region found
        }
      }
    }
    if (robotFound) {
      // If robot is too close and its legs are not showing
      int robotMaxY = robotFound->rect.y + robotFound->rect.height;
      if (getImageHeight() - robotMaxY < 10 && !rr->fallen) {
        Rect updatedJersey;
        updatedJersey.x = robotFound->leftBase.x;
        updatedJersey.width = robotFound->rightBase.x - updatedJersey.x;
        updatedJersey.y = rr->sr->rect.y;
        updatedJersey.height = rr->sr->rect.height;
        rr->sr = boost::make_shared < ScannedRegion > (updatedJersey);
        rr->posFromJersey = true;
      } else {
        rr->sr = robotFound;
      }

      if (rr->posFromJersey) {
        // Jersey center with approximate height of 40 cms
        cameraTransforms[currentImage]->imageToWorld(
          rr->world,
          rr->sr->center,
          0.35);
      } else {
        cameraTransforms[currentImage]->imageToWorld(
          rr->world,
          Point2f(robotFound->center.x, robotFound->leftBase.y),
          0.0);
      }
      // Update new regions and check if a robot already exists in the region
      float closest = 1000;
      int minIndex = -1;
      for (int i = 0; i < filtRobotRegions.size(); ++i) {
        if (!filtRobotRegions[i]) continue;
        float dist = norm(rr->world - filtRobotRegions[i]->world);
        if (dist < closest) {
          closest = dist;
          minIndex = i;
        }
      }
      if (minIndex != -1 && closest < filtRobotRegions[minIndex]->minDistValidation) {
        // Update previous detected region
        filtRobotRegions[minIndex] = rr;
        filtRobotRegions[minIndex]->refresh = true;
      } else {
        // Add newly detected region
        robotRegions.push_back(rr);
      }
    }
  }
  filtRobotRegions.insert(
    filtRobotRegions.end(),
    robotRegions.begin(),
    robotRegions.end());
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  classifyRobotsTime = timeSpan.count();
}

void RobotExtraction::findStrayRegions()
{
  auto tStart = high_resolution_clock::now();
  for (int i = 0; i < verRobotLines.size(); ++i) {
    if (!verRobotLines[i]) continue;
    auto rl = verRobotLines[i];
    if (rl->p1.y < border[rl->p1.x].y) {
      if (rl->p2.y - border[rl->p2.x].y > 5) {
        rl->p1.y = border[rl->p2.x].y;
      } else {
        verRobotLines[i].reset();
      }
    }
  }
  vector<ScannedRegionPtr> verStrayRegions;
  vector<ScannedRegionPtr> verStrayRegionsF;
  //for (int i = 0; i < verRobotLines.size(); ++i) {
  //  if (verRobotLines[i])
  //    line(bgrMat[currentImage], verRobotLines[i]->p1, verRobotLines[i]->p2, Scalar(255,0,0), 2);
  //}
  findRegions(verStrayRegions, verRobotLines, 18, 18, false);
  //drawRegions(bgrMat[currentImage], verStrayRegions, Scalar(0,0,0));
  ScannedRegion::linkRegions(verStrayRegionsF, verStrayRegions, 36, 36);
  for (int i = 0; i < verStrayRegionsF.size(); ++i) {
    Rect r = verStrayRegionsF[i]->rect;
    r = r - Point(r.width / 2, 0);
    r += Size(r.width, 0);
    r = r & Rect(0, 0, getImageWidth(), getImageHeight());
    verStrayRegionsF[i]->rect = r;
    strayRegions.push_back(verStrayRegionsF[i]->rect);
  }
  if (GET_DVAR(int, drawStrayRegions))
    ScannedRegion::drawRegions(bgrMat[currentImage], verStrayRegionsF, Scalar(255, 0, 255));
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  findStratRegionsTime = timeSpan.count();
}

void RobotExtraction::updateRobotsInfo()
{
  auto tStart = high_resolution_clock::now();
  for (int i = 0; i < filtRobotRegions.size(); ++i) {
    if (!filtRobotRegions[i]) continue;
    for (int j = 0; j < filtRobotRegions.size(); ++j) {
      if (!filtRobotRegions[j]) continue;
      if (i != j) {
        float dist = abs(
          filtRobotRegions[i]->sr->center.x - filtRobotRegions[j]->sr->center.x);
        if (dist < 15 && filtRobotRegions[i]->ourTeam == filtRobotRegions[j]->ourTeam) { // 15 pixels difference of centers
          filtRobotRegions[j].reset();
        }
      }
    }
  }
  float currentTime = visionModule->getModuleTime();
  for (int i = 0; i < filtRobotRegions.size(); ++i) {
    if (!filtRobotRegions[i]) continue;
    if (filtRobotRegions[i]->refresh) filtRobotRegions[i]->timeDetected = currentTime;
    ObstacleType type = ObstacleType::unknown;
    if (filtRobotRegions[i]->ourTeam) {
      if (!filtRobotRegions[i]->fallen) type = ObstacleType::teammate;
      else type = ObstacleType::teammateFallen;
    } else {
      if (!filtRobotRegions[i]->fallen) type = ObstacleType::opponent;
      else type = ObstacleType::opponentFallen;
    }
    Obstacle<float> obstacle(type);
    obstacle.center = filtRobotRegions[i]->world;
    if (filtRobotRegions[i]->posFromJersey) {
      auto left = Point(
        filtRobotRegions[i]->sr->leftBase.x,
        filtRobotRegions[i]->sr->center.y);
      auto right = Point(
        filtRobotRegions[i]->sr->rightBase.x,
        filtRobotRegions[i]->sr->center.y);
      cameraTransforms[currentImage]->imageToWorld(
        obstacle.front.p1, left, 0.35);
      cameraTransforms[currentImage]->imageToWorld(
        obstacle.front.p2, right, 0.35);
    } else {
      cameraTransforms[currentImage]->imageToWorld(
        obstacle.front.p1, filtRobotRegions[i]->sr->leftBase, 0.0);
      cameraTransforms[currentImage]->imageToWorld(
        obstacle.front.p2, filtRobotRegions[i]->sr->rightBase, 0.0);
    }
    obstacle.front.setup();
    obstacle.back =
      TNRSLine<float>(
        obstacle.front.p1 + obstacle.front.perp * obstacle.depth,
        obstacle.front.p2 + obstacle.front.perp * obstacle.depth
      );
    auto robotPose2D = ROBOT_POSE_2D_IN(VisionModule);
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
    for (int i = 0; i < filtRobotRegions.size(); ++i) {
      if (!filtRobotRegions[i]) continue;
      auto rr = filtRobotRegions[i];
      if (rr->ourTeam)
        rectangle(bgrMat[currentImage], rr->sr->rect, Scalar(0, 255, 255), 2);
      else
        rectangle(bgrMat[currentImage], rr->sr->rect, Scalar(255, 0, 0), 2);
    }
  }
}
