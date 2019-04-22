/**
 * @file VisionModule/src/FeatureExtraction/RegionSegmentation.cpp
 *
 * This file implements the class for segmenting the image into different
 * regions for further processing.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author AbdulRehman
 * @date 5 Mar 2018
 */

#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/BallExtraction.h"
#include "VisionModule/include/FeatureExtraction/GoalExtraction.h"
#include "VisionModule/include/FeatureExtraction/RegionSegmentation.h"
#include "VisionModule/include/FeatureExtraction/RegionScanners.h"
#include "VisionModule/include/ColorHandler.h"
#include "Utils/include/ConfigManager.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/EnumUtils.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/VisionUtils.h"

RegionSegmentation::RegionSegmentation(VisionModule* visionModule) :
  FeatureExtraction(visionModule, "RegionSegmentation"),
  DebugBase("RegionSegmentation", this)
{
  initDebugBase();
  getDebugVars();

  ourColor = TNColors::yellow;
  oppColor = TNColors::blue;

  auto top = toUType(CameraId::headTop);
  auto bottom = toUType(CameraId::headBottom);
  hScans.resize(toUType(ScanTypes::count));
  vScans.resize(toUType(ScanTypes::count));
  hMinScanStepLow.resize(toUType(CameraId::count));
  vMinScanStepLow.resize(toUType(CameraId::count));
  hScanStepSizes.resize(toUType(CameraId::count));
  vScanStepSizes.resize(toUType(CameraId::count));
  hScanStepHigh.resize(toUType(CameraId::count));
  vScanStepHigh.resize(toUType(CameraId::count));
  hScanStepSizes[top].resize(toUType(ScanTypes::count));
  vScanStepSizes[bottom].resize(toUType(ScanTypes::count));

  auto root =
    JsonUtils::readJson(ConfigManager::getCommonConfigDirPath() + "VisionConfig.json");
  JsonUtils::jsonToType(hScanStepHigh, root["hScanStepHigh"], vector<int>());
  JsonUtils::jsonToType(vScanStepHigh, root["vScanStepHigh"], vector<int>());
  JsonUtils::jsonToType(hScanStepSizes[top], root["upperCamHScanSteps"], vector<int>());
  JsonUtils::jsonToType(vScanStepSizes[top], root["upperCamVScanSteps"], vector<int>());
  JsonUtils::jsonToType(hScanStepSizes[bottom], root["lowerCamHScanSteps"], vector<int>());
  JsonUtils::jsonToType(vScanStepSizes[bottom], root["lowerCamVScanSteps"], vector<int>());
  hMinScanStepLow[top] = *std::min_element(begin(hScanStepSizes[top]), end(hScanStepSizes[top]));
  vMinScanStepLow[top] = *std::min_element(begin(vScanStepSizes[top]), end(vScanStepSizes[top]));
  hMinScanStepLow[bottom] = *std::min_element(begin(hScanStepSizes[bottom]), end(hScanStepSizes[bottom]));
  vMinScanStepLow[bottom] = *std::min_element(begin(vScanStepSizes[bottom]), end(vScanStepSizes[bottom]));

  auto defaultCamera = top;
  //auto ballType = root["ballType"].asInt();
  //hScans[toUType(ScanTypes::field)] =
    //new FieldScan(vScanStepSizes[defaultCamera][toUType(ScanTypes::field)], hScanStepHigh[defaultCamera], false);
  hScans[toUType(ScanTypes::robot)] =
    new RobotScan(hScanStepSizes[defaultCamera][toUType(ScanTypes::robot)], hScanStepHigh[defaultCamera], false);
  //hScans[toUType(ScanTypes::ball)] = hScans[toUType(ScanTypes::robot)];
  //hScans[toUType(ScanTypes::ball)] =
//    new BallScan(hScanStepSizes[defaultCamera][toUType(ScanTypes::ball)], hScanStepHigh[defaultCamera], false);
  hScans[toUType(ScanTypes::ourJersey)] =
    new JerseyScan(ourColor, hScanStepSizes[defaultCamera][toUType(ScanTypes::ourJersey)], hScanStepHigh[defaultCamera], false);
  hScans[toUType(ScanTypes::oppJersey)] =
    new JerseyScan(oppColor, hScanStepSizes[defaultCamera][toUType(ScanTypes::oppJersey)], hScanStepHigh[defaultCamera], false);
  hScans[toUType(ScanTypes::lines)] =
    new LinesScan(hScanStepSizes[defaultCamera][toUType(ScanTypes::lines)], hScanStepHigh[defaultCamera], false);
  hScans[toUType(ScanTypes::goal)] =
    new GoalScan(hScanStepSizes[defaultCamera][toUType(ScanTypes::goal)], hScanStepHigh[defaultCamera], false);

  vScans[toUType(ScanTypes::field)] =
    new FieldScan(vScanStepSizes[defaultCamera][toUType(ScanTypes::field)], vScanStepHigh[defaultCamera], true);
  vScans[toUType(ScanTypes::robot)] =
    new RobotScan(vScanStepSizes[defaultCamera][toUType(ScanTypes::robot)], vScanStepHigh[defaultCamera], true);
  //vScans[toUType(ScanTypes::ball)] =
  //  new BallScan(vScanStepSizes[defaultCamera][toUType(ScanTypes::ball)], vScanStepHigh[defaultCamera], true);
  //vScans[toUType(ScanTypes::ball)] = vScans[toUType(ScanTypes::robot)];
  vScans[toUType(ScanTypes::ourJersey)] =
    new JerseyScan(ourColor, vScanStepSizes[defaultCamera][toUType(ScanTypes::ourJersey)], vScanStepHigh[defaultCamera], true);
  vScans[toUType(ScanTypes::oppJersey)] =
    new JerseyScan(oppColor, vScanStepSizes[defaultCamera][toUType(ScanTypes::oppJersey)], vScanStepHigh[defaultCamera], true);
  vScans[toUType(ScanTypes::lines)] =
    new LinesScan(vScanStepSizes[defaultCamera][toUType(ScanTypes::lines)], vScanStepHigh[defaultCamera], true);
  //vScans[toUType(ScanTypes::goal)] =
    //new GoalScan(vScanStepSizes[defaultCamera][toUType(ScanTypes::goal)], vScanStepHigh[defaultCamera], true);

  /*if (ballType == 0) {
    hScans[toUType(ScanTypes::ball)]->color = TNColors::red;
    vScans[toUType(ScanTypes::ball)]->color = TNColors::red;
  } else {
    hScans[toUType(ScanTypes::ball)]->color = TNColors::black;
    vScans[toUType(ScanTypes::ball)]->color = TNColors::black;
  }*/

  for (size_t i = 0; i < hScans.size(); ++i) {
    auto& scan = hScans[i];
    if (scan) {
      scan->enabled = true;
      scan->scanTables.resize(toUType(CameraId::count));
      scan->scanTables[top] = vector<bool>(imageWidth[top], false);
      scan->scanTables[bottom] = vector<bool>(imageWidth[bottom], false);
      for (size_t j = 0; j < scan->scanTables[top].size(); ++j) {
        scan->scanTables[top][j] = j % hScanStepSizes[top][i] == 0;
      }
      for (size_t j = 0; j < scan->scanTables[bottom].size(); ++j) {
        scan->scanTables[bottom][j] = j % hScanStepSizes[bottom][i] == 0;
      }
    }
  }
  for (size_t i = 0; i < vScans.size(); ++i) {
    auto& scan = vScans[i];
    if (scan) {
      scan->enabled = true;
      scan->scanTables.resize(toUType(CameraId::count));
      scan->scanTables[top] = vector<bool>(imageHeight[top], false);
      scan->scanTables[bottom] = vector<bool>(imageHeight[bottom], false);
      for (size_t j = 0; j < scan->scanTables[top].size(); ++j) {
        scan->scanTables[top][j] = j % vScanStepSizes[top][i] == 0;
      }
      for (size_t j = 0; j < scan->scanTables[bottom].size(); ++j) {
        scan->scanTables[bottom][j] = j % vScanStepSizes[bottom][i] == 0;
      }
    }
  }


  ///< Initializing processing times
  horizontalScanTime = 0.0;
  verticalScanTime = 0.0;
  processTime = 0.0;
}

void RegionSegmentation::getDebugVars()
{
  int tempSendTime;
  int tempDrawVerticalLines;
  int tempDrawHorizontalLines;
  int tempDrawPoints;
  int tempDisplayInfo;
  int tempDisplayOutput;
  GET_CONFIG(
    "VisionConfig",
    (int, RegionSegmentation.sendTime, tempSendTime),
    (int, RegionSegmentation.drawHorizontalLines, tempDrawHorizontalLines),
    (int, RegionSegmentation.drawVerticalLines, tempDrawVerticalLines),
    (int, RegionSegmentation.drawPoints, tempDrawPoints),
    (int, RegionSegmentation.displayInfo, tempDisplayInfo),
    (int, RegionSegmentation.displayOutput, tempDisplayOutput),
  );
  SET_DVAR(int, sendTime, tempSendTime);
  SET_DVAR(int, drawVerticalLines, tempDrawVerticalLines);
  SET_DVAR(int, drawHorizontalLines, tempDrawHorizontalLines);
  SET_DVAR(int, drawPoints, tempDrawPoints);
  SET_DVAR(int, displayInfo, tempDisplayInfo);
  SET_DVAR(int, displayOutput, tempDisplayOutput);
}

void RegionSegmentation::processImage()
{
  auto tStart = high_resolution_clock::now();
  reset();
  vScanLimitIdx = getImageHeight() - vMinScanStepLow[toUType(activeCamera)];
  hScanLimitIdx = getImageWidth() - hMinScanStepLow[toUType(activeCamera)];
  horizontalScan();
  verticalScan();
  auto tEnd = high_resolution_clock::now();
  duration<double> timeSpan = tEnd - tStart;
  processTime = timeSpan.count();
  if (GET_DVAR(int, displayInfo)) {
    LOG_INFO("RegionSegmentation Results:");
    LOG_INFO("Time taken by horizontal scan: " << horizontalScanTime);
    LOG_INFO("Time taken by vertical scan: " << verticalScanTime);
    LOG_INFO("Time taken by overall processing: " << processTime);
    LOG_INFO("Average field height:" << avgHeight);
    LOG_INFO("Minimum best field height:" << minBestHeight);
  }
  drawResults();
  if (GET_DVAR(int, displayOutput)) {
    VisionUtils::displayImage(name, bgrMat[toUType(activeCamera)]);
    waitKey(0);
  }
}

void RegionSegmentation::reset()
{
  setScanSettings();
  for (size_t i = 0; i < hScans.size(); ++i) {
    if (hScans[i] && hScans[i]->enabled) {
      hScans[i]->scanLines.clear();
      hScans[i]->lowStep =
        hScanStepSizes[toUType(activeCamera)][i];
    }
  }
  for (size_t i = 0; i < vScans.size(); ++i) {
    if (vScans[i] && vScans[i]->enabled) {
      vScans[i]->scanLines.clear();
      vScans[i]->lowStep =
        vScanStepSizes[toUType(activeCamera)][i];
    }
  }
  borderPoints.clear();
  static_cast<LinesScan*>(hScans[toUType(ScanTypes::lines)])->edgePoints.clear();
  static_cast<LinesScan*>(vScans[toUType(ScanTypes::lines)])->edgePoints.clear();
}

void RegionSegmentation::setScanSettings()
{
  if (activeCamera == CameraId::headTop) {
    //hScans[toUType(ScanTypes::field)]->enabled = true;
    hScans[toUType(ScanTypes::ourJersey)]->enabled = true;
    hScans[toUType(ScanTypes::oppJersey)]->enabled = true;
    hScans[toUType(ScanTypes::goal)]->enabled = true;

    vScans[toUType(ScanTypes::field)]->enabled = true;
    vScans[toUType(ScanTypes::ourJersey)]->enabled = true;
    vScans[toUType(ScanTypes::oppJersey)]->enabled = true;
    //vScans[toUType(ScanTypes::goal)]->enabled = true;
  } else {
    //hScans[toUType(ScanTypes::field)]->enabled = false;
    hScans[toUType(ScanTypes::ourJersey)]->enabled = false;
    hScans[toUType(ScanTypes::oppJersey)]->enabled = false;
    hScans[toUType(ScanTypes::goal)]->enabled = false;

    vScans[toUType(ScanTypes::field)]->enabled = false;
    vScans[toUType(ScanTypes::ourJersey)]->enabled = false;
    vScans[toUType(ScanTypes::oppJersey)]->enabled = false;
    //vScans[toUType(ScanTypes::goal)]->enabled = false;
  }
}

void RegionSegmentation::horizontalScan()
{
  auto tStart = high_resolution_clock::now();
  auto minScanStepLow = hMinScanStepLow[toUType(activeCamera)];
  if (activeCamera == CameraId::headTop) {
    ///< Last field height with some margin
    auto lastHeight = fieldExt->getFieldHeight() - 50;
    for (int y = 0; y < getImageHeight(); y = y + hScanStepHigh[toUType(activeCamera)]) {
      bool scanInField = y > lastHeight;
      for (const auto& scan : hScans) {
        if (scan && scan->enabled) scan->reset();
      }
      for (int x = 0; x < getImageWidth(); x = x + minScanStepLow) {
        if (scanInField) {
          if (x >= hScanLimitIdx) {
            for (const auto& scan : hScans) {
              if (scan && scan->enabled) scan->onScanLimitReached(x, y);
            }
            break;
          }
          auto pixel = getYUV(x, y);
          TNColors color = colorHandler->whichColor(pixel);
          for (const auto& scan : hScans) {
            if (scan && scan->enabled && scan->scanTables[toUType(activeCamera)][x])
              scan->update(color, x, y);
          }
          if (color == TNColors::white)
            bgrMat[toUType(activeCamera)].at<Vec3b>(y, x) = Vec3b(0, 0, 0);
          else if (color == TNColors::green)
            bgrMat[toUType(activeCamera)].at<Vec3b>(y, x) = Vec3b(0, 255, 0);
          else {
            bgrMat[toUType(activeCamera)].at<Vec3b>(y, x) = Vec3b(0, 255, 255);
          }
        } else {
          auto& ourScan = hScans[toUType(ScanTypes::ourJersey)];
          auto& oppScan = hScans[toUType(ScanTypes::oppJersey)];
          auto& goalScan = hScans[toUType(ScanTypes::goal)];
          if (x >= hScanLimitIdx) {
            if (ourScan->enabled) {
              ourScan->onScanLimitReached(x, y);
            }
            if (oppScan->enabled) {
              oppScan->onScanLimitReached(x, y);
            }
            if (goalScan->enabled) {
              goalScan->onScanLimitReached(x, y);
            }
            break;
          }
          auto pixel = getYUV(x, y);
          TNColors color = colorHandler->whichColor(pixel);
          if (ourScan->enabled && ourScan->scanTables[toUType(activeCamera)][x]) {
            ourScan->update(color, x, y);
          }
          if (oppScan->enabled && oppScan->scanTables[toUType(activeCamera)][x]) {
            oppScan->update(color, x, y);
          }
          if (goalScan->enabled && goalScan->scanTables[toUType(activeCamera)][x]) {
            goalScan->update(color, x, y);
          }
          if (color == TNColors::white)
            bgrMat[toUType(activeCamera)].at<Vec3b>(y, x) = Vec3b(0, 0, 0);
          else if (color == TNColors::green)
            bgrMat[toUType(activeCamera)].at<Vec3b>(y, x) = Vec3b(0, 255, 0);
          else {
            bgrMat[toUType(activeCamera)].at<Vec3b>(y, x) = Vec3b(0, 255, 255);
          }
        }
      }
      //hScans[toUType(ScanTypes::robot)]->draw(bgrMat[toUType(activeCamera)]);
    }
  } else {
    for (int y = 0; y < getImageHeight(); y = y + hScanStepHigh[toUType(activeCamera)]) {
      for (const auto& scan : hScans) {
        if (scan && scan->enabled) scan->reset();
      }
      for (int x = 0; x < getImageWidth(); x = x + minScanStepLow) {
        if (x >= hScanLimitIdx) {
          for (const auto& scan : hScans) {
            if (scan && scan->enabled) scan->onScanLimitReached(x, y);
          }
          break;
        }
        auto pixel = getYUV(x, y);
        TNColors color = colorHandler->whichColor(pixel);
        for (const auto& scan : hScans) {
          if (scan && scan->enabled && scan->scanTables[toUType(activeCamera)][x])
            scan->update(color, x, y);
        }
      }
    }
  }
  auto tEnd = high_resolution_clock::now();
  duration<double> timeSpan = tEnd - tStart;
  horizontalScanTime = timeSpan.count();
}

void RegionSegmentation::verticalScan()
{
  auto tStart = high_resolution_clock::now();
  auto minScanStepLow = vMinScanStepLow[toUType(activeCamera)];
  if (activeCamera == CameraId::headTop) {
    ///< Last field height with some margin
    auto lastHeight = fieldExt->getFieldHeight() - 50;
    avgHeight = 0;
    minBestHeight = 1000;

    for (int x = 0; x < getImageWidth(); x = x + vScanStepHigh[toUType(activeCamera)]) {
      for (const auto& scan : vScans) {
        if (scan) scan->reset();
      }
      for (int y = 0; y < getImageHeight(); y = y + minScanStepLow) {
        bool scanInField = y > lastHeight;
        if (scanInField) {
          if (y >= vScanLimitIdx) {
            for (const auto& scan : vScans) {
              if (scan && scan->enabled) scan->onScanLimitReached(y, x);
            }
            break;
          }
          auto pixel = getYUV(x, y);
          TNColors color = colorHandler->whichColor(pixel);
          for (const auto& scan : vScans) {
            if (scan && scan->scanTables[toUType(activeCamera)][y]) {
              scan->update(color, y, x);
            }
          }
        } else {
          auto& ourScan = vScans[toUType(ScanTypes::ourJersey)];
          auto& oppScan = vScans[toUType(ScanTypes::oppJersey)];
          if (y >= vScanLimitIdx) {
            if (ourScan->enabled) {
              ourScan->onScanLimitReached(y, x);
            }
            if (oppScan->enabled) {
              oppScan->onScanLimitReached(y, x);
            }
            break;
          }

          auto pixel = getYUV(x, y);
          TNColors color = colorHandler->whichColor(pixel);
          if (ourScan->enabled) {
            ourScan->update(color, y, x);
          }
          if (oppScan->enabled) {
            oppScan->update(color, y, x);
          }
        }
        //bgrMat[toUType(activeCamera)].at<Vec3b>(y, x) = Vec3b(255,0,0);
        //VisionUtils::displayImage("bgrMat[toUType(activeCamera)]:", bgrMat[toUType(activeCamera)]);
        //waitKey(0);
      }
      if (GET_DVAR(int, drawVerticalLines))
        vScans[toUType(ScanTypes::field)]->draw(bgrMat[toUType(activeCamera)]);
      //vScans[toUType(ScanTypes::robot)]->draw(bgrMat[toUType(activeCamera)]);
      //VisionUtils::displayImage("bgrMat[toUType(activeCamera)]:", bgrMat[toUType(activeCamera)]);
      //waitKey(0);
      if (static_cast<FieldScan*>(vScans[toUType(ScanTypes::field)])->fieldFound) {
        if (static_cast<FieldScan*>(vScans[toUType(ScanTypes::field)])->fieldFound &&
            static_cast<FieldScan*>(vScans[toUType(ScanTypes::field)])->fieldMin != getImageHeight())
        {
          borderPoints.push_back(Point(x, static_cast<FieldScan*>(vScans[toUType(ScanTypes::field)])->fieldMin));
          //borderPoints.push_back(Point(x, static_cast<FieldScan*>(vScans[toUType(ScanTypes::field)])->fieldMax));
          minBestHeight =
            static_cast<FieldScan*>(vScans[toUType(ScanTypes::field)])->fieldMin < minBestHeight ?
            static_cast<FieldScan*>(vScans[toUType(ScanTypes::field)])->fieldMin : minBestHeight;
          avgHeight += static_cast<FieldScan*>(vScans[toUType(ScanTypes::field)])->fieldMin;
        }
      }
    }
    if (borderPoints.size() > 1)
      avgHeight /= borderPoints.size();
    else
      avgHeight = 0;
  } else {
    for (int x = 0; x < getImageWidth(); x = x + vScanStepHigh[toUType(activeCamera)]) {
      for (const auto& scan : vScans) {
        if (scan && scan->enabled) scan->reset();
      }
      for (int y = 0; y < getImageHeight(); y = y + minScanStepLow) {
        if (y >= vScanLimitIdx) {
          for (const auto& scan : vScans) {
            if (scan && scan->enabled) scan->onScanLimitReached(y, x);
          }
          break;
        }
        auto pixel = getYUV(x, y);
        TNColors color = colorHandler->whichColor(pixel);
        for (const auto& scan : vScans) {
          if (scan && scan->enabled && scan->scanTables[toUType(activeCamera)][y]) {
            scan->update(color, y, x);
          }
        }
      }
    }
  }
  auto tEnd = high_resolution_clock::now();
  duration<double> timeSpan = tEnd - tStart;
  verticalScanTime = timeSpan.count();
}

void RegionSegmentation::drawResults()
{
  Mat bgr = bgrMat[toUType(activeCamera)];
  if (GET_DVAR(int, drawPoints)) {
    if (!borderPoints.empty())
      VisionUtils::drawPoints(borderPoints, bgr);
  }

  if (GET_DVAR(int, drawVerticalLines)) {
    for (const auto& scan : vScans) {
      if (scan && scan->enabled) scan->draw(bgrMat[toUType(activeCamera)]);
    }
  }

  if (GET_DVAR(int, drawHorizontalLines)) {
    for (const auto& scan : hScans) {
      if (scan && scan->enabled) scan->draw(bgrMat[toUType(activeCamera)]);
    }
  }
}

Scan* RegionSegmentation::getHorizontalScan(const ScanTypes& type) { return hScans[toUType(type)]; }
Scan* RegionSegmentation::getVerticalScan(const ScanTypes& type) { return vScans[toUType(type)]; }
