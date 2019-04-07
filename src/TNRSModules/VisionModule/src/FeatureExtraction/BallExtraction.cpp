/**
 * @file VisionModule/src/FeatureExtraction/BallExtraction.cpp
 *
 * This file implements the class for ball extraction from the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/TNColors.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "UserCommModule/include/UserCommModule.h"
#include "VisionModule/include/CameraTransform.h"
#include "VisionModule/include/ColorHandler.h"
#include "VisionModule/include/FeatureExtraction/BallExtraction.h"
#include "VisionModule/include/FeatureExtraction/BallTracker.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/RegionSegmentation.h"
#include "VisionModule/include/FeatureExtraction/RegionScanners.h"
#include "VisionModule/include/FeatureExtraction/ScannedRegion.h"
#include "VisionModule/include/FeatureExtraction/ScannedLine.h"
#include "VisionModule/include/VisionExceptions.h"

BallExtraction::BallExtraction(VisionModule* visionModule) :
  FeatureExtraction(visionModule),
  DebugBase("BallExtraction", this),
  gridSizeTop(Point(160, 120)),
  gridSizeBottom(Point(80, 60)),
  ballType(1) // For checkered ball, see config VisionConfig.ini.
{
  initDebugBase();
  int tempSendTime;
  int tempDrawPredictionState;
  int tempDrawScannedRegions;
  int tempDrawBallContour;
  int tempDisplayInfo;
  int tempDisplayOutput;
  GET_CONFIG(
    "EnvProperties",
    (int, ballType, ballType),
    (float, ballRadius, ballRadius),
    (float, coeffSF, coeffSF),
    (float, coeffRF, coeffRF),
  )

  GET_CONFIG(
    "VisionConfig",
    (float, BallExtraction.ballRadiusMin, ballRadiusMin),
    (float, BallExtraction.ballRadiusMax, ballRadiusMax),
    (int, BallExtraction.sendTime, tempSendTime),
    (int, BallExtraction.drawPredictionState, tempDrawPredictionState),
    (int, BallExtraction.drawScannedRegions, tempDrawScannedRegions),
    (int, BallExtraction.drawBallContour, tempDrawBallContour),
    (int, BallExtraction.displayInfo, tempDisplayInfo),
    (int, BallExtraction.displayOutput, tempDisplayOutput),
    (int, BallExtraction.scanStepLow, scanStepLow),
    (int, BallExtraction.scanStepHigh, scanStepHigh),
  )

  SET_DVAR(int, sendTime, tempSendTime);
  SET_DVAR(int, drawScannedRegions, tempDrawScannedRegions);
  SET_DVAR(int, drawBallContour, tempDrawBallContour);
  SET_DVAR(int, displayInfo, tempDisplayInfo);
  SET_DVAR(int, displayOutput, tempDisplayOutput);

  ///< Get other extraction classes
  fieldExt = GET_FEATURE_EXT_CLASS(FieldExtraction, FeatureExtractionIds::field);
  regionSeg = GET_FEATURE_EXT_CLASS(RegionSegmentation, FeatureExtractionIds::segmentation);

  ///< Make a ball tracker
  ballTracker = boost::make_shared <BallTracker> (visionModule);
  ballTracker->init(CameraId::headTop);

  ///< Load ball classifier
  try {
    loadBallClassifier();
  } catch (VisionException& e) {
    LOG_EXCEPTION(e.what());
  }

  ///< Ball scan parameters
  regionsDist.resize(static_cast<size_t>(CameraId::count));
  regionsDist[0] = 50;
  regionsDist[1] = 200;
  int gridDivX = 640 / gridSizeTop.x;
  int gridDivY = 480 / gridSizeTop.y;
  for (int x = 0; x < gridDivX; ++x) {
    for (int y = 0; y < gridDivY; ++y) {
      topXYPairs.push_back(Point(x, y));
    }
  }
  gridDivX = 160 / (gridSizeBottom.x);
  gridDivY = 120 / (gridSizeBottom.y);
  for (int x = 0; x < gridDivX; ++x) {
    for (int y = 0; y < gridDivY; ++y) {
      bottomXYPairs.push_back(Point(x, y));
    }
  }

  ///< Initialize processing times
  processTime = 0.f;
  resetBallTrackerTime = 0.f;
  ballDetectionTime = 0.f;
  scanTime = 0.f;
  regionFilterTime = 0.f;
  regionClassificationTime = 0.f;
  updateBallInfoTime = 0.f;
  findBallRegionsTime = 0.f;

  //string tfliteFile =
  //  ConfigManager::getConfigDirPath() +
  //"/Classifiers/quantized_model.tflite";

  //model = FlatBufferModel::BuildFromFile(tfliteFile.c_str());
  //InterpreterBuilder builder(*model, resolver);
  //builder(&interpreter);
  // Resize input tensors, if desired.
  //interpreter->AllocateTensors();
}

void BallExtraction::loadBallClassifier()
{
  string classifierFile =
    ConfigManager::getCommonConfigDirPath() +
    "/Classifiers/BallClassifier.xml";

  if (!cascade.load(classifierFile)) {
    throw
      VisionException(
        "BallExtraction",
        "Ball classifier not found.",
        true
      );
  }
}

void BallExtraction::processImage()
{
  try {
    auto tStart = high_resolution_clock::now();
    ///< Whether the ball extraction class is working on the lower cam
    if (resetBallTracker()) {
      foundBall.clear();
      ///< Update ball tracker prediction
      Mat predState = ballTracker->predict();
      if (ballTracker->getBallFound()) {
        findBallFromPredState(predState);
      } else {
        findBallRegions();
      }
      ///< Perform ball tracker correction step
      ballTracker->updateFilter(foundBall);
      ///< Update ball info in memory
      updateBallInfo();
      drawResults();
    }
    duration<double> timeSpan = high_resolution_clock::now() - tStart;
    processTime = timeSpan.count();
    if (GET_DVAR(int, displayOutput))
      VisionUtils::displayImage("BallExtraction", bgrMat[toUType(activeCamera)]);
    if (GET_DVAR(int, displayInfo)) {
      LOG_INFO("BallExtraction Results");
      LOG_INFO("Time taken by overall process: " << processTime);
      LOG_INFO("Time taken by resetting ball tracker: " << resetBallTrackerTime);
      LOG_INFO("Time taken by scan: " << scanTime);
      LOG_INFO("Time taken by regions filteration: " << regionFilterTime);
      LOG_INFO("Time taken by regions classification: " << regionClassificationTime);
      LOG_INFO("Time taken by ball detection: " << ballDetectionTime);
      LOG_INFO("Time taken by updating ball in memory: " << updateBallInfoTime);
      LOG_INFO("Ball found: " << ballTracker->getBallFound());
    }
  } catch (exception& e){
    LOG_EXCEPTION("Exception raised in BallExtraction:\n\t" << e.what());
  }
}

bool BallExtraction::resetBallTracker()
{
  auto tStart = high_resolution_clock::now();
  auto& ballInfo = BALL_INFO_OUT(VisionModule);
  bool continueProcessing = true;
  if (ballTracker->getBallFound())///< Ball is found
  {
    if (ballInfo.cameraNext == activeCamera) { ///< If the other camera is curret image))
      if (ballInfo.cameraNext != ballInfo.camera) { ///< It is expected to be found in other camera from previous one
        auto ballState = ballTracker->getEstimatedState();

        ///< Relative position in real world
        auto ballWorldRel =
          Point3f(ballState.at<float>(0), ballState.at<float>(1), ballRadius);

        ///< Ball in activeCamera
        Point2f ballInCurrent;
        cameraTransforms[toUType(activeCamera)]->worldToImage(ballWorldRel, ballInCurrent);

        if (ballInCurrent.x >= 0 && ballInCurrent.x <= getImageWidth() &&
            ballInCurrent.y >= 0 && ballInCurrent.y <= getImageHeight())
        {
          ///< Ball in this current camera
          ///< Position is shifted from the previous cam to new cam
          ballState.at<float>(6) = ballInCurrent.x;
          ballState.at<float>(7) = ballInCurrent.y;

          ///< Reassigning ball width and height and scaling velocity according
          ///< to current camera
          auto camRatio = getImageWidth() / imageWidth[toUType(ballInfo.camera)];
          ballState.at<float>(8) *= camRatio;
          ballState.at<float>(9) *= camRatio;
          ballState.at<float>(10) *= camRatio;
          ballState.at<float>(11) *= camRatio;
          ballTracker->reset(activeCamera, ballState);
          //xySeen.clear();
        }
      }
    } else {
      continueProcessing = false; ///< Expected camera is not the current image
    }
  } else if (!ballTracker->getBallFound()) {
    ///< Reset the tracker for current camera
    ballTracker->reset(activeCamera);
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  resetBallTrackerTime = timeSpan.count();
  return continueProcessing;
}

void BallExtraction::findBallFromPredState(Mat& predState)
{
  ///< Update predicted position with new position for current
  ///< camera transformation matrix. This is important for when
  ///< robot head is moving during scan
  MatrixXf predImagePos;
  predImagePos.resize(3, 1);
  predImagePos(0, 0) = predState.at<float>(6);
  predImagePos(1, 0) = predState.at<float>(7);
  predImagePos(2, 0) = 1.0;
  Matrix<float, 3, 1> newImagePos =
    cameraTransforms[toUType(activeCamera)]->prevImageToCurrentImage(predImagePos);
  predState.at<float>(6) = newImagePos(0, 0);
  predState.at<float>(7) = newImagePos(1, 0);

  Rect predRoi;
  getPredRoi(predRoi, predState);
  if (GET_DVAR(int, drawPredictionState))
    drawState(predState, Scalar(0, 255, 0));
  if (ballType == 0) {
    redBallDetector(predRoi);
  } else if (ballType == 1) {
    if (activeCamera == CameraId::headTop) findBallUpperCam(predRoi);
    else findBallLowerCam(predRoi);
  }
}

void BallExtraction::getPredRoi(Rect& predRoi, const Mat& predState)
{
  predRoi.width = predState.at<float>(10);
  predRoi.height = predState.at<float>(11);
  predRoi.x = predState.at<float>(6) - predRoi.width / 2;
  predRoi.y = predState.at<float>(7) - predRoi.height / 2;
  predRoi = predRoi & Rect(0, 0, getImageWidth(), getImageHeight());
}

void BallExtraction::redBallDetector(const Rect& origRect)
{
  auto tStart = high_resolution_clock::now();
  if (origRect.width < 5 || origRect.height < 5) return;
  Rect scaled = origRect;
  int factor = 2;
  scaled = scaled - Point((scaled.width * factor) / 2, (scaled.height * factor) / 2);
  scaled += Size(scaled.width * factor, scaled.height * factor);
  scaled = scaled & Rect(0, 0, getImageWidth(), getImageHeight());
  Mat cropped = bgrMat[toUType(activeCamera)](scaled), redImage;
  inRange(cropped, Scalar(0, 0, 150), Scalar(60, 60, 255), redImage);
  vector < vector<Point> > contours;
  vector < Vec4i > hierarchy;
  findContours(
    redImage,
    contours,
    hierarchy,
    CV_RETR_TREE,
    CV_CHAIN_APPROX_SIMPLE,
    Point(0, 0));
  if (contours.empty()) return;
  vector < Point > possibleBall;
  float maxArea = 0;
  for (size_t i = 0; i < contours.size(); i++) {
    float area = contourArea(contours[i]);
    if (area > 10) {
      if (area > maxArea) {
        maxArea = area;
        possibleBall = contours[i];
      }
    }
  }
  if (maxArea > 0) {
    Rect ball = boundingRect(possibleBall);
    ball.x = scaled.x + ball.x;
    ball.y = scaled.y + ball.y;
    foundBall.push_back(ball);
    /*rectangle(
      bgrMat[toUType(activeCamera)],
      ball,
      Scalar(0,0,0),
      3
    );
    for( int i = 0; i < contours.size(); i++ )
    {
      drawContours(cropped, contours, i, Scalar(0), 2, 8, hierarchy, 0, Point() );
    }*/
  }
  //VisionUtils::displayImage(bgrMat[toUType(activeCamera)], "bgrMat[toUType(activeCamera)]1");
  //VisionUtils::displayImage(cropped, "croppedImage");
  //VisionUtils::displayImage(redImage, "redImage");
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  ballDetectionTime = timeSpan.count();
}

void BallExtraction::findBallRegions()
{
  auto tStart = high_resolution_clock::now();
  auto verBallLines = regionSeg->getVerticalScan(ScanTypes::ball)->scanLines;
  auto horBallLines = regionSeg->getHorizontalScan(ScanTypes::ball)->scanLines;
  //regionSeg->getVerticalScan(ScanTypes::ball)->draw(bgrMat[toUType(activeCamera)]);
  //regionSeg->getHorizontalScan(ScanTypes::ball)->draw(bgrMat[toUType(activeCamera)]);

  vector<ScannedRegionPtr> verBallRegions;
  vector<ScannedRegionPtr> horBallRegions;
  auto verLineLinkXTol =
    regionSeg->getVerticalScan(ScanTypes::ball)->highStep * 1.5; // pixels
  auto verLineLinkYTol = verLineLinkXTol;
  auto horLineLinkXTol =
    regionSeg->getHorizontalScan(ScanTypes::ball)->highStep * 1.5; // pixels
  auto horLineLinkYTol = horLineLinkXTol;
  auto maxLineLengthDiffRatio = 1.5;
  findRegions(
    verBallRegions,
    verBallLines,
    verLineLinkXTol,
    verLineLinkYTol,
    maxLineLengthDiffRatio,
    bgrMat[toUType(activeCamera)]);
  findRegions(
    horBallRegions,
    horBallLines,
    horLineLinkXTol,
    horLineLinkYTol,
    maxLineLengthDiffRatio,
    bgrMat[toUType(activeCamera)]);
  //ScannedRegion::drawRegions(
  //  bgrMat[toUType(activeCamera)], verBallRegions, Scalar(255,0,0));
  //ScannedRegion::drawRegions(
  //  bgrMat[toUType(activeCamera)], horBallRegions, Scalar(0,255,0));
  horBallRegions.insert(
    horBallRegions.end(),
    verBallRegions.begin(),
    verBallRegions.end());
  vector<ScannedRegionPtr> regionsFiltered;
  static const auto regionsXDiffTol = 16; // pixels
  static const auto regionsYDiffTol = 16; // pixels
  static const auto maxRegionSizeDiffRatio = 2.5;
  ScannedRegion::linkRegions(
    regionsFiltered,
    horBallRegions,
    regionsXDiffTol,
    regionsYDiffTol,
    maxRegionSizeDiffRatio,
    bgrMat[toUType(activeCamera)]);
  //ScannedRegion::drawRegions(
  //  bgrMat[toUType(activeCamera)], regionsFiltered, Scalar(0,0,255));
  for (const auto& br : regionsFiltered) {
    if (br) {
      if (br->rect.width > 5 && br->rect.height > 5 &&
          br->rect.width > 5 * br->rect.height ||
          br->rect.height > 5 * br->rect.width)
        continue;
      auto& r = br->rect;
      auto wHRatio = r.width / (float) r.height;
      auto newCenter = Point(r.x + r.width / 2, r.y + r.height / 2);
      r = Rect(
        newCenter - Point(r.width / wHRatio / 2, r.height / 2),
        Size(r.width / wHRatio, r.height));
      auto ratio = 1.25;
      r -= Point(r.width / (2 * ratio), r.height / (2 * ratio));
      r += Size(r.width / ratio, r.height / ratio);
      r = r & Rect(0, 0, getImageWidth(), getImageHeight());
      Mat croppedImage = getGrayImage()(r);
      applyClassifier(br->rect, croppedImage);
      if (GET_DVAR(int, drawScannedRegions))
        br->draw(bgrMat[toUType(activeCamera)], Scalar(0, 0, 0));
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  findBallRegionsTime = timeSpan.count();
}

void BallExtraction::findBallUpperCam(const Rect& roi)
{
  auto tStart = high_resolution_clock::now();
  vector<RectPtr> boundRects;
  scanRoi(boundRects, roi);
  filterRegions(boundRects, 50);
  for (int j = 0; j < boundRects.size(); ++j) {
    if (!boundRects[j])
    continue;
    auto r = *boundRects[j];
    if (GET_DVAR(int, drawScannedRegions))
      rectangle(bgrMat[toUType(activeCamera)], r, Scalar(255,0,0), 1);
    float factorX = roi.width / boundRects[j]->width;
    float factorY = roi.height / boundRects[j]->height;
    r = r - Point((boundRects[j]->width * factorX) / 2, (boundRects[j]->height * factorY) / 2);
    r += Size(boundRects[j]->width * factorX, boundRects[j]->height * factorY);
    float wHRatio = r.width / (float) r.height;
    Point newCenter = Point(r.x + r.width / 2, r.y + r.height / 2);
    //VisionUtils::drawPoint(newCenter, bgrMat[toUType(activeCamera)]);
    r = Rect(
      newCenter - Point(r.width / wHRatio / 2, r.height / 2),
      Size(r.width / wHRatio, r.height));
    r = r & Rect(0, 0, getImageWidth(), getImageHeight());
    *boundRects[j] = r;
    if (GET_DVAR(int, drawScannedRegions))
      rectangle(bgrMat[toUType(activeCamera)], *boundRects[j], Scalar(255,0,0), 2);
  }
  classifyRegions(boundRects);
  //rectangle(bgrMat[toUType(activeCamera)], roi, Scalar(255,0,255), 1);
  //VisionUtils::displayImage(bgrMat[toUType(activeCamera)], "after prediction");
  //waitKey(0);
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  ballDetectionTime = timeSpan.count();
}

void BallExtraction::findBall(vector<int>& pairIndices)
{
  auto tStart = high_resolution_clock::now();
  srand(visionModule->getModuleTime() * 100);
  random_shuffle(pairIndices.begin(), pairIndices.end());
  vector<RectPtr> boundRects;
  scanRandom(boundRects, pairIndices, 4);
  for (int j = 0; j < boundRects.size(); ++j) {
    if (!boundRects[j])
    continue;
    //rectangle(bgrMat[toUType(activeCamera)], *boundRects[j], Scalar(0,255,255), 2);
  }
  filterRegions(boundRects, 50);
  for (int j = 0; j < boundRects.size(); ++j) {
    if (!boundRects[j])
    continue;
    if (!boundRects[j]) continue;
    //r//ectangle(bgrMat[toUType(activeCamera)], *boundRects[j], Scalar(255,255,255), 1);
    //VisionUtils::displayImage(bgrMat[toUType(activeCamera)], "bgrMat[toUType(activeCamera)]");
    //waitKey(0);
    Rect r = *boundRects[j];
    float wHRatio = r.width / (float) r.height;
    Point newCenter = Point(r.x + r.width / 2, r.y + r.height / 2);
    VisionUtils::drawPoint(newCenter, bgrMat[toUType(activeCamera)]);
    r = Rect(
      newCenter - Point(r.width / wHRatio / 2, r.height / 2),
      Size(r.width / wHRatio, r.height));
    int ratio = 1.25;
    r -= Point(r.width / (2 * ratio), r.height / (2 * ratio));
    r += Size(r.width / ratio, r.height / ratio);
    r = r & Rect(0, 0, getImageWidth(), getImageHeight());
    *boundRects[j] = r;
    //rectangle(bgrMat[toUType(activeCamera)], *boundRects[j], Scalar(0,255,0), 2);
  }
  //VisionUtils::displayImage(bgrMat[toUType(activeCamera)], "bgrmat");
  //waitKey(0);

  classifyRegions(boundRects);
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  ballDetectionTime = timeSpan.count();
}

void BallExtraction::findBallLowerCam(const Rect& roi)
{
  auto tStart = high_resolution_clock::now();
  vector<RectPtr> boundRects;
  scanRoi(boundRects, roi);
  filterRegions(boundRects, 50);
  for (int j = 0; j < boundRects.size(); ++j) {
    if (!boundRects[j])
    continue;
      //rectangle(bgrMat[toUType(activeCamera)], *boundRects[j], Scalar(255,0,0), 1);
    float factorX = roi.width / boundRects[j]->width;
    float factorY = roi.height / boundRects[j]->height;
    *boundRects[j] = *boundRects[j] - Point((boundRects[j]->width * factorX) / 2, (boundRects[j]->height * factorY) / 2);
    *boundRects[j] += Size(boundRects[j]->width * factorX, boundRects[j]->height * factorY);
    *boundRects[j] = *boundRects[j] & Rect(0, 0, getImageWidth(), getImageHeight());
    //rectangle(bgrMat[toUType(activeCamera)], *boundRects[j], Scalar(255,0,0), 2);
  }
  classifyRegions(boundRects);
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  ballDetectionTime = timeSpan.count();
}

void BallExtraction::scanRandom(
  vector<RectPtr>& boundRects,
  const vector<int>& pairIndices,
  const int& iters)
{
  auto tStart = high_resolution_clock::now();
  vector<Point> pairs;
  pairs = activeCamera == CameraId::headTop ? topXYPairs : bottomXYPairs;
  Point gridSize = activeCamera == CameraId::headTop ? gridSizeTop : gridSizeBottom;
  for (int i = 0; i < pairIndices.size(); ++i) {
    if (i >= iters) break;
    auto index = pairIndices[i];
    auto x = pairs[index].x * gridSize.x;
    auto y = pairs[index].y * gridSize.y;
    xySeen.push_back(index);
    Mat cropped =
      getGrayImage()(
        Rect(x, y, gridSize.x, gridSize.y) & Rect( 0, 0, getImageWidth(), getImageHeight()
      ));
    //rectangle(
    //  bgrMat[toUType(activeCamera)],
   //   Rect(x, y, gridSize.x, gridSize.y) & Rect(0, 0, getImageWidth(), getImageHeight()),
   //   Scalar(255,255,255), 1);
    //VisionUtils::displayImage(cropped, "cropped1");
    cropped.convertTo(cropped, -1, 1.25, 0);
    Mat black;
    //threshold(cropped, black, 50, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
    colorHandler->getBinary(cropped, black, TNColors::black);
    //VisionUtils::displayImage(cropped, "cropped2");
    //for (int i = 0; i < 100; ++i) {
    //int subC = activeCamera == toUType(CameraId::headTop) ? 15 : 15;
    //adaptiveThreshold(cropped, black, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3, -subC);
    //  cout << "i: " << i << endl;
    //}
    //VisionUtils::displayImage(cropped, "cropped");
    //VisionUtils::displayImage(black, "black");
    //waitKey(0);
    //threshold(cropped, black, 50, 255, 2);
    if (countNonZero(black) <= 0) continue;
    dilate(black, black, Mat(), Point(-1, -1));
    vector < vector<Point> > contours;
    vector < Vec4i > hierarchy;
    //auto offset = Point(0, fHeight);
    //auto offset = Point(x, y >= fHeight ? y : fHeight);
    auto offset = Point(x, y);
    findContours(
      black,
      contours,
      hierarchy,
      CV_RETR_EXTERNAL,
      CV_CHAIN_APPROX_SIMPLE,
      offset);
    for (int i = 0; i < contours.size(); i++) {
      //cout << "countour: " << i << endl;
      //drawContours(bgrMat[toUType(activeCamera)], contours, i, Scalar(0,255,0), 2, 8, hierarchy, 0, Point() );
      Rect boundRect = boundingRect(contours[i]);
      //rectangle(bgrMat[toUType(activeCamera)], boundRect, Scalar(255,255,255), 1);
      float ratio = boundRect.width / (float) boundRect.height;
      if (ratio < 0.333333333 || ratio > 3.0) continue;
      boundRects.push_back(boost::make_shared < Rect > (boundRect));
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  scanTime = timeSpan.count();
}

void BallExtraction::scanRoi(vector<RectPtr>& boundRects, const Rect& roi)
{
  auto tStart = high_resolution_clock::now();
  Rect scaled = roi;
  int factor = 1.5;
  scaled = scaled - Point((scaled.width * factor) / 2, (scaled.height * factor) / 2);
  scaled += Size(scaled.width * factor, scaled.height * factor);
  scaled = scaled & Rect(0, 0, getImageWidth(), getImageHeight());
  Mat cropped = getGrayImage()(scaled);
  Mat black;
  threshold(cropped, black, 50, 255, 1);
  //int subC = activeCamera == toUType(CameraId::headTop) ? 5 : 15;
  //adaptiveThreshold(cropped, black, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3, -subC);
  //VisionUtils::displayImage(black, "black");
  if (countNonZero(black) <= 0) return;
  dilate(black, black, Mat(), Point(-1, -1));
  vector < vector<Point> > contours;
  vector < Vec4i > hierarchy;
  auto offset = Point(scaled.x, scaled.y);
  findContours(
    black,
    contours,
    hierarchy,
    CV_RETR_EXTERNAL,
    CV_CHAIN_APPROX_SIMPLE,
    offset);
  for (int i = 0; i < contours.size(); i++) {
    //drawContours(bgrMat[toUType(activeCamera)], contours, i, Scalar(0,255,0), 2, 8, hierarchy, 0, Point() );
    Rect boundRect = boundingRect(contours[i]);
    //rectangle(bgrMat[toUType(activeCamera)], boundRect, Scalar(255,255,255), 1);
    //rectangle(bgrMat[toUType(activeCamera)], roi, Scalar(255,255,0), 1);
    float ratio = boundRect.width / (float) boundRect.height;
    if (ratio < 0.333333333 || ratio > 3.0) continue;
    boundRects.push_back(boost::make_shared<Rect> (boundRect));
  }
  //VisionUtils::displayImage(bgrMat[toUType(activeCamera)], "bgr");
  //waitKey(0);
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  scanTime = timeSpan.count();
}

void BallExtraction::filterRegions(
  vector<RectPtr>& boundRects,
  const float& threshold)
{
  auto tStart = high_resolution_clock::now();
  for (size_t j = 0; j < boundRects.size(); ++j) {
    if (!boundRects[j]) continue;
    auto r1 = *boundRects[j];
    for (size_t k = 0; k < boundRects.size(); ++k) {
      if (!boundRects[k]) continue;
      if (j != k) {
        auto r2 = *boundRects[k];
        Point c1 = Point(r1.x + r1.width / 2, r1.y + r1.height / 2);
        Point c2 = Point(r2.x + r2.width / 2, r2.y + r2.height / 2);
        if (norm(c1 - c2) < threshold) {
          r1 = r1 | r2;
          *boundRects[j] = r1;
          boundRects[k].reset();
        }
      }
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  regionFilterTime = timeSpan.count();
}

void BallExtraction::classifyRegions(vector<RectPtr>& boundRects)
{
  auto tStart = high_resolution_clock::now();
  for (size_t j = 0; j < boundRects.size(); ++j) {
    if (!boundRects[j])
      continue;
      //cout << "getting gray imag1..." << endl;
    Mat croppedImage = getGrayImage()(*boundRects[j]);
    //cout << "getting gray imag2..." << endl;
    applyClassifier(*boundRects[j], croppedImage);
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  regionClassificationTime = timeSpan.count();
}

void BallExtraction::applyClassifier(const Rect& origRect, Mat& croppedImage)
{
  if (croppedImage.cols < 5 || croppedImage.rows < 5) return;
  //if (croppedImage.rows > 35) {
  //  ratio = 35.f / (float) croppedImage.rows;
  //  resize(croppedImage, croppedImage, cv::Size(), ratio, ratio);
  //}
  //cascade.detectMultiScale(croppedImage, ball, 1.05, 1);
  //float* input = interpreter->typed_input_tensor<float>(0);
  float ratioY = croppedImage.rows;
  float ratioX = croppedImage.cols;
  vector<Rect> balls;
  resize(croppedImage, croppedImage, cv::Size(32, 32));
  ratioY = croppedImage.rows / ratioY;
  ratioX = croppedImage.cols / ratioX;
  cascade.detectMultiScale(croppedImage, balls, 1.05, 1);
  //cout << "balls: " << balls.size() << endl;
  //VisionUtils::displayImage("cropped", croppedImage);
  for (size_t i = 0; i < balls.size(); ++i) {
    auto ball = balls[i];
    /*rectangle(
      croppedImage,
      ball,
      cv::Scalar(255,255,255),
      1
    );*/
    ball.x = ball.x / ratioX + origRect.x;
    ball.y = ball.y / ratioY + origRect.y;
    ball.width = ball.width / ratioX;
    ball.height = ball.height / ratioY;
    foundBall.push_back(ball);
    /*rectangle(
      bgrMat[toUType(activeCamera)],
      origRect,
      cv::Scalar(255,255,0),
      1
    );
    rectangle(
      bgrMat[toUType(activeCamera)],
      ball,
      cv::Scalar(255,255,255),
      1
    );*/
    //VisionUtils::displayImage(croppedImage, "croppedImage");
    //VisionUtils::displayImage(bgrMat[toUType(activeCamera)], "bgr");
    //waitKey(0);
  }
  //Mat floatMat = Mat(croppedImage.size(), CV_32F);
  //croppedImage.convertTo(floatMat, CV_32F);
  //float mean = 80.98907329963235;
  //float std = 74.59484120515009;
  //floatMat = (floatMat - mean) / std;
  //float* inputMat = floatMat.ptr<float>();
  //for (int i = 0; i < 32*32; ++i) {
  //  *input++ = *inputMat++;
  //}

  // Fill `input`.
  //auto start = high_resolution_clock::now();
  //interpreter->Invoke();
  //duration<double> td = high_resolution_clock::now() - start;
  //cout << "time: " << td.count() << endl;
  //printf("=== Post-invoke Interpreter State ===\n");
  /*float* ballLabel = interpreter->typed_output_tensor<float>(0);
  ballLabel++;
  if (fabsf(*ballLabel - 1.f) < 0.75) {
    float* boundingBox = interpreter->typed_output_tensor<float>(1);
    float* o2 = boundingBox;
    Rect r;
    r.y =  *(o2) * 32;
    r.x =  *(++o2) * 32;
    r.height = *(++o2) * 32;
    r.width = *(++o2) * 32;
    Rect ball;
    ball.y = origRect.y + *(boundingBox) * 32 / ratioY,
    ball.x = origRect.x + *(++boundingBox) * 32 / ratioX;
    ball.height = *(++boundingBox) * 32 / ratioY;
    ball.width = *(++boundingBox) * 32 / ratioX;
    foundBall.push_back(ball);*/
    /*rectangle(
      croppedImage,
      r,
      cv::Scalar(255,255,255),
      1
    );
    rectangle(
      bgrMat[toUType(activeCamera)],
      origRect,
      cv::Scalar(255,255,0),
      1
    );
    rectangle(
      bgrMat[toUType(activeCamera)],
      ball,
      cv::Scalar(255,255,255),
      1
    );
    VisionUtils::displayImage(croppedImage, "croppedImage");
    VisionUtils::displayImage(bgrMat[toUType(activeCamera)], "bgr");*/
    //waitKey(0);
  //} else {
    //float* ballLabel2 = interpreter->typed_output_tensor<float>(0);
    //VisionUtils::displayImage(croppedImage, "croppedImage");
    //waitKey(0);
  //}
}

void BallExtraction::drawState(const Mat& state, const Scalar& color)
{
  Point2f orig;
  orig.x = state.at<float>(6);
  orig.y = state.at<float>(7);
  float w = state.at<float>(10);
  float h = state.at<float>(11);
  rectangle(
    bgrMat[toUType(activeCamera)],
    Rect(Point2f(orig.x - w / 2, orig.y - h / 2), Size(w, h)),
    color,
    3
  );
}

void BallExtraction::drawResults()
{
  if (GET_DVAR(int, drawBallContour)) {
    for (int i = 0; i < foundBall.size(); ++i) {
      rectangle(bgrMat[toUType(activeCamera)], foundBall[i], Scalar(0,0,255), 3);
    }
  }
}

void BallExtraction::updateBallInfo()
{
  auto tStart = high_resolution_clock::now();
  Mat ballState = ballTracker->getEstimatedState();
  BallInfo<float> ballInfo;
  ballInfo.found = ballTracker->getBallFound();
  ballInfo.camera = static_cast<CameraId>(activeCamera);
  ballInfo.posRel.x = ballState.at<float>(0);
  ballInfo.posRel.y = ballState.at<float>(1);
  ballInfo.velRel.x = ballState.at<float>(2);
  ballInfo.velRel.y = ballState.at<float>(3);
  ballInfo.accRel.x = ballState.at<float>(4);
  ballInfo.accRel.y = ballState.at<float>(5);
  ballInfo.posImage.x = ballState.at<float>(6);
  ballInfo.posImage.y = ballState.at<float>(7);
  //Point2f nextImagePos;
  //nextImagePos.x = ballInfo.posImage.x + ballState.at<float>(8) * cycleTime;
  //nextImagePos.x = ballInfo.posImage.y + ballState.at<float>(9) * cycleTime;
  //line(bgrMat[toUType(activeCamera)], ballInfo.posImage, nextImagePos, Scalar(255,0,0));
  ballInfo.bboxWidth = ballState.at<float>(10);
  ballInfo.bboxHeight = ballState.at<float>(11);
  ballInfo.ballAge = ballTracker->getTimeSinceLost();
  ballInfo.radius = ballRadius;
  getBallFrameInNextCycle(ballInfo);
  /*cout << "Ball Position: " << ballInfo.posRel << endl;
  cout << "Ball posImage: " << ballInfo.posImage << endl;
  cout << "Ball Camera: " << ballInfo.camera << endl;
  cout << "Ball Camera Next frame: " << ballInfo.cameraNext << endl;
  cout << "Ball Found: " << ballInfo.found << endl;
  cout << "Ball W: " << ballInfo.width << endl;
  cout << "Ball H: " << ballInfo.height << endl;*/
  BALL_INFO_OUT(VisionModule) = ballInfo;
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  updateBallInfoTime = timeSpan.count();
}

unsigned BallExtraction::getBallFrameInNextCycle(BallInfo<float>& ballInfo)
{
  // Just an estimated state
  auto& posRel = ballInfo.posRel;
  auto& posImage = ballInfo.posImage;
  auto ballWorldRel = cv::Point3_<float>(posRel.x, posRel.y, ballRadius);
  auto otherCam = activeCamera == CameraId::headTop ? CameraId::headBottom : CameraId::headTop;
  cv::Point_<float> ballInOther;
  cameraTransforms[toUType(otherCam)]->worldToImage(ballWorldRel, ballInOther);
  if (ballInOther.x > 0 &&
      ballInOther.x < imageWidth[toUType(otherCam)] &&
      ballInOther.y > 0 &&
      ballInOther.y < imageHeight[toUType(otherCam)]
     )
  {
    ///< See if the ball is more visible in the other cam
    auto yBoundary =
      posImage.y > getImageHeight() / 2 ?
      getImageHeight() - posImage.y : posImage.y;
    yBoundary /= getImageHeight();
    auto yBoundaryInOther =
      ballInOther.y > imageHeight[toUType(otherCam)] / 2 ?
      imageHeight[toUType(otherCam)] - ballInOther.y  : ballInOther.y;
    yBoundaryInOther /= imageHeight[toUType(otherCam)];
    //cout << "yBoundaryInOther: " << yBoundaryInOther << endl;
    //cout << "yBoundary: " << yBoundary << endl;
    if (yBoundaryInOther > yBoundary) {
      ballInfo.cameraNext = static_cast<CameraId>(otherCam);
    } else {
      ballInfo.cameraNext = static_cast<CameraId>(activeCamera);
    }
  }
  ballInfo.cameraNext = static_cast<CameraId>(activeCamera);
}

bool BallExtraction::getBallFound()
{
  return ballTracker->getBallFound();
}

/*void BallExtraction::simpleDetect(const Rect& origRect, Mat croppedImage)
{
  vector < vector<Point> > contours;
  vector < Vec4i > hierarchy;
  Mat black;
  threshold(croppedImage, black, 50, 255, 1);
  Mat canny;
  Canny(black, canny, 100, 255, 3);
  findContours(
    canny,
    contours,
    hierarchy,
    CV_RETR_TREE,
    CV_CHAIN_APPROX_SIMPLE,
    Point(0, 0));
  if (contours.size() < 5) return;
  //vector < Point > result;
  vector < Point > pts;
  for (size_t i = 0; i < contours.size(); i++)
    for (size_t j = 0; j < contours[i].size(); j++)
      pts.push_back(contours[i][j]);
  Rect ball = boundingRect(pts);
  Point center;
  center.x = ball.x + ball.width / 2 - croppedImage.rows / 2;
  center.y = ball.y + ball.height / 2 - croppedImage.cols / 2;
  ball.x = origRect.x + center.x;
  ball.y = origRect.y + center.y;
  ball.width = origRect.width;
  ball.height = origRect.height;
  foundBall.push_back(ball);
  //polylines(croppedImage, result, false, Scalar(255), 2);
  //for( int i = 0; i < contours.size(); i++ )
  //{
  //  drawContours(croppedImage, contours, i, Scalar(0), 2, 8, hierarchy, 0, Point() );
  //}
  //vector<Rect> ball;
  //rectangle(croppedImage, ball, Scalar(0), 3);
  //imshow("croppedImage" , croppedImage);
  //waitKey(0);
}*/
