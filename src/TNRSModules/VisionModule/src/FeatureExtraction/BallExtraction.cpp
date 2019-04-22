/**
 * @file VisionModule/src/FeatureExtraction/BallExtraction.cpp
 *
 * This file implements the class for ball extraction from the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include <algorithm>
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
#include "VisionModule/include/FeatureExtraction/RobotExtraction.h"
#include "VisionModule/include/FeatureExtraction/RobotRegion.h"
#include "VisionModule/include/FeatureExtraction/RegionSegmentation.h"
#include "VisionModule/include/FeatureExtraction/RegionScanners.h"
#include "VisionModule/include/FeatureExtraction/ScannedRegion.h"
#include "VisionModule/include/FeatureExtraction/ScannedLine.h"
#include "VisionModule/include/VisionExceptions.h"
#include "Utils/include/AngleDefinitions.h"

struct Blob {
  Blob (const Rect& rect, const Point& center, const vector<Point>& contour) :
    rect(rect), center(center), contour(contour)
  {
    size = rect.area();
  }

  float size;
  Point center;
  Rect rect;
  vector<Point> contour;
};

struct Triangle {
  Triangle () = default;
  vector<Point> points;
  vector<boost::shared_ptr<Blob> > pentagons;
};

BallExtraction::BallExtraction(VisionModule* visionModule) :
  FeatureExtraction(visionModule, "BallExtraction"),
  DebugBase("BallExtraction", this),
  ballType(1) // For checkered ball, see config VisionConfig.ini.
{
  initDebugBase();
  GET_CONFIG(
    "EnvProperties",
    (int, ballType, ballType),
    (float, ballRadius, ballRadius),
    (float, coeffSF, coeffSF),
    (float, coeffRF, coeffRF),
  )

  GET_DEBUG_CONFIG(
    VisionConfig,
    BallExtraction,
    (int, sendTime),
    (int, drawPredictionState),
    (int, drawScannedLines),
    (int, drawUnlinkedScannedRegions),
    (int, drawClassifiedPentagons),
    (int, drawClassifiedTriangles),
    (int, drawBallCircles),
    (int, drawScannedRegions),
    (int, drawBallContour),
    (int, drawPredictionROI),
    (int, displayInfo),
    (int, displayOutput),
  );

  GET_CLASS_CONFIG(
    VisionConfig,
    BallExtraction,
    lineLinkHorXTolRatio,
    lineLinkHorYTolRatio,
    lineLinkVerXTolRatio,
    lineLinkVerYTolRatio,
    maxLineLengthDiffRatio,
    regionsXDiffTol,
    regionsYDiffTol,
    maxRegionSizeDiffRatio,
    regionFilterMinWidth,
    regionFilterMinHeight,
    regionFilterMaxAspectRatio,
    robotFilterHeightRatio,
    fallenRobotFilterHeightRatio,
    robotMaxOverlapTop,
    robotMaxOverlapBottom,
    minOverlapAreaThreshold,
    maxBallRegionSizeRatio,
    ballRegionPaddingRatio,
    adaptiveThresholdWindowSizeRatio,
    adaptiveThresholdSubConstantRatio,
    maxBallBlobSizeRatio,
    minBallBlobSizeRatio,
    ballBlobMaxAspectRatio,
    maxBallBlobIntensity,
    minPentagonsRequired,
    maxPentagonsRequired,
    minPentagonsPoorCandidates,
    maxBlobToBlobSizeDiffRatio,
    gaussianSizeX,
    gaussianSizeY,
    gaussianSigmaX,
    gaussianSigmaY,
    houghCirclesMethod,
    houghCirclesMinDistRatio,
    minRadiusPixelTolerance,
    maxRadiusPixelTolerance,
    triangleToCircleMaxDistRatio,
    cascadePaddingRatio,
    CNNClassificationTolerance,
    predictedAreaRoiRatio,
  );

  ///< Get other extraction classes
  fieldExt = GET_FEATURE_EXT_CLASS(FieldExtraction, FeatureExtractionIds::field);
  robotExt = GET_FEATURE_EXT_CLASS(RobotExtraction, FeatureExtractionIds::robot);
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

  string tfliteFile =
    ConfigManager::getCommonConfigDirPath() +
  "/Classifiers/quantized_model.tflite";

  model = FlatBufferModel::BuildFromFile(tfliteFile.c_str());
  InterpreterBuilder builder(*model, resolver);
  builder(&interpreter);
  // Resize input tensors, if desired.
  interpreter->AllocateTensors();
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
    vector<Rect> bestCandidates;
    vector<Rect> poorCandidates;
    if (resetBallTracker()) {
      classifiedBalls.clear();
      ///< Update ball tracker prediction
      Mat predState = ballTracker->predict();
      if (GET_DVAR(int, drawPredictionState))
        drawState(predState, Scalar(0, 255, 0));

      vector<ScannedRegionPtr> ballRegions;
      if (ballType == 1) {
        findBallRegions(ballRegions);
        if (activeCamera == CameraId::headTop)
          filterFromRobotRegions(ballRegions);
      }

      Rect roi = Rect(0, 0, getImageWidth(), getImageHeight());
      if (ballTracker->getBallFound()) {
        findBallROIFromPredState(roi, predState);
      }

      if (GET_DVAR(int, drawPredictionROI))
        rectangle(bgrMat[toUType(activeCamera)], roi, Scalar(255, 255, 255), 0);

      if (ballType == 0) {
        redBallDetector(roi);
      } else if (ballType == 1) {
        for (auto& br : ballRegions) {
          if (!br) continue;
          br->rect = br->rect & roi;
          if (br->rect.area() <= 100) {
            br.reset();
            continue;
          }
          br->resetParams();
        }
      }

      findCandidatesWithBallFeatures(ballRegions, bestCandidates, poorCandidates);
      for (const auto& c : bestCandidates) {
        rectangle(bgrMat[toUType(activeCamera)], c, Scalar(255, 0, 0), 2);
      }
      for (const auto& c : poorCandidates) {
        rectangle(bgrMat[toUType(activeCamera)], c, Scalar(0, 0, 255), 2);
      }
      if (!bestCandidates.empty()) {
        classifyRegionsCNN(bestCandidates);
      } else {
        classifyRegionsCascade(poorCandidates);
      }
      ///< Perform ball tracker correction step
      ballTracker->updateFilter(classifiedBalls);
      ///< Update ball info in memory
      updateBallInfo();
      drawResults();
    }
    duration<double> timeSpan = high_resolution_clock::now() - tStart;
    processTime = timeSpan.count();
    if (GET_DVAR(int, displayInfo)) {
      LOG_INFO("BallExtraction Results:");
      LOG_INFO("Time taken by processImage(): " << processTime);
      LOG_INFO("Time taken by resetBallTracker(): " << resetBallTrackerTime);
      LOG_INFO("Time taken by findBallFromPredState(): " << findBallFromPredStateTime);
      LOG_INFO("Time taken by findBallRegions(): " << findBallRegionsTime);
      LOG_INFO("Time taken by filterFromRobotRegions(): " << filterFromRobotRegionsTime);
      LOG_INFO("Time taken by findCandidatesWithBallFeatures(): " << findCandidatesWithBallFeaturesTime);
      LOG_INFO("Time taken by classifyRegionsCascade(): " << classifyRegionsCascadeTime);
      LOG_INFO("Time taken by classifyRegionsCNN(): " << classifyRegionsCNNTime);
      LOG_INFO("Time taken by redBallDetector(): " << redBallDetectorTime);
      LOG_INFO("Time taken by updateBallInfo(): " << updateBallInfoTime);
      LOG_INFO("bestCandidates found: " << bestCandidates.size());
      LOG_INFO("poorCandidates found: " << poorCandidates.size());
      LOG_INFO("classifiedBalls found: " << classifiedBalls.size());
    }
    if (GET_DVAR(int, displayOutput)) {
      VisionUtils::displayImage(name, bgrMat[toUType(activeCamera)]);
      waitKey(0);
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

void BallExtraction::findBallROIFromPredState(Rect& roi, Mat& predState)
{
  auto tStart = high_resolution_clock::now();
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

  roi.width = predState.at<float>(10) * predictedAreaRoiRatio;
  roi.height = predState.at<float>(11) * predictedAreaRoiRatio;
  roi.x = predState.at<float>(6) - roi.width / 2;
  roi.y = predState.at<float>(7) - roi.height / 2;
  roi = roi & Rect(0, 0, getImageWidth(), getImageHeight());
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  findBallFromPredStateTime = timeSpan.count();
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
    classifiedBalls.push_back(ball);
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
  redBallDetectorTime = timeSpan.count();
}

void BallExtraction::findBallRegions(vector<ScannedRegionPtr>& ballRegions)
{
  auto tStart = high_resolution_clock::now();
  //! Filter out horizontal lines above border
  if (activeCamera == CameraId::headTop) {
    auto& verBallLines = regionSeg->getVerticalScan(ScanTypes::robot)->scanLines;
    auto& horBallLines = regionSeg->getHorizontalScan(ScanTypes::robot)->scanLines;

    //! Remove horizontal lines above border average height in image
    const auto& fh = fieldExt->getFieldHeight();
    for (auto& rl : horBallLines) {
      if (rl && rl->baseIndex < fh)
        rl.reset();
    }

    const auto& border = fieldExt->getBorder();
    //! Remove vertical lines above border in image
    for (auto& rl : verBallLines) {
      if (!rl) continue;
      rl->start = max(rl->start, border[rl->baseIndex]);
    }

    if (GET_DVAR(int, drawScannedLines)) {
      //! Draw vertical scan lines
      regionSeg->getVerticalScan(ScanTypes::robot)->draw(bgrMat[toUType(activeCamera)]);
      //! Draw horizontal scan lines
      regionSeg->getHorizontalScan(ScanTypes::robot)->draw(bgrMat[toUType(activeCamera)]);
    }

    vector<ScannedRegionPtr> verBallRegions;
    vector<ScannedRegionPtr> horBallRegions;

    auto horLineLinkXTol = regionSeg->getHorizontalScan(ScanTypes::robot)->highStep * lineLinkHorXTolRatio; // pixels
    auto horLineLinkYTol = regionSeg->getHorizontalScan(ScanTypes::robot)->highStep * lineLinkHorYTolRatio; // pixels
    auto verLineLinkXTol = regionSeg->getVerticalScan(ScanTypes::robot)->highStep * lineLinkVerXTolRatio; // pixels
    auto verLineLinkYTol = regionSeg->getVerticalScan(ScanTypes::robot)->highStep * lineLinkVerYTolRatio; // pixels

    //! Find regions based on horizontal scan lines
    findRegions(
      horBallRegions,
      horBallLines,
      horLineLinkXTol,
      horLineLinkYTol,
      maxLineLengthDiffRatio,
      bgrMat[toUType(activeCamera)]);

    //! Find regions based on vertical scan lines
    findRegions(
      verBallRegions,
      verBallLines,
      verLineLinkXTol,
      verLineLinkYTol,
      maxLineLengthDiffRatio,
      bgrMat[toUType(activeCamera)]);

    if (GET_DVAR(int, drawUnlinkedScannedRegions)) {
      ScannedRegion::drawRegions(
        bgrMat[toUType(activeCamera)], verBallRegions, Scalar(255,0,0));
      ScannedRegion::drawRegions(
        bgrMat[toUType(activeCamera)], horBallRegions, Scalar(0,255,0));
    }

    horBallRegions.insert(
      horBallRegions.end(),
      verBallRegions.begin(),
      verBallRegions.end());

    ScannedRegion::linkRegions(
      ballRegions,
      horBallRegions,
      regionsXDiffTol,
      regionsYDiffTol,
      maxRegionSizeDiffRatio,
      bgrMat[toUType(activeCamera)]);
  } else {
    //! Ball regions are obtained from robot regions directly for lower camera
    ballRegions = robotExt->getLowerCamRobotRegions();
  }

  //! Remove out regions too small in size or having bad aspect ratio
  for (auto& br : ballRegions) {
    if (br) {
      if (br->rect.width < regionFilterMinWidth ||
          br->rect.height < regionFilterMinHeight ||
          br->rect.width > regionFilterMaxAspectRatio * br->rect.height ||
          br->rect.height > regionFilterMaxAspectRatio * br->rect.width)
      {
        br.reset();
        continue;
      }
    }
    if (GET_DVAR(int, drawScannedRegions))
      br->draw(bgrMat[toUType(activeCamera)], Scalar(255, 255, 255));
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  findBallRegionsTime = timeSpan.count();
}

void BallExtraction::filterFromRobotRegions(
  vector<boost::shared_ptr<ScannedRegion> >& ballRegions)
{
  auto tStart = high_resolution_clock::now();
  const auto& robotRegions = robotExt->getRobotRegions();
  auto maxWidth = getImageWidth() * maxBallRegionSizeRatio;
  auto maxHeight = getImageHeight() * maxBallRegionSizeRatio;
  for (const auto& rr : robotRegions) {
    //! Make a rect for upper region
    auto top = rr->sr->rect;
    auto bottom = rr->bodySr->rect;
    if (
      rr->obstacleType == ObstacleType::teammate ||
      rr->obstacleType == ObstacleType::opponent)
    {
      //! Make a rect for lower region
      bottom.height *= robotFilterHeightRatio;
    } else if (
      rr->obstacleType == ObstacleType::teammateFallen ||
      rr->obstacleType == ObstacleType::opponentFallen)
    {
      bottom.height *= fallenRobotFilterHeightRatio;
    }

    //rectangle(bgrMat[toUType(activeCamera)], top, Scalar(255,255,255), 2);
    //rectangle(bgrMat[toUType(activeCamera)], bottom, Scalar(255,255,255), 2);
    for (auto& br : ballRegions) {
      if (!br) continue;
      auto topOverlap = (br->rect & top);
      auto topArea = topOverlap.area();
      auto topOverlapRatio = topArea / (float) br->rect.area();
      if (topOverlapRatio >= robotMaxOverlapTop) {
        br.reset();
        continue;
      }

      auto bottomOverlap = (br->rect & bottom);
      auto bottomArea = bottomOverlap.area();
      auto bottomOverlapRatio = bottomArea / (float) br->rect.area();
      if (bottomOverlapRatio >= robotMaxOverlapBottom) {
        br.reset();
        continue;
      }

      if (topArea <= minOverlapAreaThreshold &&
          bottomArea <= minOverlapAreaThreshold)
      {
        continue;
      }

      //rectangle(bgrMat[toUType(activeCamera)], topOverlap, Scalar(0,255,0), 2);
      //br->draw(bgrMat[toUType(activeCamera)], Scalar(0, 0, 0), 2);
      //VisionUtils::displayImage("bgr", bgrMat[toUType(activeCamera)]);
      //waitKey(0);

      if (topArea > 0) {
        if (topOverlap.width / (float) br->rect.width >=
            topOverlap.height / (float) br->rect.height)
        {
          auto overlapBaseY = topOverlap.y + topOverlap.height;
          if (overlapBaseY < br->leftBase.y) {
            br->rect.y = (topOverlap.y + topOverlap.height);
            br->rect.height = br->leftBase.y - br->rect.y;
          } else if (topOverlap.y > br->rect.y) {
            br->rect.height -= topOverlap.height;
          }

          br->resetParams();
          //br->draw(bgrMat[toUType(activeCamera)], Scalar(255, 0, 0), 4);
        } else {
          auto overlapEndX = topOverlap.x + topOverlap.width;
          if (overlapEndX < br->rightBase.x) {
            br->rect.x = (topOverlap.x + topOverlap.width);
            br->rect.width = br->rightBase.x - br->rect.x;
          } else if (topOverlap.x > br->rect.x) {
            br->rect.width -= topOverlap.width;
          }
          br->resetParams();
          //br->draw(bgrMat[toUType(activeCamera)], Scalar(0, 0, 255), 4);
        }
      }

      bottomOverlap = (br->rect & bottom);
      if (bottomArea > 0) {
        if (bottomOverlap.area() > minOverlapAreaThreshold) {
          if (br->rect.height == bottomOverlap.height) {
            br.reset();
            continue;
          }

          auto overlapBaseY = topOverlap.y + bottomOverlap.height;
          if (overlapBaseY < br->leftBase.y) {
            br->rect.y = (bottomOverlap.y + bottomOverlap.height);
            br->rect.height = br->leftBase.y - br->rect.y;
          } else if (bottomOverlap.y > br->rect.y) {
            br->rect.height -= bottomOverlap.height;
          }

          br->resetParams();
          //br->draw(bgrMat[toUType(activeCamera)], Scalar(0, 255, 255), 4);
        }
      }
      //br->draw(bgrMat[toUType(activeCamera)], Scalar(0, 255, 255), 4);

      if (br->rect.width > maxWidth || br->rect.height > maxHeight) {
        br.reset();
        continue;
      }
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  filterFromRobotRegionsTime = timeSpan.count();
  //for (auto& br : ballRegions) {
  //  if (br) br->draw(bgrMat[toUType(activeCamera)], Scalar(0, 255, 0), 2);
  //}
  //VisionUtils::displayImage("bgr", bgrMat[toUType(activeCamera)]);
  //waitKey(0);
}

void BallExtraction::findCandidatesWithBallFeatures(
  vector<boost::shared_ptr<ScannedRegion> >& ballRegions,
  vector<Rect>& bestCandidates,
  vector<Rect>& poorCandidates)
{
  auto tStart = high_resolution_clock::now();
  for (const auto& br : ballRegions) {
    if (br) {
      //! Increase the region size for given padding ratio
      auto& r = br->rect;
      r -= Point(r.width / (2 * ballRegionPaddingRatio), r.height / (2 * ballRegionPaddingRatio));
      r += Size(r.width / ballRegionPaddingRatio, r.height / ballRegionPaddingRatio);
      r = r & Rect(0, 0, getImageWidth(), getImageHeight());

      //! Region offset in original image
      auto offset = Point(r.x, r.y);

      //! Get gray scale image
      auto gray = getGrayImage()(r);

      //! Get the expected ball size in image for this region
      auto center = Point(gray.rows / 2, gray.cols / 2) + offset;
      Point2f centerWorld;
      cameraTransforms[toUType(activeCamera)]->imageToWorld(centerWorld, center, 0.05);
      auto worldCenterL = cv::Point3_<float>(centerWorld.x, centerWorld.y - this->ballRadius, 0.05);
      auto worldCenterR = cv::Point3_<float>(centerWorld.x, centerWorld.y + this->ballRadius, 0.05);
      cv::Point_<float> imageL, imageR;
      cameraTransforms[toUType(activeCamera)]->worldToImage(worldCenterL, imageL);
      cameraTransforms[toUType(activeCamera)]->worldToImage(worldCenterR, imageR);
      Rect ballExpected;
      ballExpected.x = min(imageL.x, imageR.x);
      ballExpected.width = abs(imageR.x - imageL.x);
      ballExpected.height = ballExpected.width;
      ballExpected.y = center.y - ballExpected.height / 2;

      //! Window size for adaptive threshold
      int windowSize = ballExpected.width / adaptiveThresholdWindowSizeRatio;

      //! Get it into odd number
      windowSize = windowSize % 2 == 0 ? windowSize + 1 : windowSize;

      //! Subtraction constant
      int subC = ballExpected.width / adaptiveThresholdSubConstantRatio;

      //! Get binary image using adaptive threshold
      Mat binary;
      adaptiveThreshold(gray, binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, windowSize, subC);

      vector<vector<Point>> contours;
      vector<Vec4i> hierarchy;
      findContours(
        binary.clone(), //! Clone the image since it is needed later on and findContours destroys it
        contours,
        hierarchy,
        CV_RETR_TREE,
        CV_CHAIN_APPROX_SIMPLE,
        Point());

      auto maxBlobSize = maxBallBlobSizeRatio * ballExpected.width;
      auto minBlobSize = minBallBlobSizeRatio * ballExpected.width;
      vector<boost::shared_ptr<Blob> > pentagons;
      for (size_t i = 0; i < contours.size(); i++) {
        //! Make a bounding box for contour
        Rect boundRect = boundingRect(contours[i]);

        //! Ignore contour if it is too small
        if (
            boundRect.width < minBlobSize ||
            boundRect.height < minBlobSize)
        {
          continue;
        }

        //! Ignore contour if its width is close to expected width but set it as ball expected width
        if (fabsf(boundRect.width - ballExpected.width) / ballExpected.width < 0.3) {
          ballExpected.width = max(boundRect.width, ballExpected.width);
          continue;
        }

        //! Ignore contour if its height is close to expected height but set it as ball expected height
        if (fabsf(boundRect.height - ballExpected.height) / ballExpected.height < 0.3) {
          ballExpected.height = max(boundRect.height, ballExpected.height);
          continue;
        }

        //! Ignore contour if it is too big wrt expected ball size
        if (
            boundRect.width > maxBlobSize ||
            boundRect.height > maxBlobSize)
        {
          continue;
        }

        //! Ignore if region aspect ratio is above max ratio
        auto ratio = boundRect.width / (float) boundRect.height;
        if (1.0 / ratio > ballBlobMaxAspectRatio || ratio > ballBlobMaxAspectRatio)
          continue;

        //! Ignore if mean intensity of the blob is above the threshold
        if (mean(gray(boundRect))[0] > maxBallBlobIntensity)
          continue;

        //! Get the contour center
        auto pc =
          Point (boundRect.x + boundRect.width * 0.5,
                 boundRect.y + boundRect.height * 0.5);

        if (GET_DVAR(int, drawClassifiedPentagons))
          drawContours(bgrMat[toUType(activeCamera)], contours, i, Scalar(0,255,255), -1, 8, hierarchy, 0, offset);

        //! Add contour as a ball blob
        pentagons.push_back(boost::make_shared<Blob> (boundRect, pc, contours[i]));
      }

      //! Ignore if the number of pentagons are too many or less than required
      if (pentagons.size() < minPentagonsRequired ||
          pentagons.size() >= maxPentagonsRequired)
      {
        if (pentagons.size() > minPentagonsPoorCandidates) {
          auto center = (pentagons[0]->center + pentagons[1]->center) * 0.5;
          auto radius = cv::norm(pentagons[0]->center - pentagons[1]->center) * 2;
          ballExpected.x = center.x - radius + offset.x;
          ballExpected.y = center.y - radius + offset.y;
          ballExpected.width = radius * 2;
          ballExpected.height = radius * 2;
          poorCandidates.push_back(ballExpected);
        }
        continue;
      }

      //! Make triangle combinations
      static const int triangleN = 3;
      vector<boost::shared_ptr<Triangle>> triangles;
      vector<bool> v(pentagons.size());
      fill(v.end() - triangleN, v.end(), true);
      do {
          triangles.push_back(boost::make_shared<Triangle>());
          auto& t = triangles.back();
          auto sizeMean = 0.f;
          for (size_t i = 0; i < pentagons.size(); ++i) {
              if (v[i]) {//! A triangle combination
                t->points.push_back(pentagons[i]->center);
                t->pentagons.push_back(pentagons[i]);
                sizeMean += pentagons[i]->size;
              }
          }

          //! Mean size of combination pentagons
          sizeMean /= (float) triangleN;
          for (const auto& p : t->pentagons) {
            //! Ignore combination if their sizes are too far apart
            if (fabsf(p->size - sizeMean) / sizeMean > maxBlobToBlobSizeDiffRatio) {
              t.reset();
              continue;
            }
          }
      } while (std::next_permutation(v.begin(), v.end()));

      //! Filter out pentagon triangles a bit more
      for (auto& t : triangles) {
        if (!t) continue;
        auto d1 = norm(t->points[0] - t->points[1]);
        auto d2 = norm(t->points[1] - t->points[2]);
        auto d3 = norm(t->points[2] - t->points[0]);

        //! Cosine law
        auto a1 = acos((d2*d2 + d3*d3 - d1*d1) / (2.f * d2 * d3));
        auto a2 = acos((d1*d1 + d3*d3 - d2*d2) / (2.f * d1 * d3));

        //! Triangle property, find the largest angle
        auto a3 = Angle::DEG_180 - (a1 + a2);
        auto maxA = max(max(a1, a2), a3);
        if (maxA < Angle::DEG_90) { //! Maximum angle is at least below 90 degrees
          for (const auto& p : pentagons) { //! Check if there is another pentagon in the triangle from all the pentagons
            //! Ignore pentagons associated with this triangle
            if (std::find(t->pentagons.begin(), t->pentagons.end(), p) != t->pentagons.end())
              continue;

            //! Check if exists between the region of this triangle
            if (cv::pointPolygonTest(t->points, Point2f(p->center.x, p->center.y), false) >= 0) {
              t.reset();
              break;
            }
          }
        } else {
          t.reset();
        }
      }

      //! Take the inverse of binary image
      cv::bitwise_not(binary, binary);
      for (auto& t : triangles) {
        if (!t) continue;
        if (GET_DVAR(int, drawClassifiedTriangles)) {
          vector<vector<Point> > triangles;
          triangles.push_back(t->points);
          drawContours(bgrMat[toUType(activeCamera)], triangles, 0, Scalar(255, 0, 0), 1, 8, hierarchy, 0, offset);
        }
        auto centroid = (t->points[0] + t->points[1] + t->points[2]) * (1.0 / 3.0);
        vector<vector<Point> > contours;
        for (size_t i = 0; i < triangleN; ++i) {
          contours.push_back(t->pentagons[i]->contour);
          //! This draw command is a part of algorithm, do not comment
          //! Draw pentagons on binary image
          drawContours(binary, contours, i, Scalar(255), -1);
        }
        //! Smooth out the image
        GaussianBlur(binary, binary, Size(gaussianSizeX, gaussianSizeY), gaussianSigmaX, gaussianSigmaY);

        //VisionUtils::displayImage("binary", binary);
        vector<Vec3f> circles;
        auto expRadius = max(ballExpected.width, ballExpected.height);
        HoughCircles(
          binary,
          circles,
          CV_HOUGH_GRADIENT,
          2,
          binary.rows / houghCirclesMinDistRatio,
          max(10, expRadius - minRadiusPixelTolerance),
          expRadius + maxRadiusPixelTolerance);

        //! If a circle is found, set it as expected ball bounding box
        if (!circles.empty()) {
          for (const auto& c : circles)
          {
            Point center(cvRound(c[0]), cvRound(c[1]));
            int radius = cvRound(c[2]);

            if (GET_DVAR(int, drawBallCircles)) {
              circle(bgrMat[toUType(activeCamera)], center, radius, Scalar(255, 0, 0));
            }

            if (cv::norm(centroid - center) < ballExpected.width / triangleToCircleMaxDistRatio) { // center to centroid minimum
              ballExpected.x = center.x - radius + offset.x;
              ballExpected.y = center.y - radius + offset.y;
              ballExpected.width = radius * 2;
              ballExpected.height = radius * 2;
              bestCandidates.push_back(ballExpected);
            }
          }
        } else {
          ballExpected.x = centroid.x - ballExpected.width / 2 + offset.x;
          ballExpected.y = centroid.y - ballExpected.height / 2 + offset.y;
          poorCandidates.push_back(ballExpected);
        }
      }
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  findCandidatesWithBallFeaturesTime = timeSpan.count();
}

#define CNN_IMAGES_MEAN 80.98907329963235
#define CNN_IMAGES_STD 74.59484120515009
#define CNN_INPUT_IMAGE_SIZE 32

void BallExtraction::classifyRegionsCNN(vector<Rect>& boundRects)
{
  auto tStart = high_resolution_clock::now();
  for (auto& br : boundRects) {
    br = br & Rect(0, 0, getImageWidth(), getImageHeight());
    Mat cropped = getGrayImage()(br);
    resize(cropped, cropped, Size(CNN_INPUT_IMAGE_SIZE, CNN_INPUT_IMAGE_SIZE));

    Mat floatMat = Mat(cropped.size(), CV_32F);
    cropped.convertTo(floatMat, CV_32F);
    floatMat = (floatMat - CNN_IMAGES_MEAN) / CNN_IMAGES_STD;
    float* inputMat = floatMat.ptr<float>();
    float* input = interpreter->typed_input_tensor<float>(0);
    for (int i = 0; i < CNN_INPUT_IMAGE_SIZE * CNN_INPUT_IMAGE_SIZE; ++i) {
      *input++ = *inputMat++;
    }
    interpreter->Invoke();
    float* ballLabel = interpreter->typed_output_tensor<float>(0);
    if (fabsf(*(++ballLabel) - 1.f) < CNNClassificationTolerance) {
      classifiedBalls.push_back(br);
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  classifyRegionsCNNTime = timeSpan.count();
}

void BallExtraction::classifyRegionsCascade(vector<Rect>& boundRects)
{
  auto tStart = high_resolution_clock::now();
  for (auto& br : boundRects) {
    auto wHRatio = br.width / (float) br.height;
    auto newCenter = Point(br.x + br.width / 2, br.y + br.height / 2);
    br = Rect(
      newCenter - Point(br.width / wHRatio / 2, br.height / 2),
      Size(br.width / wHRatio, br.height));
    br -= Point(br.width / (2 * cascadePaddingRatio), br.height / (2 * cascadePaddingRatio));
    br += Size(br.width / cascadePaddingRatio, br.height / cascadePaddingRatio);
    br = br & Rect(0, 0, getImageWidth(), getImageHeight());
    Mat cropped = getGrayImage()(br);

    auto ratioY = cropped.rows / (float)br.height;
    auto ratioX = cropped.cols / (float)br.width;

    vector<Rect> ballsFound;
    cascade.detectMultiScale(cropped, ballsFound, 1.05, 1);
    for (auto& b : ballsFound) {
      b.x = b.x / ratioX + br.x;
      b.y = b.y / ratioY + br.y;
      b.width = b.width / ratioX;
      b.height = b.height / ratioY;
      classifiedBalls.push_back(b);
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  classifyRegionsCascadeTime = timeSpan.count();
}

void BallExtraction::applyClassifier(const Rect& origRect, Mat& croppedImage)
{
  if (croppedImage.cols < 5 || croppedImage.rows < 5) return;

  /*float* input = interpreter->typed_input_tensor<float>(0);
  float ratioY = croppedImage.rows;
  float ratioX = croppedImage.cols;
  //vector<Rect> balls;
  auto centerX = croppedImage.rows / 2;
  auto centerY = croppedImage.cols / 2;
  croppedImage = croppedImage(Rect(centerX - 24, centerY - 24, 48, 48));
  resize(croppedImage, croppedImage, cv::Size(32, 32));
  ratioY = croppedImage.rows / ratioY;
  ratioX = croppedImage.cols / ratioX;*/
  //cascade.detectMultiScale(croppedImage, balls, 1.05, 1);
  //cout << "balls: " << balls.size() << endl;
  //for (size_t i = 0; i < balls.size(); ++i) {
  //  auto ball = balls[i];
    /*rectangle(
      croppedImage,
      ball,
      cv::Scalar(255,255,255),
      1
    );*/
  //  ball.x = ball.x / ratioX + origRect.x;
 //   ball.y = ball.y / ratioY + origRect.y;
 //   ball.width = ball.width / ratioX;
 //   ball.height = ball.height / ratioY;
 //   classifiedBalls.push_back(ball);
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
 // }
  /*Mat floatMat = Mat(croppedImage.size(), CV_32F);
  croppedImage.convertTo(floatMat, CV_32F);
  float mean = 80.98907329963235;
  float std = 74.59484120515009;
  floatMat = (floatMat - mean) / std;
  float* inputMat = floatMat.ptr<float>();
  for (int i = 0; i < 32*32; ++i) {
    *input++ = *inputMat++;
  }

  // Fill `input`.
  auto start = high_resolution_clock::now();
  interpreter->Invoke();
  duration<double> td = high_resolution_clock::now() - start;
  cout << "time: " << td.count() << endl;
  printf("=== Post-invoke Interpreter State ===\n");
  float* ballLabel = interpreter->typed_output_tensor<float>(0);
  cout << "ballLabel1: " << *ballLabel << endl;
  cout << "ballLabel2: " << *(++ballLabel) << endl;
  VisionUtils::displayImage("cropped", croppedImage);
  waitKey(0);*/
  /*ballLabel++;
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
    classifiedBalls.push_back(ball);*/
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
    for (int i = 0; i < classifiedBalls.size(); ++i) {
      rectangle(bgrMat[toUType(activeCamera)], classifiedBalls[i], Scalar(0,0,255), 3);
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
  classifiedBalls.push_back(ball);
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
