/**
 * @file VisionModule/src/FeatureExtraction/GoalExtraction.cpp
 *
 * This file implements the class for goal extraction from
 * the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017
 */

#include "TNRSBase/include/MemoryIOMacros.h"
#include "LocalizationModule/include/FieldLandmarkIds.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/DataHolders/GoalInfo.h"
#include "Utils/include/DataHolders/Landmark.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/DataHolders/ObstacleType.h"
#include "VisionModule/include/CameraTransform.h"
#include "VisionModule/include/ColorHandler.h"
#include "VisionModule/include/FeatureExtraction/RegionSegmentation.h"
#include "VisionModule/include/FeatureExtraction/RegionScanners.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/FittedLine.h"
#include "VisionModule/include/FeatureExtraction/GoalPost.h"
#include "VisionModule/include/FeatureExtraction/GoalExtraction.h"
#include "VisionModule/include/FeatureExtraction/RobotExtraction.h"
#include "VisionModule/include/FeatureExtraction/RobotRegion.h"
#include "VisionModule/include/FeatureExtraction/ScannedLine.h"
#include "VisionModule/include/FeatureExtraction/ScannedRegion.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/MathsUtils.h"

using namespace MathsUtils;

GoalExtraction::GoalExtraction(VisionModule* visionModule) :
  FeatureExtraction(visionModule, "GoalExtraction"), DebugBase("GoalExtraction", this)
{
  initDebugBase();
  GET_DEBUG_CONFIG(
    VisionConfig,
    GoalExtraction,
    (int, sendTime),
    (int, drawScannedLines),
    (int, drawScannedRegions),
    (int, drawGoalBaseWindows),
    (int, drawShiftedBorderLines),
    (int, drawShiftedBorderLines),
    (int, displayInfo),
    (int, displayOutput),
  );

  GET_CLASS_CONFIG(
    VisionConfig,
    GoalExtraction,
    lineLinkHighStepRatio,
    lineLinkLowToHighStepRatio,
    maxLineLengthDiffRatio,
    goalBaseToBorderDistance,
    goalPostMinWidth,
    goalPostMinArea,
    maxHeightToWidthRatio,
    minGoalPostWidthWorld,
    maxGoalPostWidthWorld,
    goalPostWindowHeight,
    minWindowGreenRatio,
    minGoalPostImageDist,
    minGoalToGoalDist,
    maxGoalToGoalDist,
    actualGoalToGoalDist,
    maxGoalKeeperParallelDist,
    maxGoalKeeperPerpDist,
    refreshTime,

  );
  regionSeg = GET_FEATURE_EXT_CLASS(RegionSegmentation, FeatureExtractionIds::segmentation);
  fieldExt = GET_FEATURE_EXT_CLASS(FieldExtraction, FeatureExtractionIds::field);
  robotExt = GET_FEATURE_EXT_CLASS(RobotExtraction, FeatureExtractionIds::robot);

  ///< Initializing processing times
  processTime= 0.0;
  scanTime= 0.0;
  classifyPostsTime= 0.0;
  findBestPostsTime= 0.0;
  updateGoalInfoTime= 0.0;
}

void GoalExtraction::processImage()
{
  if (activeCamera == CameraId::headBottom)
    return;
  auto tStart = high_resolution_clock::now();
  if (fieldExt->isFound()) {
    refreshGoalPosts();
    vector<LinearScannedLinePtr> horGoalLines;
    if (filterGoalLines(horGoalLines)) {
      vector<vector<Point>> horGoalHulls;
      auto lineLinkYTol =
        regionSeg->getHorizontalScan(ScanTypes::goal)->highStep * lineLinkHighStepRatio; // pixels
      auto lineLinkXTol = lineLinkYTol * lineLinkLowToHighStepRatio;
      findConvexHulls(
        horGoalHulls,
        horGoalLines,
        lineLinkXTol,
        lineLinkYTol,
        maxLineLengthDiffRatio,
        bgrMat[toUType(activeCamera)]);
      for (const auto& h : horGoalHulls) {
        if (GET_DVAR(int, drawScannedRegions)) {
          auto r = boundingRect(h);
          rectangle(bgrMat[toUType(activeCamera)], r, Scalar(0,255,0), 1);
        }
      }
      classifyPosts(horGoalHulls);
      findBestPosts();
      updateGoalInfo();
      drawResults();
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  processTime = timeSpan.count();
  if (GET_DVAR(int, displayInfo)) {
    LOG_INFO("GoalExtraction Results:");
    LOG_INFO("Time taken by image scan: " << scanTime);
    LOG_INFO("Time taken by posts classification: " << classifyPostsTime);
    LOG_INFO("Time taken by finding best description of posts" << findBestPostsTime);
    LOG_INFO("Time taken by updating goal info to memory" << updateGoalInfoTime);
    LOG_INFO("Time taken by overall processing: " << processTime);
    LOG_INFO("Goal posts found: " << goalPosts.size());
    LOG_INFO("Goal found: " << GOAL_INFO_OUT(VisionModule).found);
  }
  if (GET_DVAR(int, displayOutput)) {
    VisionUtils::displayImage(name, bgrMat[toUType(activeCamera)]);
    waitKey(0);
  }
}

void GoalExtraction::refreshGoalPosts()
{
  float currentTime = visionModule->getModuleTime();
  GPIter iter = goalPosts.begin();
  while (iter != goalPosts.end()) {
    if (currentTime - (*iter)->timeDetected < refreshTime) {
      //Matrix<float, 3, 1> prevImage;
      //prevImage(0, 0) = (*iter)->image.x;
      //prevImage(1, 0) = (*iter)->image.y;
      //prevImage(2, 0) = 1.0;

      Point3f goalWorld = Point3f((*iter)->world.x, (*iter)->world.y, 0.0);
      Point2f goalImage;
      cameraTransforms[toUType(activeCamera)]->worldToImage(goalWorld, goalImage);
      (*iter)->image.x = goalImage.x;
      (*iter)->image.y = goalImage.x;
      ++iter;
    } else {
      iter = goalPosts.erase(iter);
    }
  }
}

bool GoalExtraction::filterGoalLines(vector<LinearScannedLinePtr>& horGoalLines)
{
  auto tStart = high_resolution_clock::now();
  auto border = fieldExt->getBorder();
  if (border.empty()) {
    duration<double> timeSpan = high_resolution_clock::now() - tStart;
    scanTime = timeSpan.count();
    return false;
  }
  auto borderLines = fieldExt->getBorderLinesWorld();
  if (borderLines.empty())
    return false;
  vector<double> borderSlopes;
  vector<double> borderCs;
  vector<Vec4f> shiftedBorders;
  sort(borderLines.begin(), borderLines.end(),
    [](const FittedLinePtr& fl1, const FittedLinePtr& fl2) {
      return (fl1->p1.x < fl1->p2.x);
    });
  for (const auto& borderLine : borderLines) {
    // Distance from field to border is 70cm
    Point2f goalBaseP1 = borderLine->p1 - borderLine->perp * goalBaseToBorderDistance;
    Point2f goalBaseP2 = borderLine->p2 - borderLine->perp * goalBaseToBorderDistance;
    cameraTransforms[toUType(activeCamera)]->worldToImage(
      Point3f(goalBaseP1.x, goalBaseP1.y, 0.0), goalBaseP1);
    cameraTransforms[toUType(activeCamera)]->worldToImage(
          Point3f(goalBaseP2.x, goalBaseP2.y, 0.0), goalBaseP2);
    auto slope = (goalBaseP2.y - goalBaseP1.y) / (goalBaseP2.x - goalBaseP1.x);
    borderSlopes.push_back(slope);
    borderCs.push_back(-slope * goalBaseP1.x + goalBaseP1.y);
    shiftedBorders.push_back(Vec4f(goalBaseP1.x, goalBaseP1.y, goalBaseP2.x, goalBaseP2.y));
    if (GET_DVAR(int, drawShiftedBorderLines)) {
      line(
        bgrMat[toUType(activeCamera)],
        Point(goalBaseP1.x, goalBaseP1.y),
        Point(goalBaseP2.x, goalBaseP2.y),
        Scalar(0, 0, 255), 1);
    }
  }
  Point2f intersection;
  if (shiftedBorders.size() >= 2)
    VisionUtils::findIntersection(shiftedBorders[0], shiftedBorders[1], intersection);

  horGoalLines = regionSeg->getHorizontalScan(ScanTypes::goal)->scanLines;
  auto robotRegions = robotExt->getRobotRegions();
  for (auto& gl : horGoalLines) {
    if (!gl) continue;
    auto mean = (gl->start+gl->end)/2;
    //VisionUtils::drawPoint(Point((gl->start+gl->end)/2, slope * (gl->start+gl->end)/2 + lineC), bgrMat[toUType(activeCamera)]);
    if (shiftedBorders.size() >= 2) {
      if (mean < intersection.x) {
        if (gl->baseIndex > borderSlopes[0] * mean + borderCs[0]) {
          gl.reset();
          continue;
        }
      } else {
        if (gl->baseIndex > borderSlopes[1] * mean + borderCs[1]) {
          gl.reset();
          continue;
        }
      }
    } else {
      if (gl->baseIndex > borderSlopes[0] * mean + borderCs[0]) {
        gl.reset();
        continue;
      }
    }
    for (const auto& rr : robotRegions) {
      if (!rr) continue;
      if (rr->sr->rect.contains(Point(mean, gl->baseIndex))) {
        gl.reset();
        break;
      }
      if (rr->bodySr->rect.contains(Point(mean, gl->baseIndex))) {
        gl.reset();
        break;
      }
    }
  }
  if (GET_DVAR(int, drawScannedLines)) {
    for (auto& gl : horGoalLines) {
      if (gl) gl->draw(bgrMat[toUType(activeCamera)], Scalar(0,255,0));
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  scanTime = timeSpan.count();
  return true;
}

void GoalExtraction::classifyPosts(vector<vector<Point>>& horGoalHulls)
{
  auto tStart = high_resolution_clock::now();
  vector<GoalPostPtr> newPosts;
  for (auto& gh : horGoalHulls) {
    if (gh.size() < 4) continue;
    auto r = boundingRect(gh);
    //rectangle(bgrMat[toUType(activeCamera)], r, Scalar(0,255,0), 2);
    if (r.width < goalPostMinWidth || r.area() < goalPostMinArea) continue;
    if (r.height / (float) r.width < maxHeightToWidthRatio) continue;
    sort(
      gh.begin(),
      gh.end(),
      [](const Point& p1, const Point& p2) {
        return (p1.y > p2.y) ||
        ((p1.y == p2.y) && (p1.x < p2.x));
      });
    vector<Point2f> regionBase, regionBaseWorld;
    regionBase.push_back(Point2f(gh[0].x, gh[0].y));
    regionBase.push_back(Point2f(gh[1].x, gh[1].y));
    cameraTransforms[toUType(activeCamera)]->imageToWorld(regionBaseWorld, regionBase, 0.0);
    auto width = norm(regionBaseWorld[0] - regionBaseWorld[1]);
    if (width > minGoalPostWidthWorld && width < maxGoalPostWidthWorld) {
      int winX = gh[0].x;
      int winSizeX = abs(gh[1].x - gh[0].x);
      int winSizeY = goalPostWindowHeight;
      int winY = gh[0].y;
      Rect window = Rect(winX, winY, winSizeX, winSizeY);
      window = window & Rect(0, 0, getImageWidth(), getImageHeight());
      if (GET_DVAR(int, drawGoalBaseWindows)) {
        rectangle(bgrMat[toUType(activeCamera)], window, Scalar(255, 255, 255), 1);
      }
      int greenCnt = 0;
      int totalCnt = winSizeX * winSizeY;
      for (int j = winY; j < winSizeY + winY; ++j) {
        for (int k = winX; k < winSizeX + winX; ++k) {
          auto p = getYUV(k, j);
          if (colorHandler->isColor(p, TNColors::green)) {
            //bgrMat[toUType(activeCamera)].at<Vec3b>(j, k) = Vec3b(0,255,0);
            greenCnt++;
          }
        }
      }
      //cout << "greenCnt / (float) totalCnt: " << greenCnt / (float) totalCnt<< endl;
      if (greenCnt / (float) totalCnt > minWindowGreenRatio) {
        auto cW = (regionBaseWorld[0] + regionBaseWorld[1]) * 0.5;
        auto cI = Point2f((gh[0].x + gh[1].x) * 0.5, gh[0].y);
        auto newPost = boost::make_shared<GoalPost> (cW, cI);
        bool exists = false;
        for (const auto& gp : goalPosts) {
          if (gp->checkDuplicate(newPost)) {
            exists = true;
            break;
          }
        }
        if (!exists) newPosts.push_back(newPost);
      }
    }
  }
  goalPosts.insert(goalPosts.end(), newPosts.begin(), newPosts.end());
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  classifyPostsTime = timeSpan.count();
}

void GoalExtraction::findBestPosts()
{
  auto tStart = high_resolution_clock::now();
  if (!goalPosts.empty()) {
    if (goalPosts.size() > 1) {
      for (auto& gp1 : goalPosts) {
        if (!gp1) continue;
        auto gp1Image = gp1->image;
        for (auto& gp2 : goalPosts) {
          if (!gp2) continue;
          if (gp1 != gp2) {
            auto gp2Image = gp2->image;
            if (norm(gp1Image - gp2Image) < minGoalPostImageDist && gp1Image.y > gp2Image.y) {
                gp2.reset();
            }
          }
        }
      }
      GPIter iter = goalPosts.begin();
      while (iter != goalPosts.end()) {
        if (*iter) {
          ++iter;
        } else {
          iter = goalPosts.erase(iter);
        }
      }
      if (goalPosts.empty()) {
        duration<double> timeSpan = high_resolution_clock::now() - tStart;
        findBestPostsTime = timeSpan.count();
        return;
      }

      if (goalPosts.size() > 1) { // If greater than 1, find the right pair of posts.
        vector<int> bestIds(2, -1);
        auto bestDist = 0.0;
        for (int i = 0; i < goalPosts.size(); ++i) {
          if (!goalPosts[i]) continue;
          for (int j = i; j < goalPosts.size(); ++j) {
            if (!goalPosts[j]) continue;
            if (i != j) {
              float dist = norm(goalPosts[i]->world - goalPosts[j]->world);
              if (dist > minGoalToGoalDist && dist < maxGoalToGoalDist) {
                auto d1 = fabsf(dist - actualGoalToGoalDist);
                auto d2 = fabsf(bestDist - actualGoalToGoalDist);
                if (d1 < d2) {
                  bestDist = dist;
                  bestIds[0] = i;
                  bestIds[1] = j;
                }
              }
            }
          }
        }

        if (bestIds[0] != -1) {
          vector<GoalPostPtr> bestPosts;
          for (int i = 0; i < bestIds.size(); ++i) {
            bestPosts.push_back(goalPosts[bestIds[i]]);
          }
          goalPosts = bestPosts;
        }
      } else {
        addGoalPost(goalPosts[0]);
      }
    } else {
      addGoalPost(goalPosts[0]);
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  findBestPostsTime = timeSpan.count();
}

void GoalExtraction::updateGoalInfo()
{
  auto tStart = high_resolution_clock::now();
  GoalInfo<float> goalInfo;
  auto goalMid = Point2f(NAN, NAN);
  auto rightPost = Point2f(NAN, NAN);
  auto leftPost = Point2f(NAN, NAN);
  if (ON_SIDE_LINE_IN(VisionModule)) {
    if (!goalPosts.empty()) {
      if (goalPosts.size() == 2) {
        goalMid = (goalPosts[0]->world + goalPosts[1]->world) * 0.5;
        // Max norm for 3x4.5 distance from sidelines
        goalInfo.type = norm(goalMid) < 5.4083 ? GoalPostType::ourTeam : GoalPostType::opponentTeam;
      } else if (goalPosts.size() == 1) {
        // Max norm for 3.8x4.5 distance from sidelines
        goalInfo.type = norm(goalPosts[0]->world) < 5.889821729 ? GoalPostType::ourTeam : GoalPostType::opponentTeam;
      }
      if (goalInfo.type == GoalPostType::ourTeam) {
        for (const auto& gp : goalPosts) {
          if (!gp) continue;
          if (gp->world.y < 0) {
            if (gp->world.x > 3.5 && gp->world.x < 4.1) { // Goal on Right - Left Post
              leftPost = gp->world;
              goalInfo.found = true;
            } else if (gp->world.x > 1.9 && gp->world.x < 2.5) { // Goal on Right - Right Post
              rightPost = gp->world;
              goalInfo.found = true;
            }
          } else {
            if (gp->world.x > 3.5 && gp->world.x < 4.1) { // Goal on Left - Right Post
              rightPost = gp->world;
              goalInfo.found = true;
            } else if (gp->world.x > 1.9 && gp->world.x < 2.5) { // Goal on Left - Left Pos
              leftPost = gp->world;
              goalInfo.found = true;
            }
          }
        }
      } else if (goalInfo.type == GoalPostType::opponentTeam) {
        for (const auto& gp : goalPosts) {
          if (!gp) continue;
          if (gp->world.y > 0) {
            if (gp->world.x > 3.5 && gp->world.x < 4.1) // Goal on Left - Right Post
            {
              rightPost = gp->world;
            } else if (gp->world.x > 1.9 && gp->world.x < 2.5) // Goal on Left - Left Post
            {
              leftPost = gp->world;
            }
          } else {
            if (gp->world.x > 3.5 && gp->world.x < 4.1) // Goal on Left - Left Post
            {
              leftPost = gp->world;
            } else if (gp->world.x > 1.9 && gp->world.x < 2.5) // Goal on Left - Right Post
            {
              rightPost = gp->world;
            }
          }
        }
      }
    } else {
      goalInfo.found = false;
    }
    goalInfo.leftPost = leftPost;
    goalInfo.rightPost = rightPost;
    goalInfo.mid = goalMid;
  } else {
    if (goalPosts.size() == 2) {
      if (goalPosts[0]->image.x < goalPosts[1]->image.x) { ///< Post at index 0 is to the left in image
        goalInfo.leftPost = goalPosts[0]->world;
        goalInfo.rightPost = goalPosts[1]->world;
      } else {
        goalInfo.leftPost = goalPosts[1]->world;
        goalInfo.rightPost = goalPosts[0]->world;
      }
      findGoalSide(goalInfo);
      goalInfo.found = true;
    } else {
      goalInfo.found = false;
    }
  }
  float currentTime = visionModule->getModuleTime();
  for (const auto& gp : goalPosts) {
    if (gp && gp->refresh) gp->timeDetected = currentTime;
  }
  if (goalInfo.found)
    goalInfo.resetId();
  GOAL_INFO_OUT(VisionModule) = goalInfo;
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  updateGoalInfoTime = timeSpan.count();
}

void GoalExtraction::findGoalSide(GoalInfo<float>& goalInfo)
{
  goalInfo.type = GoalPostType::unknown;
  goalInfo.mid = (goalInfo.leftPost + goalInfo.rightPost) * 0.5;
  auto goalUnit = goalInfo.rightPost - goalInfo.leftPost;
  goalUnit = goalUnit / (float) cv::norm(goalUnit);
  auto robotToGoalU = goalInfo.mid / (float) cv::norm(goalInfo.mid);
  auto cross = robotToGoalU.x * goalUnit.y - robotToGoalU.y * goalUnit.x;
  auto dot = robotToGoalU.dot(goalUnit);
  auto perpAngle = atan2(cross, dot) + atan2(goalInfo.mid.y, goalInfo.mid.x);
  auto cosa = cos(perpAngle);
  auto sina = sin(perpAngle);
  goalInfo.poseFromGoal.x() = -cosa * goalInfo.mid.x - sina * goalInfo.mid.y; // From -R^t * t inverse rotation
  goalInfo.poseFromGoal.y() = +sina * goalInfo.mid.x - cosa * goalInfo.mid.y;
  goalInfo.poseFromGoal.theta() = -perpAngle;
  //cout << "goalInfo.poseFromLandmark.x: " << goalInfo.poseFromGoal.x << endl;
  //cout << "goalInfo.poseFromLandmark.y: " << goalInfo.poseFromGoal.y << endl;
  //cout << "goalInfo.poseFromLandmark.t: " << goalInfo.poseFromGoal.theta * 180 / M_PI << endl;
  // Assumes that goalInfo has both left post and right post determined
  auto robotRegions = robotExt->getRobotRegions();
  auto closest = 1000.f;
  auto distFromCenter = -1.f;
  RobotRegionPtr bestRobotRegion;
  for (const auto& rr : robotRegions) {
    if (!rr) continue;
    auto goalToOtherRobot = goalInfo.mid - rr->world;
    auto goalToPerp = abs(
      goalToOtherRobot.x * goalUnit.x + goalToOtherRobot.y * goalUnit.y);
    auto gR = cv::norm(goalToOtherRobot);
    auto dist = sqrt(gR * gR - goalToPerp * goalToPerp);
    // The perpendicular distance from robot to goal line
    //cout << "dist[" << i << "]: " << dist << endl;
    if (dist < closest) {
      closest = dist;
      distFromCenter = goalToPerp;
      bestRobotRegion = rr;
    }
  }
  if (bestRobotRegion) {
    //cout << "Robot found between goal posts" << endl;
    //cout << "distFromCenter: " << distFromCenter << endl;
    //cout << "dist:" << closest << endl;
    // If robot is within 80cm of the goal mid along the goal posts
    // in either direction and 65cm in front of it then mark it as goal keeper
    if (closest < maxGoalKeeperPerpDist && distFromCenter <maxGoalKeeperParallelDist) {
      if (bestRobotRegion->obstacleType == ObstacleType::teammate ||
          bestRobotRegion->obstacleType == ObstacleType::teammateFallen)
      {
        goalInfo.type = GoalPostType::ourTeam;
      } else if (
        bestRobotRegion->obstacleType == ObstacleType::opponent ||
        bestRobotRegion->obstacleType == ObstacleType::opponentFallen)
      {
        goalInfo.type = GoalPostType::opponentTeam;
      }
    }
  }
}

void GoalExtraction::addGoalPost(const GoalPostPtr& goalPost)
{
  addGoalObstacle(goalPost);
  addGoalLandmark(goalPost);
}

void GoalExtraction::addGoalObstacle(const GoalPostPtr& goalPost)
{
  auto robotPose2D = ROBOT_POSE_2D_IN(VisionModule);
  Obstacle<float> obs(ObstacleType::goalPost);
  obs.center = goalPost->world;
  obs.center.x += obs.depth;
  obs.centerT = robotPose2D.transform(obs.center);
  OBSTACLES_OBS_OUT(VisionModule).data.push_back(obs);
}

void GoalExtraction::addGoalLandmark(const GoalPostPtr& goalPost)
{
  auto p = goalPost->world;
  auto borderLines = fieldExt->getBorderLinesWorld();
  auto minDist = 1000;
  // Find which line intersects with the current goal post
  FittedLinePtr bestBorderLine;
  for (const auto& bl : borderLines) {
    float dist = fabsf(bl->perp.x * p.x + bl->perp.y * p.y - bl->perpDist);
    if (dist < minDist) {
      minDist = dist;
      bestBorderLine = bl;
    }
  }
  float len = norm(p);
  Point2f robotToGoalU = Point2f(p.x / len, p.y / len);
  float cross = robotToGoalU.x * bestBorderLine->unit.y - robotToGoalU.y * bestBorderLine->unit.x;
  float dot = robotToGoalU.dot(bestBorderLine->unit);
  float perpAngle = atan2(cross, dot) + atan2(p.y, p.x);
  float cosa = cos(perpAngle);
  float sina = sin(perpAngle);
  auto poseFromLandmark =
    RobotPose2D<float>(
      -cosa * p.x - sina * p.y, // From -R^t * t inverse rotation
      +sina * p.x - cosa * p.y,
      -perpAngle
    );
  auto l =
    boost::make_shared<KnownLandmark<float> >(
      FL_TYPE_GOAL_POST,
      goalPost->world,
      poseFromLandmark
    );
  //cout << "l.pos: " << l.pos << endl;
  //cout << "l.poseFromLandmark.x: " << l.poseFromLandmark.x << endl;
  //cout << "l.poseFromLandmark.y: " << l.poseFromLandmark.y << endl;
  //cout << "l.poseFromLandmark.t: " << l.poseFromLandmark.theta * 180 / M_PI << endl;
  knownLandmarks.push_back(l);
}

void GoalExtraction::drawResults()
{
  if (GET_DVAR(int, drawGoalPostBases)) {
    //if (GOAL_INFO_OUT(VisionModule).found) {
    for (const auto& gp : goalPosts)
      VisionUtils::drawPoint(gp->image, bgrMat[toUType(activeCamera)], Scalar(0,0,0));
    //}
  }
}
