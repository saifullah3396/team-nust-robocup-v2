/**
 * @file FeatureExtraction/GoalExtraction.cpp
 *
 * This file implements the class for goal extraction from 
 * the image. 
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017  
 */

#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/DataHolders/GoalInfo.h"
#include "Utils/include/DataHolders/Landmark.h"
#include "Utils/include/DataHolders/ObstacleType.h"
//#include "VisionModule/include/ColorHandler.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/FittedLine.h"
#include "VisionModule/include/FeatureExtraction/GoalPost.h"
#include "VisionModule/include/FeatureExtraction/GoalExtraction.h"
#include "VisionModule/include/FeatureExtraction/RobotExtraction.h"
#include "VisionModule/include/FeatureExtraction/RobotRegion.h"
#include "Utils/include/MathsUtils.h"

using namespace MathsUtils;

GoalExtraction::GoalExtraction(VisionModule* visionModule) :
  FeatureExtraction(visionModule), DebugBase("GoalExtraction", this)
{
  initDebugBase();
  int tempSendTime;
  int tempDrawScannedLines;
  int tempDrawScannedRegions;
  int tempDrawGoalBaseWindows;
  int tempDrawGoalPostBases;
  int tempDisplayInfo;
  int tempDisplayOutput;
  houghSettings.resize(3);
  GET_CONFIG(
    "VisionConfig",
    (int, GoalExtraction.sendTime, tempSendTime),
    (int, GoalExtraction.drawScannedLines, tempDrawScannedLines),
    (int, GoalExtraction.drawScannedRegions, tempDrawScannedRegions),
    (int, GoalExtraction.drawGoalBaseWindows, tempDrawGoalBaseWindows),
    (int, GoalExtraction.drawGoalPostBases, tempDrawGoalPostBases),
    (int, GoalExtraction.displayInfo, tempDisplayInfo),
    (int, GoalExtraction.displayOutput, tempDisplayOutput),
    (int, GoalExtraction.hltThreshold, houghSettings[0]),
    (int, GoalExtraction.hltLineLength, houghSettings[1]),
    (int, GoalExtraction.hltLineGap, houghSettings[2]),
  )
  SET_DVAR(int, sendTime, tempSendTime);
  SET_DVAR(int, drawScannedLines, tempDrawScannedLines);
  SET_DVAR(int, drawScannedRegions, tempDrawScannedRegions);
  SET_DVAR(int, drawGoalBaseWindows, tempDrawGoalBaseWindows);
  SET_DVAR(int, drawGoalPostBases, tempDrawGoalPostBases);
  SET_DVAR(int, displayInfo, tempDisplayInfo);
  SET_DVAR(int, displayOutput, tempDisplayOutput);
  fieldExt = GET_FEATURE_EXT_CLASS(FieldExtraction, FeatureExtractionIds::field);
  robotExt = GET_FEATURE_EXT_CLASS(RobotExtraction, FeatureExtractionIds::robot);

  //! Initializing processing times
  processTime= 0.0;
  scanTime= 0.0;
  classifyPostsTime= 0.0;
  findBestPostsTime= 0.0;
  updateGoalInfoTime= 0.0;
}

void GoalExtraction::processImage()
{
  auto tStart = high_resolution_clock::now();
  if (fieldExt->isFound()) {
    refreshGoalPosts();
    vector<ScannedLinePtr> verGoalLines;
    if (scanForPosts(verGoalLines)) {
      vector<ScannedRegionPtr> verGoalRegions;
      findRegions(verGoalRegions, verGoalLines, 8, 8, false);
      classifyPosts(verGoalRegions);
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
    LOG_INFO("Goal posts found: " << GOAL_INFO_OUT(VisionModule).found);
  }
  if (GET_DVAR(int, displayOutput))
    VisionUtils::displayImage("GoalExtraction", bgrMat[currentImage]);
}

void GoalExtraction::refreshGoalPosts()
{
  float currentTime = visionModule->getModuleTime();
  GPIter iter = goalPosts.begin();
  while (iter != goalPosts.end()) {
    if (currentTime - (*iter)->timeDetected < refreshTime) {
      ++iter;
    } else {
      iter = goalPosts.erase(iter);
    }
  }
}

bool GoalExtraction::scanForPosts(vector<ScannedLinePtr>& verGoalLines)
{
  auto tStart = high_resolution_clock::now();
  auto border = fieldExt->getBorder();
  auto robotRegions = robotExt->getRobotRegions();
  if (border.empty()) {
    duration<double> timeSpan = high_resolution_clock::now() - tStart;
    scanTime = timeSpan.count();
    return false;
  }
  srand(time(0));
  int scanStepHigh = 4, scanStepLow = 8;
  int yWhite = 150;
  int scanStart = rand() % scanStepHigh;
  for (int x = scanStart; x < getImageWidth(); x = x + scanStepHigh) {
    bool scan = true;
    for (int j = 0; j < robotRegions.size(); ++j) {
      if (!robotRegions[j]) continue;
      int min = robotRegions[j]->sr->rect.x;
      int max = robotRegions[j]->sr->rect.x + robotRegions[j]->sr->rect.width;
      if (x > min && x < max) {
        scan = false;
        break;
      }
    }
    if (!scan) continue;
    int borderY = border[x].y - 10;
    if (borderY < 0) continue;
    int colorY = getY(x, borderY);
    if (colorY < yWhite) continue;
    int end = -1;
    int down = 0;
    for (int j = 1; j <= 10; ++j) {
      int yDown = borderY + j;
      if (yDown > getImageHeight()) {
        end = getImageHeight();
        if (down > 5) {
          auto sl =
            boost::make_shared <ScannedLine> (Point(x, borderY), Point(x, end));
          verGoalLines.push_back(sl);
        }
        break;
      }
      if (yDown > 0) {
        colorY = getY(x, yDown);
        if (colorY > yWhite && j != 10) {
          down++;
        } else {
          end = yDown;
        }
      }
    }
    if (down > 5) {
      while (true) {
        int newEnd = end + scanStepLow;
        if (getY(x, newEnd) < yWhite || newEnd > getImageHeight()) {
          newEnd -= scanStepLow;
          break;
        }
        end = newEnd;
      }
      auto sl = boost::make_shared<ScannedLine>(Point(x, borderY), Point(x, end));
      verGoalLines.push_back(sl);
      if (GET_DVAR(int, drawScannedLines))
        line(bgrMat[currentImage], sl->p1, sl->p2, Scalar(255, 0, 255), 1);
    }
  }
  duration<double> timeSpan = high_resolution_clock::now() - tStart;
  scanTime = timeSpan.count();
  return true;
}

void GoalExtraction::classifyPosts(vector<ScannedRegionPtr>& verGoalRegions)
{
  auto tStart = high_resolution_clock::now();
  vector<GoalPostPtr> newPosts;
  for (int vgr = 0; vgr < verGoalRegions.size(); ++vgr) {
    //cout << "vgr: " << vgr << endl;
    auto gr = verGoalRegions[vgr];
    Rect r = gr->rect;
    if (r.width < 5 || r.area() < 50) continue;
    r = r - Point(r.width * 2.5 / 2, 0);
    r += Size(r.width * 2.5, 10);
    r = r & Rect(0, 0, getImageWidth(), getImageHeight());
    if (GET_DVAR(int, drawScannedRegions)) {
      rectangle(bgrMat[currentImage], r, Scalar(0,255,0), 1);
    }
    Mat cropped = getGrayImage()(r);
    float resizeRatio = 1.f;
    if (cropped.cols > 40) {
      resizeRatio = 40.f / (float) cropped.cols;
      resize(cropped, cropped, cv::Size(), resizeRatio, resizeRatio);
    }
    //VisionUtils::displayImage(cropped, "cropped");
    cropped.convertTo(cropped, -1, 1.5, 2);
    //VisionUtils::displayImage(cropped, "cropped2");
    Mat white;
    threshold(cropped, white, 225, 255, 0);
    Mat gradX;
    Mat absGradX;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    Sobel(white, gradX, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(gradX, absGradX);
    //dilate(absGradX, absGradX, Mat(), Point(-1, -1), 1, 1, 1);
    //VisionUtils::displayImage(white, "white");
    //VisionUtils::displayImage(absGradX, "absGradX");
    vector<int> gradHist;
    gradHist.resize(absGradX.cols);
    int gradRowCheck = 0;
    int lastRowCheck = 0;
    int maxCheck = 0.1 * absGradX.rows;
    int height = -1;
    uchar* p = absGradX.data;
    for (unsigned i = 0; i < absGradX.cols * absGradX.rows; ++i) {
      if (*p++ > 100) {
        gradHist[i % absGradX.cols]++;
        if (height == -1 && gradRowCheck < maxCheck) gradRowCheck++;
      }
      if (height == -1 && i % absGradX.cols == 0) {
        if (gradRowCheck < maxCheck) lastRowCheck++;
        if (lastRowCheck >= 5) {
          height = i / absGradX.cols - lastRowCheck;
        }
        gradRowCheck = 0;
      }
    }

    if (height == -1) {
      height = absGradX.rows;
    }
    height /= resizeRatio;
    height += r.y;

    vector<int> filt(gradHist.size());
    for (int i = 1; i < gradHist.size() - 1; ++i) {
      filt[i - 1] = (gradHist[i - 1] + gradHist[i] + gradHist[i + 1]) / 3;
    }

    gradHist = filt;
    int maxima = 0;
    for (unsigned i = 0; i < gradHist.size(); ++i) {
      maxima = gradHist[i] > maxima ? gradHist[i] : maxima;
    }

    const int threshold = maxima * 0.75;
    const int minThreshold = maxima * 0.25;
    int grad = -1;
    vector < pair<int, int> > peaks;
    for (int i = 0; i < gradHist.size() - 1; i++) {
      if (gradHist[i] - gradHist[i + 1] > 0) {
        if (grad == 1 && gradHist[i] > threshold) {
          peaks.push_back(make_pair(i, 1));
        }
        grad = -1;
      } else if (gradHist[i + 1] - gradHist[i] > 0) {
        if (grad == -1 && gradHist[i] < minThreshold) {
          peaks.push_back(make_pair(i, 0));
        }
        grad = 1;
      }
    }

    //for (int i = 0; i < peaks.size(); ++i)
    //  cout << "peaks[" << peaks[i].first << "]:" << peaks[i].second << endl;

    if (peaks.empty()) continue;
    int unchangedStart = peaks[0].first;
    int last = -1;
    vector < pair<int, int> > peaksFilt;
    for (int i = 0; i < peaks.size(); i++) {
      if (peaks[i].second != last) {
        if (last != -1) {
          float peak = (peaks[i - 1].first + peaks[unchangedStart].first) / 2;
          peaksFilt.push_back(make_pair(peak, last));
        }
        unchangedStart = i;
      }
      last = peaks[i].second;
      if (i == peaks.size() - 1) {
        float peak = (peaks[i].first + peaks[unchangedStart].first) / 2;
        peaksFilt.push_back(make_pair(peak, last));
      }
    }

    //for (int i = 0; i < peaksFilt.size(); ++i)
    //  cout << "peaksFilt[" << peaksFilt[i].first << "]:" << peaksFilt[i].second << endl;
    //waitKey(0);
    if (peaksFilt.empty()) continue;
    int imageCenter = absGradX.cols / 2;
    last = peaksFilt[0].second;
    int peak1 = -1;
    int peak2 = -1;
    for (int i = 1; i < peaksFilt.size(); i++) {
      if (peaksFilt[i].second == 0) {
        if (last == 1) {
          if (peaksFilt[i - 1].first < imageCenter) {
            peak1 = peaksFilt[i - 1].first;
          }
        }
      } else {
        if (last == 0) {
          if (peak1 != -1) {
            if (peak2 == -1 && peaksFilt[i].first > imageCenter) peak2 =
              peaksFilt[i].first;
          } else {
            peak1 = peaksFilt[i].first;
          }
        }
      }
      last = peaksFilt[i].second;
    }

    if (peak1 == -1 || peak2 == -1) continue;

    peak1 /= resizeRatio;
    peak2 /= resizeRatio;

    int diff = peak2 - peak1;
    //cout << "diff : " << diff << endl;
    if (diff < 5) continue;
    r.x = r.x + peak1;
    r.y = gr->rect.y;
    r.width = diff;
    r.height = gr->rect.height;
    int winX = r.x;
    int winSizeX = r.width;
    int winSizeY = 10;
    int winY = r.y + r.height;
    Rect window = Rect(winX, winY, winSizeX, winSizeY);
    window = window & Rect(0, 0, getImageWidth(), getImageHeight());
    if (GET_DVAR(int, drawGoalBaseWindows)) {
      rectangle(bgrMat[currentImage], window, Scalar(255, 255, 255), 1);
    }
    int whiteCnt = 0;
    int greenCnt = 0;
    int totalCnt = winSizeX * winSizeY;
    for (int j = winY; j < winSizeY + winY; ++j) {
      for (int k = winX; k < winSizeX + winX; ++k) {
        auto p = getYUV(k, j);
        if (colorHandler->isColor(p, TNColors::WHITE)) {
          whiteCnt++;
        } else if (colorHandler->isColor(p, TNColors::GREEN)) {
          bgrMat[currentImage].at<Vec3b>(j, k) = Vec3b(0,255,0);
          greenCnt++;
        }
      }
    }
    if ((float) greenCnt / (float) totalCnt > 0.1) {
      vector<Point2f> goalBases, goalBasesWorld;
      goalBases.push_back(Point2f(r.x, winY));
      goalBases.push_back(Point2f(r.x + r.width, winY));
      VisionUtils::drawPoint(Point(r.x, winY), bgrMat[currentImage], Scalar(255,255,0));
      VisionUtils::drawPoint(Point(r.x + r.width, winY), bgrMat[currentImage], Scalar(255,255,0));
      cameraTransforms[currentImage]->imageToWorld(
        goalBasesWorld,
        goalBases,
        0.0);
      float width = norm(goalBasesWorld[0] - goalBasesWorld[1]);
     //cout << "width: " << width << endl;
      if (width > 0.075 && width < 0.20) {
        auto cW = Point2f((goalBasesWorld[0].x + goalBasesWorld[1].x) / 2, // center world
        (goalBasesWorld[0].y + goalBasesWorld[1].y) / 2);
        auto cI = Point2f(r.x + r.width / 2, winY);
        auto gp = boost::make_shared < GoalPost > (cW, cI);
        bool exists = false;
        for (int i = 0; i < goalPosts.size(); ++i) {
          if (goalPosts[i]->checkDuplicate(gp)) {
            exists = true;
            break;
          }
        }
        if (!exists) newPosts.push_back(gp);
      }
    }
    //if (GET_DVAR(int, displayOutput))
    //VisionUtils::displayImage(bgrMat[currentImage], "GoalExtraction");
    //waitKey(0);
    if (vgr >= 2) break;
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
      for (int i = 0; i < goalPosts.size(); ++i) {
        if (!goalPosts[i]) continue;
        auto gp1 = goalPosts[i]->image;
        for (int j = 0; j < goalPosts.size(); ++j) {
          if (!goalPosts[j]) continue;
          if (i != j) {
            auto gp2 = goalPosts[j]->image;
            if (norm(gp1 - gp2) < 75) {
              if (gp1.y > gp2.y) {
                goalPosts[j].reset();
              }
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
        float bestDist = 0;
        for (int i = 0; i < goalPosts.size(); ++i) {
          if (!goalPosts[i]) continue;
          for (int j = i; j < goalPosts.size(); ++j) {
            if (!goalPosts[j]) continue;
            if (i != j) {
              float dist = norm(goalPosts[i]->world - goalPosts[j]->world);
              if (dist > 1.35 && dist < 1.85) {
                float d1 = fabsf(dist - 1.6);
                float d2 = fabsf(bestDist - 1.6);
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
  Point2f goalMid, rightPost, leftPost;
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
        for (int i = 0; i < goalPosts.size(); ++i) {
          if (goalPosts[i]->world.y < 0) {
            if (goalPosts[i]->world.x > 3.5 && goalPosts[i]->world.x < 4.1) // Goal on Right - Left Post
            {
              leftPost = goalPosts[i]->world;
            } else if (goalPosts[i]->world.x > 1.9 && goalPosts[i]->world.x < 2.5) // Goal on Right - Right Post
            {
              rightPost = goalPosts[i]->world;
            }
          } else {
            if (goalPosts[i]->world.x > 3.5 && goalPosts[i]->world.x < 4.1) // Goal on Left - Right Post
            {
              rightPost = goalPosts[i]->world;
            } else if (goalPosts[i]->world.x > 1.9 && goalPosts[i]->world.x < 2.5) // Goal on Left - Left Post
            {
              leftPost = goalPosts[i]->world;
            }
          }
          //LOG_INFO("Goal post: " << goalPosts[i]->world << endl)
        }
      } else if (goalInfo.type == GoalPostType::opponentTeam) {
        for (int i = 0; i < goalPosts.size(); ++i) {
          if (goalPosts[i]->world.y > 0) {
            if (goalPosts[i]->world.x > 3.5 && goalPosts[i]->world.x < 4.1) // Goal on Left - Right Post
            {
              rightPost = goalPosts[i]->world;
            } else if (goalPosts[i]->world.x > 1.9 && goalPosts[i]->world.x < 2.5) // Goal on Left - Left Post
            {
              leftPost = goalPosts[i]->world;
            }
          } else {
            if (goalPosts[i]->world.x > 3.5 && goalPosts[i]->world.x < 4.1) // Goal on Left - Left Post
            {
              leftPost = goalPosts[i]->world;
            } else if (goalPosts[i]->world.x > 1.9 && goalPosts[i]->world.x < 2.5) // Goal on Left - Right Post
            {
              rightPost = goalPosts[i]->world;
            }
          }
        }
      }
      goalInfo.found = true;
    } else {
      goalInfo.found = false;
    }
    goalInfo.leftPost = leftPost;
    goalInfo.rightPost = rightPost;
    goalInfo.mid = goalMid;
  } else {
    if (goalPosts.size() == 2) {
      if (goalPosts[0]->image.x < goalPosts[1]->image.x) { // 0 is to the left in image
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
  for (int i = 0; i < goalPosts.size(); ++i) {
    if (goalPosts[i]->refresh) 
      goalPosts[i]->timeDetected = currentTime;
  }
  if (goalInfo.found) {
    goalInfo.resetId();
    /*cout << "goalInfo.id: " << goalInfo.id << endl;
    cout << "goalInfo.left: " << goalInfo.leftPost << endl;
    cout << "goalInfo.right: " << goalInfo.rightPost << endl;
    cout << "goalInfo.mid: " << goalInfo.mid << endl;
    cout << "goalInfo.poseFromGoal: " << goalInfo.poseFromGoal.mat << endl;*/
  }
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
  int minIndex = -1;
  for (int i = 0; i < robotRegions.size(); ++i) {
    if (!robotRegions[i]) continue;
    auto goalToOtherRobot = goalInfo.mid - robotRegions[i]->world;
    auto goalToPerp = abs(
      goalToOtherRobot.x * goalUnit.x + goalToOtherRobot.y * goalUnit.y);
    auto gR = cv::norm(goalToOtherRobot);
    auto dist = sqrt(gR * gR - goalToPerp * goalToPerp);
    // The perpendicular distance from robot to goal line
    //cout << "dist[" << i << "]: " << dist << endl;
    if (dist < closest) {
      closest = dist;
      distFromCenter = goalToPerp;
      minIndex = i;
    }
  }
  if (minIndex != -1) {
    //cout << "Robot found between goal posts" << endl;
    //cout << "distFromCenter: " << distFromCenter << endl;
    //cout << "dist:" << closest << endl;
    // If robot is within 80cm of the goal mid along the goal posts
    // in either direction and 65cm in front of it then mark it as goal keeper
    if (closest < 0.65 && distFromCenter < 0.8) {
      if (robotRegions[minIndex]->ourTeam) {
        goalInfo.type = GoalPostType::ourTeam;
      } else {
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
  obs.front =
    TNRSLine<float>(
      Point2f(goalPost->world.x, goalPost->world.y - 0.05),
      Point2f(goalPost->world.x, goalPost->world.y + 0.05)
    );
  obs.back =
    TNRSLine<float>(
      obs.front.p1 + obs.front.perp * obs.depth,
      obs.front.p2 + obs.front.perp * obs.depth
    );
  obs.frontT =
    TNRSLine<float>(
      robotPose2D.transform(obs.front.p1),
      robotPose2D.transform(obs.front.p2)
    );
  obs.backT =
    TNRSLine<float>(
      robotPose2D.transform(obs.back.p1),
      robotPose2D.transform(obs.back.p2)
    );
  OBSTACLES_OBS_OUT(VisionModule).data.push_back(obs);
}


void GoalExtraction::addGoalLandmark(const GoalPostPtr& goalPost)
{
  auto p = goalPost->world;
  auto borderLines = fieldExt->getBorderLinesWorld();
  auto minDist = 1000;
  int index = 0;
  // Find which line intersects with the current goal post
  for (int i = 0; i < borderLines.size(); ++i) {
    auto bl = borderLines[i];
    float dist = fabsf(bl->perp.x * p.x + bl->perp.y * p.y - bl->perpDist);
    if (dist < minDist) {
      minDist = dist;
      index = i;
    }
  }
  auto line = borderLines[index];
  float len = norm(p);
  Point2f robotToGoalU = Point2f(p.x / len, p.y / len);
  float cross = robotToGoalU.x * line->unit.y - robotToGoalU.y * line->unit.x;
  float dot = robotToGoalU.dot(line->unit);
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
    if (GOAL_INFO_OUT(VisionModule).found) {
      for (size_t i = 0; i < goalPosts.size(); ++i) {
        VisionUtils::drawPoint(goalPosts[i]->image, bgrMat[currentImage], Scalar(0,0,0));
      }
    }
  }
}
