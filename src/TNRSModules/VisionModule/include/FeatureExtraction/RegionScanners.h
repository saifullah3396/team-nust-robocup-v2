/**
 * @file VisionModule/include/FeatureExtraction/RegionScanners.h
 *
 * This file declares the line region scanners used by RegionSegmentation class
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author AbdulRehman
 * @date 5 Mar 2018
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

using namespace std;

struct LinearScannedLine;
enum class TNColors : unsigned int;

struct Scan {
  Scan(
    const TNColors& color,
    const int& lowStep,
    const int& highStep,
    const bool& direction);
  virtual void onScanLimitReached(const int& limitIndex, const int& highIndex) = 0;
  virtual void update(const TNColors& color, const int& lowIndex, const int& highIndex) = 0;
  virtual void draw(cv::Mat& image);
  virtual void reset();

  int lowStep = {8};
  int highStep = {16};
  int trueColor = {0};
  int falseColor = {0};
  int trueColorThreshold = {2};
  int falseColorThreshold = {2};
  bool direction = {false}; ///< False for horizontal, True for vertical
  vector<boost::shared_ptr<LinearScannedLine> > scanLines;
  TNColors color;
  vector<vector<bool> > scanTables;
  bool enabled = {false};
};

struct FieldScan : Scan {
  FieldScan(
    const int& lowStep,
    const int& highStep,
    const bool& direction);

  void lineFound(const int& lowIndex, const int& highIndex);
  void onScanLimitReached(const int& limitIndex, const int& highIndex) final;
  void update(const TNColors& color, const int& lowIndex, const int& highIndex) final;
  void draw(cv::Mat& image) final;
  void reset();

  int highIndex = {0};
  int fieldMin = {1000};
  int fieldMax = {0};
  int previousLen = {0};
  int lenComparisonThreshold = {20};
  bool fieldFound = {false};
};

struct RobotScan : Scan {
  RobotScan(
    const int& lowStep,
    const int& highStep,
    const bool& direction);

  void lineFound(const int& lowIndex, const int& highIndex);
  void onScanLimitReached(const int& limitIndex, const int& highIndex) final;
  void update(const TNColors& color, const int& lowIndex, const int& highIndex) final;
  void draw(cv::Mat& image) final;
  void reset();

  int otherColor = {0};
  int otherColorThreshold = {0};
};

struct BallScan : Scan {
  BallScan(
    const int& lowStep,
    const int& highStep,
    const bool& direction);

  void lineFound(const int& lowIndex, const int& highIndex);
  void onScanLimitReached(const int& limitIndex, const int& highIndex) final;
  void update(const TNColors& color, const int& lowIndex, const int& highIndex) final;
  void draw(cv::Mat& image) final;
  void reset();

  int otherColor = {0};
  int otherColorThreshold = {0};
};

struct JerseyScan : Scan {
  JerseyScan(
    const TNColors& color,
    const int& lowStep,
    const int& highStep,
    const bool& direction);

  void lineFound(const int& lowIndex, const int& highIndex);
  void onScanLimitReached(const int& limitIndex, const int& highIndex) final;
  void update(const TNColors& color, const int& lowIndex, const int& highIndex) final;
  void draw(cv::Mat& image) final;
  void reset();
};

struct LinesScan : Scan {
  LinesScan(
    const int& lowStep,
    const int& highStep,
    const bool& direction);

  void lineFound(const int& lowIndex, const int& highIndex);
  void onScanLimitReached(const int& limitIndex, const int& highIndex) final;
  void update(const TNColors& color, const int& lowIndex, const int& highIndex) final;
  void draw(cv::Mat& image) final;
  void reset();

  vector<cv::Point2f> edgePoints;
};

struct GoalScan : Scan {
  GoalScan(
    const int& lowStep,
    const int& highStep,
    const bool& direction);

  void lineFound(const int& lowIndex, const int& highIndex);
  void onScanLimitReached(const int& limitIndex, const int& highIndex) final;
  void update(const TNColors& color, const int& lowIndex, const int& highIndex) final;
  void draw(cv::Mat& image) final;
  void reset();

  int topRangeThreshold = {36};
};
