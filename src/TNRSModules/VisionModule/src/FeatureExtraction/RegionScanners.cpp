/**
 * @file VisionModule/src/FeatureExtraction/RegionScanners.cpp
 *
 * This file implements the line region scanners used by RegionSegmentation class
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author AbdulRehman
 * @date 5 Mar 2018
 */

#include <iostream>
#include <boost/make_shared.hpp>
#include "Utils/include/TNColors.h"
#include "VisionModule/include/FeatureExtraction/RegionScanners.h"
#include "VisionModule/include/FeatureExtraction/ScannedLine.h"

Scan::Scan(
  const TNColors& color,
  const int& lowStep,
  const int& highStep,
  const bool& direction) :
  color(color), lowStep(lowStep), highStep(highStep), direction(direction)
{}
void Scan::draw(cv::Mat& image) {}
void Scan::reset() {
  trueColor = 0;
  falseColor = 0;
}

FieldScan::FieldScan(
  const int& lowStep,
  const int& highStep,
  const bool& direction) :
  Scan(TNColors::green, lowStep, highStep, direction)
{}

void FieldScan::lineFound(const int& lowIndex, const int& highIndex) {
  if (trueColor > trueColorThreshold) {
    auto len = trueColor * lowStep;
    auto end = lowIndex - falseColor * lowStep;
    auto start = end - len;
    start = start < lowStep ? 0 : start;
    // If a new section of green is at least threshold times larger than previous section
    // discard the previous one
    if (len > lenComparisonThreshold * previousLen) {
      fieldMin = start;
      fieldMax = end;
    } else {
      fieldMin = start < fieldMin ? start : fieldMin;
      fieldMax = end > fieldMax ? end : fieldMax;
    }
    /*cout << "lowStep:" << lowStep << endl;
    cout << "lowIndex:" << lowIndex << endl;
    cout << "falseColor:" << falseColor << endl;
    cout << "start:" << start << endl;
    cout << "end:" << end << endl;
    cout << "len:" << len << endl;
    cout << "previousLen:" << previousLen << endl;*/
    previousLen = len;
    fieldFound = true;
  }
}

void FieldScan::onScanLimitReached(const int& limitIndex, const int& highIndex) {
  lineFound(limitIndex, highIndex);
  falseColor = 0;
  trueColor = 0;
}

void FieldScan::update(const TNColors& color, const int& lowIndex, const int& highIndex) {
  this->highIndex = highIndex;
  if (color == this->color) {
    if (trueColor > trueColorThreshold && falseColor <= falseColorThreshold)
      trueColor += falseColor;
    trueColor++;
    falseColor = 0;
  } else {
    falseColor++;
    if (falseColor > falseColorThreshold)
    {
      lineFound(lowIndex, highIndex);
      trueColor = 0;
      falseColor = 0;
    }
  }
}

void FieldScan::draw(cv::Mat& image) {
 if (!direction) {
    line(
      image,
      cv::Point(fieldMin, highIndex),
      cv::Point(fieldMax, highIndex),
      colorToBgr[toUType(color)], 1
    );
  } else {
    line(
      image,
      cv::Point(highIndex, fieldMin),
      cv::Point(highIndex, fieldMax),
      colorToBgr[toUType(color)], 1
    );
  }
}

void FieldScan::reset() {
  trueColor = 0;
  falseColor = 0;
  fieldMin = 1000; // high value
  fieldMax = 0; // low value
  previousLen = 0;
  highIndex = 0;
  fieldFound = false;
}


RobotScan::RobotScan(
  const int& lowStep,
  const int& highStep,
  const bool& direction) :
  Scan(TNColors::white, lowStep, highStep, direction)
{
  falseColorThreshold = 0;
  trueColorThreshold = 0;
  otherColorThreshold = 0;
}

void RobotScan::lineFound(const int& lowIndex, const int& highIndex) {
  if (trueColor > trueColorThreshold &&
      otherColor > otherColorThreshold)
  {
    auto len = (trueColor + otherColor) * lowStep;
    auto end = lowIndex - falseColor * lowStep;
    auto start = end - len;
    auto sl =
      boost::make_shared<LinearScannedLine> (start, end, highIndex, len, direction);
    scanLines.push_back(sl);
  }
}

void RobotScan::onScanLimitReached(const int& limitIndex, const int& highIndex) {
  lineFound(limitIndex, highIndex);
  otherColor = 0;
  trueColor = 0;
  falseColor = 0;
}

void RobotScan::update(const TNColors& color, const int& lowIndex, const int& highIndex) {
  if (color == this->color) {
    //if (trueColor > trueColorThreshold && falseColor <= falseColorThreshold)
    //  trueColor += falseColor;
    trueColor++;
    falseColor = 0;
  } else if (color != TNColors::green) {
    //if (falseColor <= falseColorThreshold)
    //  otherColor += falseColor;
    otherColor++;
    falseColor = 0;
  } else {
    falseColor++;
    if (falseColor > falseColorThreshold)
    {
      lineFound(lowIndex, highIndex);
      otherColor = 0;
      trueColor = 0;
      falseColor = 0;
    }
  }
}

void RobotScan::draw(cv::Mat& image) {
  for (const auto& sl : scanLines) {
    if (sl) {
      if (!direction) {
        line(
          image,
          cv::Point(sl->start, sl->baseIndex),
          cv::Point(sl->end, sl->baseIndex),
          colorToBgr[toUType(color)], 1
        );
      } else {
        line(
          image,
          cv::Point(sl->baseIndex, sl->start),
          cv::Point(sl->baseIndex, sl->end),
          colorToBgr[toUType(color)], 1
        );
      }
    }
  }
}

void RobotScan::reset() {
  trueColor = 0;
  falseColor = 0;
  otherColor = 0;
}

BallScan::BallScan(
  const int& lowStep,
  const int& highStep,
  const bool& direction) :
  Scan(TNColors::black, lowStep, highStep, direction)
{
  falseColorThreshold = 0;
  trueColorThreshold = 0;
  otherColorThreshold = 0;
}

void BallScan::lineFound(const int& lowIndex, const int& highIndex) {
  if (trueColor > trueColorThreshold &&
      otherColor > otherColorThreshold)
  {
    auto len = (trueColor + otherColor) * lowStep;
    auto end = lowIndex - falseColor * lowStep;
    auto start = end - len;
    auto sl =
      boost::make_shared<LinearScannedLine> (start, end, highIndex, len, direction);
    scanLines.push_back(sl);
  }
}

void BallScan::onScanLimitReached(const int& limitIndex, const int& highIndex) {
  if (falseColor > falseColorThreshold)
  {
    lineFound(limitIndex, highIndex);
    otherColor = 0;
    trueColor = 0;
    falseColor = 0;
  }
}

void BallScan::update(const TNColors& color, const int& lowIndex, const int& highIndex) {
  /*if (color == this->color) {
    if (trueColor > trueColorThreshold && falseColor <= falseColorThreshold)
      trueColor += falseColor;
    trueColor++;
    falseColor = 0;
  } else if (color == TNColors::white) {
    if (falseColor <= falseColorThreshold)
      otherColor += falseColor;
    otherColor++;
    falseColor = 0;
  } else {
    if (falseColor > falseColorThreshold)
    {
      lineFound(lowIndex, highIndex);
      otherColor = 0;
      trueColor = 0;
      falseColor = 0;
    }
    falseColor++;
  }*/
  if (color == TNColors::white) {
      if (trueColor > trueColorThreshold && falseColor <= falseColorThreshold)
        trueColor += falseColor;
      trueColor++;
      falseColor = 0;
    } else if (color != TNColors::green) {
      if (falseColor <= falseColorThreshold)
        otherColor += falseColor;
      otherColor++;
      falseColor = 0;
    } else {
      if (falseColor > falseColorThreshold)
      {
        lineFound(lowIndex, highIndex);
        otherColor = 0;
        trueColor = 0;
        falseColor = 0;
      }
      falseColor++;
    }
}

void BallScan::draw(cv::Mat& image) {
  for (const auto& sl : scanLines) {
    if (sl) {
      if (!direction) {
        line(
          image,
          cv::Point(sl->start, sl->baseIndex),
          cv::Point(sl->end, sl->baseIndex),
          colorToBgr[toUType(color)], 1
        );
      } else {
        line(
          image,
          cv::Point(sl->baseIndex, sl->start),
          cv::Point(sl->baseIndex, sl->end),
          colorToBgr[toUType(color)], 1
        );
      }
    }
  }
}

void BallScan::reset() {
  trueColor = 0;
  falseColor = 0;
}

JerseyScan::JerseyScan(
  const TNColors& color,
  const int& lowStep,
  const int& highStep,
  const bool& direction) :
  Scan(color, lowStep, highStep, direction)
{
  trueColorThreshold = 0;
  falseColorThreshold = 0;
}

void JerseyScan::lineFound(const int& lowIndex, const int& highIndex) {
  if (trueColor > trueColorThreshold) {
    auto len = trueColor * lowStep;
    auto end = lowIndex - falseColor * lowStep;
    auto start = end - len;
    auto sl =
      boost::make_shared<LinearScannedLine> (start, end, highIndex, len, direction);
    scanLines.push_back(sl);
  }
}

void JerseyScan::onScanLimitReached(const int& limitIndex, const int& highIndex) {
  lineFound(limitIndex, highIndex);
  falseColor = 0;
  trueColor = 0;
}

void JerseyScan::update(const TNColors& color, const int& lowIndex, const int& highIndex) {
  if (color == this->color) {
    trueColor++;
    falseColor = 0;
  } else {
    falseColor++;
    if (falseColor > falseColorThreshold)
    {
      lineFound(lowIndex, highIndex);
      trueColor = 0;
      falseColor = 0;
    }
  }
}

void JerseyScan::draw(cv::Mat& image) {
  for (const auto& sl : scanLines) {
    if (sl) {
      if (!direction) {
        line(
          image,
          cv::Point(sl->start, sl->baseIndex),
          cv::Point(sl->end, sl->baseIndex),
          colorToBgr[toUType(color)], 1
        );
      } else {
        line(
          image,
          cv::Point(sl->baseIndex, sl->start),
          cv::Point(sl->baseIndex, sl->end),
          colorToBgr[toUType(color)], 1
        );
      }
    }
  }
}

void JerseyScan::reset() {
  trueColor = 0;
  falseColor = 0;
}

LinesScan::LinesScan(
  const int& lowStep,
  const int& highStep,
  const bool& direction) :
  Scan(TNColors::white, lowStep, highStep, direction)
{
  falseColorThreshold = 0;
  trueColorThreshold = 0;
}

void LinesScan::lineFound(const int& lowIndex, const int& highIndex) {
  if (trueColor > trueColorThreshold) {
    auto len = trueColor * lowStep;
    auto end = lowIndex - falseColor * lowStep;
    auto start = end - len;
    if (!direction)
      edgePoints.push_back(cv::Point2f((start + end) / 2, highIndex));
    else
      edgePoints.push_back(cv::Point2f(highIndex, (start + end) / 2));
  }
}

void LinesScan::onScanLimitReached(const int& limitIndex, const int& highIndex) {
  lineFound(limitIndex, highIndex);
  trueColor = 0;
  falseColor = 0;
}

void LinesScan::update(const TNColors& color, const int& lowIndex, const int& highIndex) {
  if (color == this->color) {
    trueColor++;
    falseColor = 0;
  } else if (color == TNColors::green) {
    falseColor++;
    if (falseColor > falseColorThreshold)
    {
      lineFound(lowIndex, highIndex);
      trueColor = 0;
      falseColor = 0;
    }
  }
}

void LinesScan::draw(cv::Mat& image) {
  VisionUtils::drawPoints(edgePoints, image, cv::Scalar(0, 0, 255));
}

void LinesScan::reset() {
  trueColor = 0;
  falseColor = 0;
}

GoalScan::GoalScan(
  const int& lowStep,
  const int& highStep,
  const bool& direction) :
  Scan(TNColors::white, lowStep, highStep, direction)
{
  falseColorThreshold = 0;
  trueColorThreshold = 0;
}

void GoalScan::lineFound(const int& lowIndex, const int& highIndex) {
  if (trueColor > trueColorThreshold) {
    auto len = (trueColor - 1) * lowStep;
    if (len <= 100) {
      auto end = lowIndex - falseColor * lowStep;
      auto start = end - len;
      auto sl =
        boost::make_shared<LinearScannedLine> (start, end, highIndex, len, direction);
      scanLines.push_back(sl);
    }
  }
}

void GoalScan::onScanLimitReached(const int& limitIndex, const int& highIndex) {
  lineFound(limitIndex, highIndex);
  trueColor = 0;
  falseColor = 0;
}

void GoalScan::update(const TNColors& color, const int& lowIndex, const int& highIndex) {
  if (color == this->color) {
    trueColor++;
    falseColor = 0;
  } else {
    falseColor++;
    if (falseColor > falseColorThreshold)
    {
      lineFound(lowIndex, highIndex);
      trueColor = 0;
      falseColor = 0;
    }
  }
}

void GoalScan::draw(cv::Mat& image) {
  for (const auto& sl : scanLines) {
    if (sl) {
      if (!direction) {
        line(
          image,
          cv::Point(sl->start, sl->baseIndex),
          cv::Point(sl->end, sl->baseIndex),
          cv::Scalar(255, 0, 255), 1
        );
      } else {
        line(
          image,
          cv::Point(sl->baseIndex, sl->start),
          cv::Point(sl->baseIndex, sl->end),
          cv::Scalar(255, 0, 255), 1
        );
      }
    }
  }
}

void GoalScan::reset() {
  trueColor = 0;
  falseColor = 0;
}
