/**
 * @file VisionModule/include/FeatureExtraction/RegionScanners.h
 *
 * This file defines the line region scanners used by RegionSegmentation class
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author AbdulRehman
 * @date 5 Mar 2018
 */

struct Scan {
  Scan(
    const TNColors& color,
    const unsigned& lowStep,
    const unsigned& highStep,
    const bool& direction) :
    color(color), lowStep(lowStep), highStep(highStep), direction(direction)
  {}
  virtual void onScanLimitReached(const unsigned& limitIndex, const unsigned& highIndex) = 0;
  virtual void update(const TNColors& color, const unsigned& lowIndex, const unsigned& highIndex) = 0;
  virtual void draw(Mat& image) {}
  virtual void reset() {
    trueColor = 0;
    falseColor = 0;
  }

  unsigned lowStep = {8};
  unsigned highStep = {16};
  unsigned trueColor = {0};
  unsigned falseColor = {0};
  unsigned trueColorThreshold = {2};
  unsigned falseColorThreshold = {2};
  bool direction = false; //! False for horizontal, True for vertical
  TNColors color;
};

struct FieldScan : Scan {
  FieldScan(
    const TNColors& color,
    const unsigned& lowStep,
    const unsigned& highStep,
    const bool& direction) :
    Scan(color, lowStep, highStep, direction)
  {}

  void lineFound(const unsigned& lowIndex, const unsigned& highIndex) {
    if (trueColor > trueColorThreshold) {
      auto len = trueColor * lowStep;
      auto start = lowIndex - len;
      start = start < lowStep ? 0 : lowStep;
      fieldMin = start < fieldMin ? start : fieldMin;
      fieldMax = lowIndex > fieldMax ? lowIndex : fieldMax;
      fieldFound = true;
    }
  }

  void onScanLimitReached(const unsigned& limitIndex, const unsigned& highIndex) final {
    lineFound(limitIndex, highIndex);
  }

  void update(const TNColors& color, const unsigned& lowIndex, const unsigned& highIndex) final {
    this->highIndex = highIndex;
    if (color == this->color) {
      trueColor++;
    } else {
      lineFound(lowIndex, highIndex);
      trueColor = 0;
    }
  }

  void draw(Mat& image) final {
    if (fieldFound) {
      if (!direction) {
        line(
          image,
          Point(fieldMin, highIndex),
          Point(fieldMax, highIndex),
          colorToBgr[toUType(color)], 1
        );
      } else {
        line(
          image,
          Point(highIndex, fieldMin),
          Point(highIndex, fieldMax),
          colorToBgr[toUType(color)], 1
        );
      }
    }
  }

  void reset() {
    trueColor = 0;
    falseColor = 0;
    fieldMin = 1000; // high value
    fieldMax = 0; // low value
  }

  unsigned highIndex = {0};
  unsigned fieldMin = {1000};
  unsigned fieldMax = {0};
  bool fieldFound = {false};
};

struct RobotScan : Scan {
  RobotScan(
    const TNColors& color,
    const unsigned& lowStep,
    const unsigned& highStep,
    const bool& direction) :
    Scan(color, lowStep, highStep, direction)
  {
    falseColorThreshold = 1;
    trueColorThreshold = 0;
  }

  void lineFound(const unsigned& lowIndex, const unsigned& highIndex) {
    if (trueColor > trueColorThreshold &&
        otherColor > otherColorThreshold)
    {
      auto len = (trueColor + otherColor) * lowStep;
      auto end = lowIndex - falseColor * lowStep;
      auto start = end - len;
      auto sl =
        boost::make_shared<LinearScannedLine> (start, end, highIndex, direction);
      scannedLines.push_back(sl);
    }
  }

  void onScanLimitReached(const unsigned& limitIndex, const unsigned& highIndex) final {
    if (falseColor > falseColorThreshold)
    {
      lineFound(limitIndex, highIndex);
      otherColor = 0;
      trueColor = 0;
      falseColor = 0;
    }
  }

  void update(const TNColors& color, const unsigned& lowIndex, const unsigned& highIndex) final {
    if (color == this->color) {
      if (falseColor <= falseColorThreshold)
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
      }
      falseColor++;
    }
  }

  void draw(Mat& image) final {
    for (const auto& sl : scannedLines) {
      if (sl) {
        if (!direction) {
          line(
            image,
            Point(sl->start, sl->baseIndex),
            Point(sl->end, sl->baseIndex),
            colorToBgr[toUType(color)], 1
          );
        } else {
          line(
            image,
            Point(sl->baseIndex, sl->start),
            Point(sl->baseIndex, sl->end),
            colorToBgr[toUType(color)], 1
          );
        }
      }
    }
  }

  void reset() {
    trueColor = 0;
    falseColor = 0;
    otherColor = 0;
  }

  unsigned otherColor = {0};
  unsigned otherColorThreshold = {0};
  vector<boost::shared_ptr<LinearScannedLine>> scannedLines;
};

struct BallScan : Scan {
  BallScan(
    const TNColors& color,
    const unsigned& lowStep,
    const unsigned& highStep,
    const bool& direction) :
    Scan(color, lowStep, highStep, direction)
  {
    falseColorThreshold = 1;
    trueColorThreshold = 0;
  }

  void lineFound(const unsigned& lowIndex, const unsigned& highIndex) {
    if (trueColor > trueColorThreshold &&
        otherColor > otherColorThreshold)
    {
      auto len = (trueColor + otherColor) * lowStep;
      auto end = lowIndex - falseColor * lowStep;
      auto start = end - len;
      auto sl =
        boost::make_shared<LinearScannedLine> (start, end, highIndex, direction);
      scannedLines.push_back(sl);
    }
  }

  void onScanLimitReached(const unsigned& limitIndex, const unsigned& highIndex) final {
    if (falseColor > falseColorThreshold)
    {
      lineFound(limitIndex, highIndex);
      otherColor = 0;
      trueColor = 0;
      falseColor = 0;
    }
  }

  void update(const TNColors& color, const unsigned& lowIndex, const unsigned& highIndex) final {
    if (color == this->color) {
      if (falseColor <= falseColorThreshold)
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
      }
      falseColor++;
    }
  }

  void draw(Mat& image) final {
    for (const auto& sl : scannedLines) {
      if (sl) {
        if (!direction) {
          line(
            image,
            Point(sl->start, sl->baseIndex),
            Point(sl->end, sl->baseIndex),
            colorToBgr[toUType(color)], 1
          );
        } else {
          line(
            image,
            Point(sl->baseIndex, sl->start),
            Point(sl->baseIndex, sl->end),
            colorToBgr[toUType(color)], 1
          );
        }
      }
    }
  }

  void reset() {
    trueColor = 0;
    falseColor = 0;
  }

  unsigned otherColor = {0};
  unsigned otherColorThreshold = {0};
  vector<boost::shared_ptr<LinearScannedLine>> scannedLines;
};

struct JerseyScan : Scan {
  JerseyScan(
    const TNColors& color,
    const unsigned& lowStep,
    const unsigned& highStep,
    const bool& direction) :
    Scan(color, lowStep, highStep, direction)
  {
  }

  void lineFound(const unsigned& lowIndex, const unsigned& highIndex) {
    if (trueColor > trueColorThreshold) {
      auto len = trueColor * lowStep;
      auto start = lowIndex - len ;
      auto sl =
        boost::make_shared<LinearScannedLine> (start, lowIndex, highIndex, direction);
      scannedLines.push_back(sl);
    }
  }

  void onScanLimitReached(const unsigned& limitIndex, const unsigned& highIndex) final {
    lineFound(limitIndex, highIndex);
  }

  void update(const TNColors& color, const unsigned& lowIndex, const unsigned& highIndex) final {
    if (color == this->color) {
      trueColor++;
    } else {
      lineFound(lowIndex, highIndex);
      trueColor = 0;
    }
  }

  void draw(Mat& image) final {
    for (const auto& sl : scannedLines) {
      if (sl) {
        if (!direction) {
          line(
            image,
            Point(sl->start, sl->baseIndex),
            Point(sl->end, sl->baseIndex),
            colorToBgr[toUType(color)], 1
          );
        } else {
          line(
            image,
            Point(sl->baseIndex, sl->start),
            Point(sl->baseIndex, sl->end),
            colorToBgr[toUType(color)], 1
          );
        }
      }
    }
  }

  void reset() {
    trueColor = 0;
    falseColor = 0;
  }

  vector<boost::shared_ptr<LinearScannedLine>> scannedLines;
};
