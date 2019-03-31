/**
 * @file ColorHandler/ColorHandler.h
 *
 * This file declares the class ColorHandler.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Nov 2017
 */

#pragma once

#include <boost/circular_buffer.hpp>
#include "TNRSBase/include/DebugBase.h"
#include "Utils/include/TNColors.h"

/**
 * @struct ColorTable
 * @brief Defines a color lookup table
 */
struct ColorTable
{
  ColorTable(const ColorTable&) = default;
  ColorTable(ColorTable&&) = default;
  ColorTable& operator=(const ColorTable&) & = default;
  ColorTable& operator=(ColorTable&&) & = default;
  virtual ~ColorTable() {}

  ColorTable(const unsigned& nTables) {
    this->nTables = nTables;
    lower.resize(nTables);
    upper.resize(nTables);
    lutY.resize(nTables);
    lutU.resize(nTables);
    lutV.resize(nTables);
  }

  vector<Matrix<unsigned, 1, 255> > lutY;
  vector<Matrix<unsigned, 1, 255> > lutU;
  vector<Matrix<unsigned, 1, 255> > lutV;
  vector<TNColor> lower;
  vector<TNColor> upper;
  unsigned nTables;
};

/**
 * @class ColorHandler
 * @brief All kinds of processing related to colors is handled by this class
 */
class ColorHandler : public DebugBase
{
  INIT_DEBUG_BASE_(
    //! Color to update
    (int, colorIndex, -1),
    (int, tableIndex, -1),
    (vector<int>, lower, vector<int>()),
    (vector<int>, upper, vector<int>()),
  )
public:
  /**
   * @brief ColorHandler Constructor
   */
  ColorHandler();

  /**
   * @brief ~ColorHandler Destructor
   */
  ~ColorHandler() {}

  void update();
  void getBinary(const Mat& in, Mat& out, const TNColors& colorIndex) const;
  void computeUVHist(const Mat& uv, const Mat& mask, const bool& drawHists);
  bool isGreenHist(const TNColor& color) const;
  bool isColor(const TNColor& color, const TNColors& colorIndex) const;
  bool fieldHistFormed() const;

private:
  void createColorLut(uint8_t*& tableY, uint8_t*& tableU, uint8_t*& tableV, const TNColor& min, const TNColor& max);
  vector<ColorTable*> colorTables;

  //! Histogram bins
  int histBins = {32};

  //! Opencv calcHist settings
  bool uniformHist = {true};
  bool accumulateHist = {false};
  bool fieldHist = {false};

  //! Accumulative Histogram buffers
  boost::circular_buffer<Mat> uHistBuff;
  boost::circular_buffer<Mat> vHistBuff;
  boost::circular_buffer<int> pixelsBuff;
  uint8_t* greenHistU;
  uint8_t* greenHistV;
};

typedef boost::shared_ptr<ColorHandler> ColorHandlerPtr;
