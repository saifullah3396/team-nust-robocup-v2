/**
 * @file VisionModule/src/ColorHandler.cpp
 *
 * This file implements the class ColorHandler.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Nov 2017
 */

#include <boost/circular_buffer.hpp>
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/VisionUtils.h"
#include "VisionModule/include/ColorHandler.h"
#include "VisionModule/include/VisionModule.h"

ColorHandler::ColorHandler() :
  DebugBase("ColorHandler", this)
{
  initDebugBase();
  auto root = JsonUtils::readJson(ConfigManager::getConfigDirPath() + "ColorConfig.json");
  colorTables.resize(toUType(TNColors::count));
  for (int i = 0; i < toUType(TNColors::count); ++i) {
    colorTables[i] = new ColorTable(root[colorNames[i]]["tables"].asInt());
    for (int j = 0; j < colorTables[i]->nTables; ++j) {
      JsonUtils::jsonToType(colorTables[i]->lower[j].yuv, root[colorNames[i]]["lower"][j], colorTables[i]->lower[j].yuv);
      JsonUtils::jsonToType(colorTables[i]->upper[j].yuv, root[colorNames[i]]["upper"][j], colorTables[i]->upper[j].yuv);
      for (size_t k = 0; k < 255; ++k) {
        colorTables[i]->lutY[j][k] = k >= colorTables[i]->lower[j].y() && k <= colorTables[i]->upper[j].y();
        colorTables[i]->lutU[j][k] = k >= colorTables[i]->lower[j].u() && k <= colorTables[i]->upper[j].u();
        colorTables[i]->lutV[j][k] = k >= colorTables[i]->lower[j].v() && k <= colorTables[i]->upper[j].v();
      }
    }
  }
  uHistBuff.set_capacity(5);
  vHistBuff.set_capacity(5);
  pixelsBuff.set_capacity(5);
  greenHistU = new uint8_t[256];
  greenHistV = new uint8_t[256];
  fieldHist = false;
}

void ColorHandler::update()
{
  auto& ci = GET_DVAR(int, colorIndex);
  auto& ti = GET_DVAR(int, tableIndex);
  if (ci >= 0 && ci < toUType(TNColors::count) &&
      ti >= 0 && ti < colorTables[ci]->nTables)
  {
    auto& lower = GET_DVAR(vector<int>, lower);
    auto& upper = GET_DVAR(vector<int>, upper);
    if (!lower.empty()) {
      for (int i = 0; i < 3; ++i)
        colorTables[ci]->lower[ti].yuv[i] = lower[i];
    }
    if (!upper.empty()) {
      for (int i = 0; i < 3; ++i)
        colorTables[ci]->upper[ti].yuv[i] = upper[i];
    }
    cout << "ci index:" << ci << endl;
    cout << "ti index:" << ti << endl;
    cout << "lower: " << colorTables[ci]->lower[ti].yuv << endl;
    cout << "upper: " << colorTables[ci]->upper[ti].yuv << endl;
    SET_DVAR(int, colorIndex, -1);
    SET_DVAR(int, tableIndex, -1);
  }
}

void ColorHandler::getBinary(const Mat& in, Mat& out, const TNColors& colorIndex) const
{
  for (int i = 0; i < colorTables[toUType(colorIndex)]->nTables; ++i) {
    Mat binary;
    inRange(
      in,
      Scalar(
        colorTables[toUType(colorIndex)]->lower[i].y(),
        colorTables[toUType(colorIndex)]->lower[i].u(),
        colorTables[toUType(colorIndex)]->lower[i].v()),
      Scalar(
        colorTables[toUType(colorIndex)]->upper[i].y(),
        colorTables[toUType(colorIndex)]->upper[i].u(),
        colorTables[toUType(colorIndex)]->upper[i].v()),
      binary);
    if (out.empty()) out = binary;
    else bitwise_or(out, binary, out);
  }
}

 void ColorHandler::computeUVHist(const Mat& uv, const Mat& mask, const bool& drawHists)
{
  Mat uvCh[2];
  split(uv, uvCh);
  Mat uHist, vHist;
  const float histRange[] =
    { 0, 256 };
  const float* range =
    { histRange };
  calcHist(
    &uvCh[0], 1, 0, mask, uHist, 1, &histBins, &range, uniformHist, accumulateHist);
  calcHist(
    &uvCh[1], 1, 0, mask, vHist, 1, &histBins, &range, uniformHist, accumulateHist);
  uHistBuff.push_back(uHist);
  vHistBuff.push_back(vHist);
  pixelsBuff.push_back(countNonZero(mask));
  //! Draw histograms for debugging
  if (uHistBuff.size() >= 5) {
    Mat avgHistU = uHistBuff[0];
    Mat avgHistV = vHistBuff[0];
    int pixels = pixelsBuff[0];
    for (int i = 1; i < 5; ++i) {
      addWeighted(uHistBuff[i], 1.0, avgHistU, 1.0, 0, avgHistU);
      addWeighted(vHistBuff[i], 1.0, avgHistV, 1.0, 0, avgHistV);
      pixels += pixelsBuff[i];
    }
    avgHistU = avgHistU / pixels;
    avgHistV = avgHistV / pixels;
    //! Remove values below 35% of maximum value.
    double uMin, uMax, vMin, vMax;
    minMaxLoc(avgHistU, &uMin, &uMax);
    minMaxLoc(avgHistV, &vMin, &vMax);
    for (int i = 0; i < 256; ++i) {
      greenHistU[i] = avgHistU.at<float>(i / 8) < 0.3 * uMax ? 0 : 1.0;
      greenHistV[i] = avgHistV.at<float>(i / 8) < 0.3 * vMax ? 0 : 1.0;
    }
    fieldHist = true;
  } else {
    fieldHist = false;
  }
}

bool ColorHandler::isGreenHist(const TNColor& color) const
{
  if ((int) greenHistU[color.getU()] && (int) greenHistV[color.getV()]) return true;
  return false;
}

bool ColorHandler::isColor(const TNColor& color, const TNColors& colorIndex) const
{
  for (int i = 0; i < colorTables[toUType(colorIndex)]->nTables; ++i) {
    if (
      (int) colorTables[toUType(colorIndex)]->lutY[i][(int)color.getY()] &&
      (int) colorTables[toUType(colorIndex)]->lutU[i][(int)color.getU()] &&
      (int) colorTables[toUType(colorIndex)]->lutV[i][(int)color.getV()])
    {
      return true;
    }
  }
  return false;
}

bool ColorHandler::fieldHistFormed() const
{
  return fieldHist;
}


typedef boost::shared_ptr<ColorHandler> ColorHandlerPtr;
