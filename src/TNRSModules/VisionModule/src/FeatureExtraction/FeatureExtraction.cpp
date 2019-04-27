/**
 * @file VisionModule/src/FeatureExtraction/FeatureExtraction.cpp
 *
 * This file implements the base class for feature extraction
 * from the image. All the functions and algorithms for detecting
 * features from the image will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017
 */

#include <boost/make_shared.hpp>
#include <opencv2/photo/photo.hpp>
#include "Utils/include/DataHolders/Camera.h"
#include "Utils/include/TNColors.h"
#include "VisionModule/include/CameraModule.h"
#include "VisionModule/include/CameraTransform.h"
#include "VisionModule/include/ColorHandler.h"
#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"
#include "VisionModule/include/FeatureExtraction/ScannedLine.h"
#include "VisionModule/include/FeatureExtraction/ScannedRegion.h"
#include "VisionModule/include/FeatureExtraction/ScannedCurve.h"
#include "VisionModule/include/VisionModule.h"

cv::Ptr<cv::CLAHE> FeatureExtraction::clahe;
Rect FeatureExtraction::footArea = Rect();
vector<int> FeatureExtraction::imageWidth = vector<int>(static_cast<size_t>(CameraId::count));
vector<int> FeatureExtraction::imageHeight = vector<int>(static_cast<size_t>(CameraId::count));
vector<uint8_t*> FeatureExtraction::image = vector<uint8_t*>(static_cast<size_t>(CameraId::count));
vector<vector<Mat>> FeatureExtraction::channels = vector<vector<Mat>>(static_cast<size_t>(CameraId::count));
vector<Mat> FeatureExtraction::grayImage = vector<Mat>(static_cast<size_t>(CameraId::count));
vector<Mat> FeatureExtraction::imageMat = vector<Mat>(static_cast<size_t>(CameraId::count));
vector<Mat> FeatureExtraction::bgrMat = vector<Mat>(static_cast<size_t>(CameraId::count));
TNColors FeatureExtraction::ourColor = TNColors::white;
TNColors FeatureExtraction::oppColor = TNColors::white;
bool FeatureExtraction::blackJerseyExists = false;
vector<CameraPtr> FeatureExtraction::cams = vector<CameraPtr>(static_cast<size_t>(CameraId::count));
CameraModulePtr FeatureExtraction::camModule;
vector<CameraTransformPtr> FeatureExtraction::cameraTransforms = vector<CameraTransformPtr>(static_cast<size_t>(CameraId::count));
ColorHandlerPtr FeatureExtraction::colorHandler;
vector<boost::shared_ptr<KnownLandmark<float> > > FeatureExtraction::knownLandmarks;
vector<boost::shared_ptr<UnknownLandmark<float> > > FeatureExtraction::unknownLandmarks;

FeatureExtraction::FeatureExtraction(VisionModule* visionModule, const string& name) :
  MemoryBase(visionModule),
  visionModule(visionModule),
  name(name),
  activeCamera(CameraId::headTop)
{
  cycleTime = visionModule->getPeriodMinMS() / ((float) 1000);
}

FeatureExtraction::~FeatureExtraction() {}

void FeatureExtraction::setup(VisionModule* visionModule)
{
  camModule = visionModule->getCameraModule();
  colorHandler = visionModule->getColorHandler();
  cameraTransforms = visionModule->getCameraTransforms();
  cams = camModule->getCameraPtrs();
  for (size_t i = 0; i < toUType(CameraId::count); ++i) {
    imageWidth[i] = cams[i]->width;
    imageHeight[i] = cams[i]->height;
    image[i] = cams[i]->image;
    grayImage[i] = Mat(Size(imageWidth[i], imageHeight[i]), CV_8U);
    imageMat[i] = Mat(Size(imageWidth[i], imageHeight[i]), CV_8UC2);
    bgrMat[i] = Mat(Size(imageWidth[i], imageHeight[i]), CV_8UC3);
  }
  clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
  // For lower camera only.
  // As defined in
  // https://pdfs.semanticscholar.org/3742/0e2c3fc50e89e14008e7ce584ddce52cce06.pdf
  footArea.y = 0.625 * imageHeight[1];
  footArea.height = (1 - 0.625) * imageHeight[1];
  footArea.x = 0.125 * imageWidth[1] / 2;
  // Because yuv422 when converted to uv image has half the width
  footArea.width = (0.875 - 0.125) * imageWidth[1] / 2;
}

void FeatureExtraction::updateImageMatrices(
  const CameraId& activeCamera, const bool& updateBgrMat)
{
  ///< Turn yuv422 data to opencv Mat
  image[toUType(activeCamera)] = cams[toUType(activeCamera)]->image;
  imageMat[toUType(activeCamera)].data = cams[toUType(activeCamera)]->image;
  ///< Get image channels
  split(imageMat[toUType(activeCamera)], channels[toUType(activeCamera)]);
  ///< Filter the image
  //clahe->apply(channels[toUType(activeCamera)][0], channels[toUType(activeCamera)][0]);
  //cv::merge(channels[toUType(activeCamera)], imageMat[toUType(activeCamera)]);
  ///< Set gray-scale image
  grayImage[toUType(activeCamera)] = channels[toUType(activeCamera)][0];
  ///< Update bgr image matrix if required
  if (updateBgrMat)
    cvtColor(imageMat[toUType(activeCamera)], bgrMat[toUType(activeCamera)], COLOR_YUV2BGR_YUY2);
}

void FeatureExtraction::createYuvHist(
  const CameraId& activeCamera, const bool& drawHists)
{
  // Prepare to get lower cam histograms
  Mat roi = channels[toUType(activeCamera)][1].reshape(2)(footArea);
  Mat binary;
  inRange(roi,
    Scalar(124, 124), // Lower range for white without a doubt
    Scalar(132, 132), // Upper range for white without a doubt
    binary);
  bitwise_not(binary, binary);
  // Inverse binary to mask the image for histogram calculations
  colorHandler->computeUVHist(roi, binary, drawHists);
}

uint8_t FeatureExtraction::getY(const int32_t& x, const int32_t& y) {
  return image[toUType(activeCamera)][(x + y * getImageWidth()) << 1];
}

uint8_t FeatureExtraction::getU(const int32_t& x, const int32_t& y) {
  return image[toUType(activeCamera)][(((x + y * getImageWidth()) >> 1) << 2) + 1];
}

uint8_t FeatureExtraction::getV(const int32_t& x, const int32_t& y) {
  return image[toUType(activeCamera)][((x + y * getImageWidth()) << 1) | 3];
}

TNColor FeatureExtraction::getYUV(const int32_t& x, const int32_t& y) {
  return TNColor(
    (int) image[toUType(activeCamera)][(x + y * getImageWidth()) << 1],
    (int) image[toUType(activeCamera)][(((x + y * getImageWidth()) >> 1) << 2) + 1],
    (int) image[toUType(activeCamera)][((x + y * getImageWidth()) << 1) | 3]);
}

TNColor FeatureExtraction::getYUV(const int& index, const int32_t& x, const int32_t& y) {
  return TNColor(
    (int) image[index][(x + y * imageWidth[index]) << 1],
    (int) image[index][(((x + y * imageWidth[index]) >> 1) << 2) + 1],
    (int) image[index][((x + y * imageWidth[index]) << 1) | 3]);
}

void FeatureExtraction::updateColorInfo(const TNColors& ourColor,
  const TNColors& oppColor, const bool& blackJerseyExists)
{
  FeatureExtraction::ourColor = ourColor;
  FeatureExtraction::oppColor = oppColor;
  FeatureExtraction::blackJerseyExists = blackJerseyExists;
}

Mat FeatureExtraction::makeYuvMat()
{
  Mat yuv = Mat(Size(getImageWidth(), getImageHeight()), CV_8UC3);
  uchar* p;
  for (size_t y = 0; y < getImageHeight(); ++y) {
    for (size_t x = 0; x < getImageWidth() * 3; x = x + 3) {
      p = yuv.ptr < uchar > (y);
      auto yuvp = getYUV(x / 3, y);
      p[x] = yuvp.y();
      p[x + 1] = yuvp.u();
      p[x + 2] = yuvp.v();
    }
  }
  return yuv;
}

Mat FeatureExtraction::makeYuvMat(const CameraId index)
{
  Mat yuv;
  cvtColor(bgrMat[toUType(index)], yuv, COLOR_BGR2YUV);
  return yuv;
}

void FeatureExtraction::linkScannedLines(
  vector<LinearScannedLinePtr>& chains,
  vector<LinearScannedLinePtr>& scannedLines,
  const int& lowTol,
  const int& highTol,
  const int& lenTol,
  cv::Mat& image)
{
  typedef vector<LinearScannedLinePtr>::iterator slIter;
  slIter iter = scannedLines.begin();
  while (iter != scannedLines.end()) {
    if (*iter) ++iter;
    else iter = scannedLines.erase(iter);
  }

  sort(
    scannedLines.begin(),
    scannedLines.end(),
    [](const LinearScannedLinePtr& sl1, const LinearScannedLinePtr& sl2)
    { return sl1->baseIndex < sl2->baseIndex;});

  LinearScannedLinePtr pred;
  for (const auto& sl : scannedLines) {
    sl->closestDist = 1000;
    sl->pred.reset();
    sl->bestNeighbor.reset();
    if (pred) sl->pred = pred;
    pred = sl;
  }

  //Mat image2 = image.clone();
  auto maxDist = sqrt(highTol * highTol + lowTol * lowTol);
  for (const auto& sl : scannedLines) {
    //cout << "sl:" << sl <<endl;
    //sl->draw(image2, cv::Scalar(255,0,0));
    auto neighbor = sl->pred;
    //cout << "neighbor:" << neighbor<<endl;
    while (neighbor) {
      //cout << "neighbor:" << neighbor<<endl;
      //cout << "sl:" << sl <<endl;
      //neighbor->draw(image2, cv::Scalar(0,0,255));
      auto diffLen = abs(sl->len - neighbor->len);
      //cout << "len threshold: " << diffLen / (float) sl->len << endl;
      if (diffLen / (float) sl->len < lenTol) {
        auto diffHigh = fabsf(sl->baseIndex - neighbor->baseIndex);
        //cout << "diffHigh:" << diffHigh << endl;
        if (diffHigh < highTol) {
          auto diffLow = std::min(fabsf(neighbor->start - sl->start), fabsf(neighbor->end - sl->end));
          //cout << "diffLow:" << diffLow << endl;
          auto dist = sqrt(diffLow * diffLow + diffHigh * diffHigh);
          if (dist < maxDist && dist < sl->closestDist) {
            //cout << "dist:" << dist << endl;
            sl->closestDist = dist;
            sl->bestNeighbor = neighbor;
            break;
          }
        }
      }
      //VisionUtils::displayImage("image2", image2);
      //waitKey(0);
      neighbor = neighbor->pred;
    }
  }
  reverse(scannedLines.begin(), scannedLines.end());
  chains.clear();
  for (const auto& sl : scannedLines) {
    if (sl->searched) continue;
    chains.push_back(sl);
    LinearScannedLinePtr neighbor = sl->bestNeighbor;
    auto chain = 0;
    chain++;
    while (neighbor) {
      if (neighbor->searched) break;
      neighbor->searched = true;
      neighbor = neighbor->bestNeighbor;
      ++chain;
    }
    sl->chainLength = chain;
    sl->searched = true;
  }
  sort(
    chains.begin(),
    chains.end(),
    [](const LinearScannedLinePtr& sl1, const LinearScannedLinePtr& sl2)
    { return sl1->chainLength > sl2->chainLength;});
}

void FeatureExtraction::findRegions(
  vector<ScannedRegionPtr>& regions,
  vector<LinearScannedLinePtr>& scannedLines,
  const int& lowTol,
  const int& highTol,
  const int& lenTol,
  cv::Mat& image)
{
  vector<LinearScannedLinePtr> chains;
  linkScannedLines(chains, scannedLines, lowTol, highTol, lenTol, image);
  for (const auto& sl : chains) {
    if (sl->chainLength > 0) {
      auto sc = boost::make_shared<ScannedCurve>();
      sc->upper.push_back(sl->getStartPoint());
      sc->lower.push_back(sl->getEndPoint());
      LinearScannedLinePtr neighbor = sl->bestNeighbor;
      while (neighbor != 0) {
        sc->upper.push_back(neighbor->getStartPoint());
        sc->lower.push_back(neighbor->getEndPoint());
        neighbor = neighbor->bestNeighbor;
      }
      reverse(sc->lower.begin(), sc->lower.end());
      sc->upper.insert(sc->upper.end(), sc->lower.begin(), sc->lower.end());
      ScannedRegionPtr sr = boost::make_shared <ScannedRegion> (sc->upper);
      regions.push_back(sr);
    }
  }
}

void FeatureExtraction::findConvexHulls(
  vector<vector<Point>>& hulls,
  vector<LinearScannedLinePtr>& scannedLines,
  const int& lowTol,
  const int& highTol,
  const int& lenTol,
  cv::Mat& image)
{
  vector<LinearScannedLinePtr> chains;
  linkScannedLines(chains, scannedLines, lowTol, highTol, lenTol, image);
  for (const auto& sl : chains) {
    if (sl->chainLength > 0) {
      auto sc = boost::make_shared<ScannedCurve>();
      sc->upper.push_back(sl->getStartPoint());
      sc->lower.push_back(sl->getEndPoint());
      LinearScannedLinePtr neighbor = sl->bestNeighbor;
      while (neighbor != 0) {
        sc->upper.push_back(neighbor->getStartPoint());
        sc->lower.push_back(neighbor->getEndPoint());
        neighbor = neighbor->bestNeighbor;
      }
      reverse(sc->lower.begin(), sc->lower.end());
      sc->upper.insert(sc->upper.end(), sc->lower.begin(), sc->lower.end());
      vector<Point> hull;
      cv::convexHull(sc->upper, hull);
      hulls.push_back(hull);
    }
  }
}


void FeatureExtraction::filterScannedRegions(
  vector<ScannedRegionPtr>& regions,
  const int& xTol,
  const int& yTol)
{
  for (auto& r1 : regions) {
    for (auto& r2 : regions) {
      if (r1 && r2 && r1 != r2) {
        Rect overlap = r1->rect & r2->rect;
        if (overlap.area() > 0) {
          r1 = boost::make_shared<ScannedRegion>(r1->rect | r2->rect);
          r2.reset();
          continue;
        }
        auto dx = abs(r1->center.x - r2->center.x);
        auto wsum = r1->rect.width / 2 + r2->rect.width / 2;
        if (dx > wsum)
          dx -= wsum;
        else
          dx = 0;
        auto dy = abs(r1->center.y - r2->center.y);
        auto hsum = r1->rect.height / 2 + r2->rect.height / 2;
        if (dy > hsum)
          dy -= hsum;
        else
          dy = 0;
        if (dx < xTol && dy < yTol) {
          r1 = boost::make_shared < ScannedRegion > (r1->rect | r2->rect);
          r2.reset();
        }
      }
    }
  }
}
