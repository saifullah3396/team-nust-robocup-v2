/**
 * @file FeatureExtraction/FeatureExtraction.cpp
 *
 * This file implements the base class for feature extraction
 * from the image. All the functions and algorithms for detecting
 * features from the image will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017
 */

#include <opencv2/photo/photo.hpp>
#include "Utils/include/DataHolders/Camera.h"
#include "VisionModule/include/CameraModule.h"
#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"

Rect FeatureExtraction::footArea = Rect();
vector<int> FeatureExtraction::imageWidth = vector<int>(static_cast<size_t>(CameraId::count));
vector<int> FeatureExtraction::imageHeight = vector<int>(static_cast<size_t>(CameraId::count));
vector<uint8_t*> FeatureExtraction::image = vector<uint8_t*>(static_cast<size_t>(CameraId::count));
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

FeatureExtraction::FeatureExtraction(VisionModule* visionModule) :
  MemoryBase(visionModule),
  visionModule(visionModule),
  currentImage(static_cast<unsigned>(CameraId::headTop))
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
  //LOG_INFO("Setting up FeatureExtraction...");
  //LOG_INFO("cams:" << cams.size());
  for (size_t i = 0; i < toUType(CameraId::count); ++i) {
    imageWidth[i] = cams[i]->width;
    imageHeight[i] = cams[i]->height;
    image[i] = cams[i]->image;
    grayImage[i] = Mat(Size(imageWidth[i], imageHeight[i]), CV_8U);
    imageMat[i] = Mat(Size(imageWidth[i], imageHeight[i]), CV_8UC2);
    bgrMat[i] = Mat(Size(imageWidth[i], imageHeight[i]), CV_8UC3);
  }
  // For lower camera only.
  // As defined in
  // https://pdfs.semanticscholar.org/3742/0e2c3fc50e89e14008e7ce584ddce52cce06.pdf
  footArea.y = 0.625 * imageHeight[1];
  footArea.height = (1 - 0.625) * imageHeight[1];
  footArea.x = 0.125 * imageWidth[1] / 2;
  // Because yuv422 when converted to uv image has half the width
  footArea.width = (0.875 - 0.125) * imageWidth[1] / 2;
}

void FeatureExtraction::setupImagesAndHists()
{
  //LOG_INFO("Setting up images and histograms...");
  for (size_t i = 0; i < toUType(CameraId::count); ++i) {
    imageMat[i].data = cams[i]->image;
//#ifdef MODULE_IS_REMOTE
    //fastNlMeansDenoising(imageMat[i], imageMat[i], 3, 7, 5);
    cvtColor(imageMat[i], bgrMat[i], COLOR_YUV2BGR_YUY2);
//#endif
    if (i == toUType(CameraId::headBottom)) {
      Mat yuv422Ch[2];
      split(imageMat[i], yuv422Ch);
      // Set lower cam gray-scale image
      grayImage[i] = yuv422Ch[0];
      // Prepare to get lower cam histograms
      Mat roi = yuv422Ch[1].reshape(2)(footArea);
      Mat binary;
      inRange(roi, Scalar(124, 124), // Lower range for white without a doubt
      Scalar(132, 132), // Upper range for white without a doubt
      binary);
      bitwise_not(binary, binary);
      // Inverse binary to mask the image for histogram calculations
      colorHandler->computeUVHist(roi, binary, false);
      //Mat shaped = yuv422Ch[1].reshape(2);
      //Mat uvCh[2];
      //split(shaped, uvCh);
      //Mat u = Mat(Size(imageWidth[i], imageHeight[i]), CV_8UC1);
      //Mat v = Mat(Size(imageWidth[i], imageHeight[i]), CV_8UC1);
      //resize(uvCh[0], u, u.size());
      //resize(uvCh[1], v, v.size());
      //imshow("uvCh[0]", uvCh[0]);
      //imshow("uvCh[1]", uvCh[1]);
      //imshow("v", u);
      //imshow("u", v);
      //std::vector<cv::Mat> array_to_merge;
      //array_to_merge.push_back(grayImage[i]);
      //array_to_merge.push_back(u);
      //array_to_merge.push_back(v);
      //cv::Mat color
      //cv::merge(array_to_merge, color);
      //imshow("color", color);
      //waitKey(0);
      //cvtColor(bgrMat[i], color, COLOR_RGB2YUV);
      //imshow("color2", color);
      //waitKey(0);
    } else {
      cvtColor(imageMat[i], grayImage[i], COLOR_YUV2GRAY_YUY2);
    }
  }
}

void
FeatureExtraction::updateColorInfo(const TNColors& ourColor,
  const TNColors& oppColor, const bool& blackJerseyExists)
{
  FeatureExtraction::ourColor = ourColor;
  FeatureExtraction::oppColor = oppColor;
  FeatureExtraction::blackJerseyExists = blackJerseyExists;
}
