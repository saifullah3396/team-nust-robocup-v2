/**
 * @file FeatureExtraction/FeatureExtraction.h
 *
 * This file declares the base class for feature extraction
 * from the image. All the functions and algorithms for detecting
 * features from the image will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017
 */

#pragma once

#include <boost/circular_buffer.hpp>
#include <boost/make_shared.hpp>
#include <chrono>
#include "UserCommModule/include/UserCommRequest.h"
#include "VisionModule/include/VisionModule.h"
#include "VisionModule/include/FeatureExtraction/ScannedCurve.h"
#include "VisionModule/include/FeatureExtraction/ScannedEdge.h"
#include "VisionModule/include/FeatureExtraction/ScannedLine.h"
#include "VisionModule/include/FeatureExtraction/ScannedRegion.h"
#include "TNRSBase/include/MemoryBase.h"
#include "LocalizationModule/include/FieldLandmarkIds.h"
#include "VisionModule/include/CameraModule.h"
#include "VisionModule/include/CameraTransform.h"
#include "VisionModule/include/ColorHandler.h"
//#include "VisionModule/include/VisionModule.h"
#include "Utils/include/DataHolders/Landmark.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/VisionUtils.h"

using namespace std::chrono;

#define GET_FEATURE_EXT_CLASS(name, index) \
  boost::static_pointer_cast<name>(visionModule->getFeatureExtClass(index));

class VisionModule;

class FeatureExtraction : public MemoryBase
{
public:
  /**
   * @brief FeatureExtraction Constructor
   * @param visionModule Pointer to base vision module
   */
  FeatureExtraction(VisionModule* visionModule);

  /**
   * @brief ~FeatureExtraction Destructor
   */
  ~FeatureExtraction();

  /**
   * @brief setup Initializes all the image containers
   * @param visionModule
   */
  static void setup(VisionModule* visionModule);

  /**
   * @brief processImage Main image processing routine
   */
  virtual void processImage() = 0;

  /**
   * @brief setupImagesAndHists Sets up the current image for extraction
   */
  static void setupImagesAndHists();

  /**
   * @brief updateColorInfo Updates the current jersey color information
   * @param ourColor Color of our team
   * @param oppColor Color of opponents team
   * @param blackJerseyExists Whether one of the either teams
   *   has a black jersey
   */
  static void updateColorInfo(
    const TNColors& ourColor,
    const TNColors& oppColor,
    const bool& blackJerseyExists);

  /**
   * @brief clearLandmarks Clears the current information about
   *   observed landmarks
   */
  static void clearLandmarks() {
    knownLandmarks.clear();
    unknownLandmarks.clear();
  }

  //! Sets the camera index for the given class
  void setCurrentImage(const unsigned& imageIndex)
    { this->currentImage = imageIndex; }

  //! Getters
  unsigned& getCurrentImage() { return currentImage; }
  static const uint8_t* getYuv422Image(const unsigned& index) { return image[index]; }
  static size_t getBytes(const unsigned& index) { return imageWidth[index] * imageHeight[index] * 2; }
  static Mat& getBgrMat(const unsigned& index) { return bgrMat[index]; }
  static Mat& getYuvImage(const unsigned& index) { return imageMat[index]; }
  static Mat& getGrayMat(const unsigned& index) { return grayImage[index]; }
  static vector<boost::shared_ptr<KnownLandmark<float> > >
    getKnownLandmarks() { return knownLandmarks; }
  static vector<boost::shared_ptr<UnknownLandmark<float> > >
    getUnknownLandmarks() { return unknownLandmarks; }

protected:
  /**
   * @brief drawResults The function should be used for drawing the
   *   results of feature extraction
   */
  virtual void drawResults() {}

  //! Getters
  const uint8_t* getYuv422Image() { return image[currentImage]; }
  Mat& getYuvImage() { return imageMat[currentImage]; }
  Mat& getGrayImage() { return grayImage[currentImage]; }
  int& getImageWidth() { return imageWidth[currentImage]; }
  int& getImageHeight() { return imageHeight[currentImage]; }

  /**
   * @brief getY Returns the Y-component of YUV422 raw image
   * @param x Column index
   * @param y Row index
   * @return Y
   */
  uint8_t getY(const int32_t& x, const int32_t& y) {
    return image[currentImage][(x + y * getImageWidth()) << 1];
  }

  /**
   * @brief getU Returns the U-component of the YUV422 raw image
   * @param x Column index
   * @param y Row index
   * @return U
   */
  uint8_t getU(const int32_t& x, const int32_t& y) {
    return image[currentImage][(((x + y * getImageWidth()) >> 1) << 2) + 1];
  }

  /**
   * @brief getV Returns the V-component of the YUV422 raw image
   * @param x Column index
   * @param y Row index
   * @return V
   */
  inline uint8_t getV(const int32_t& x, const int32_t& y) {
    return image[currentImage][((x + y * getImageWidth()) << 1) | 3];
  }

  /**
   * @brief getYUV Returns the YUV color of the YUV422 raw image
   * @param x Column index
   * @param y Row index
   * @return TNColor
   */
  TNColor getYUV(const int32_t& x, const int32_t& y) {
    return TNColor(
      (int) image[currentImage][(x + y * getImageWidth()) << 1],
      (int) image[currentImage][(((x + y * getImageWidth()) >> 1) << 2) + 1],
      (int) image[currentImage][((x + y * getImageWidth()) << 1) | 3]);
  }

  /**
   * @brief getYUV Returns the YUV color of the required YUV422 raw image
   * @param index Image index
   * @param x Column index
   * @param y Row index
   * @return TNColor
   */
  static TNColor getYUV(const int& index, const int32_t& x, const int32_t& y) {
    return TNColor(
      (int) image[index][(x + y * imageWidth[index]) << 1],
      (int) image[index][(((x + y * imageWidth[index]) >> 1) << 2) + 1],
      (int) image[index][((x + y * imageWidth[index]) << 1) | 3]);
  }

  /**
   * @brief makeYuvMat Makes a YUV matrix from a YUV422 compressed
   *   image matrix
   * @return cv::Mat
   */
  Mat makeYuvMat()
  {
    Mat yuv = Mat(Size(getImageWidth(), getImageHeight()), CV_8UC3);
    uchar* p;
    for (int y = 0; y < getImageHeight(); ++y) {
      for (int x = 0; x < getImageWidth() * 3; x = x + 3) {
        p = yuv.ptr < uchar > (y);
        auto yuvp = getYUV(x / 3, y);
        p[x] = yuvp.y();
        p[x + 1] = yuvp.u();
        p[x + 2] = yuvp.v();
      }
    }
    return yuv;
  }

  static void computeUVHist(const Mat& uv422, const bool& drawHists);

  static inline void
  findRegions(vector<ScannedRegionPtr>& regions,
    vector<ScannedLinePtr>& scannedLines, const unsigned& xTol,
    const unsigned& yTol, const bool& horizontal)
  {
    typedef vector<ScannedLinePtr>::iterator slIter;
    slIter iter = scannedLines.begin();
    while (iter != scannedLines.end()) {
      if (*iter) ++iter;
      else iter = scannedLines.erase(iter);
    }

    sort(
      scannedLines.begin(),
      scannedLines.end(),
      [](const ScannedLinePtr& sl1, const ScannedLinePtr& sl2)
      { return sl1->p1.x < sl2->p1.x;});

    ScannedLinePtr pred;
    for (int i = 0; i < scannedLines.size(); ++i) {
      ScannedLinePtr sl = scannedLines[i];
      sl->closestDist = 1000;
      sl->bestNeighbor.reset();
      if (pred) {
        sl->pred = pred;
      }
      pred = sl;
    }

    int maxDist = sqrt(yTol * yTol + xTol * xTol);
    for (int i = 0; i < scannedLines.size(); ++i) {
      ScannedLinePtr sl = scannedLines[i];
      ScannedLinePtr neighbor = sl->pred;
      while (neighbor) {
        int diffY;
        if (horizontal) {
          diffY = abs(neighbor->p2.y - sl->p2.y);
        } else {
          int diff1 = abs(neighbor->p1.y - sl->p1.y);
          int diff2 = abs(neighbor->p2.y - sl->p2.y);
          diffY = diff1 < diff2 ? diff1 : diff2;
        }
        if (diffY < yTol) {
          int diffX;
          if (horizontal) {
            int diff1 = abs(neighbor->p1.x - sl->p1.x);
            int diff2 = abs(neighbor->p2.x - sl->p2.x);
            diffX = diff1 < diff2 ? diff1 : diff2;
          } else {
            diffX = neighbor->p2.x - sl->p2.x;
          }
          int dist = sqrt(diffX * diffX + diffY * diffY);
          if (dist < maxDist) {
            if (dist < sl->closestDist) {
              sl->closestDist = dist;
              sl->bestNeighbor = neighbor;
            }
          }
        }
        neighbor = neighbor->pred;
      }
    }
    reverse(scannedLines.begin(), scannedLines.end());
    vector<ScannedLinePtr> chainParents;
    for (int i = 0; i < scannedLines.size(); ++i) {
      ScannedLinePtr sl = scannedLines[i];
      if (sl->searched) continue;
      chainParents.push_back(sl);
      ScannedLinePtr neighbor = sl->bestNeighbor;
      int chain = 0;
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
      chainParents.begin(),
      chainParents.end(),
      [](const ScannedLinePtr& sl1, const ScannedLinePtr& sl2)
      { return sl1->chainLength > sl2->chainLength;});

    for (int i = 0; i < chainParents.size(); ++i) {
      ScannedLinePtr sl = chainParents[i];
      if (sl->chainLength > 0) {
        auto sc = boost::make_shared<ScannedCurve>();
        sc->upper.push_back(sl->p1);
        sc->lower.push_back(sl->p2);
        ScannedLinePtr neighbor = sl->bestNeighbor;
        while (neighbor != 0) {
          sc->upper.push_back(neighbor->p1);
          sc->lower.push_back(neighbor->p2);
          neighbor = neighbor->bestNeighbor;
        }
        reverse(sc->lower.begin(), sc->lower.end());
        sc->upper.insert(sc->upper.end(), sc->lower.begin(), sc->lower.end());
        //VisionUtils::drawPoints(sc->upper, bgrMat[0]);
        //waitKey(0);
        ScannedRegionPtr sr = boost::make_shared <ScannedRegion> (sc->upper);
        regions.push_back(sr);
      }
    }
  }

  /**
   * @brief filterRegions Filters out regions based on distance threshold
   * @param regions Input regions
   * @param threshold Distance threshold in pixels
   */
  void filterScannedRegions(
    vector<ScannedRegionPtr>& regions,
    const unsigned& threshold)
  {
    for (size_t i = 0; i < regions.size(); ++i) {
      if (!regions[i]) continue;
      for (size_t j = 0; j < regions.size(); ++j) {
        if (!regions[j]) continue;
        if (i != j) {
          auto r1 = regions[i]->rect;
          auto r2 = regions[j]->rect;
          Rect overlap = r1 & r2;
          if (overlap.area() > 0) {
            regions[i] = boost::make_shared < ScannedRegion > (r1 | r2);
            regions[j].reset();
            continue;
          }
          int x11 = r1.x;
          int x12 = r1.x + r1.width;
          int x21 = r2.x;
          int x22 = r2.x + r2.width;
          int d1 = abs(x11 - x21);
          int d2 = abs(x11 - x22);
          int d3 = abs(x12 - x21);
          int d4 = abs(x12 - x22);
          int distX = d1 < d2 ? d1 : d2;
          distX = distX < d3 ? distX : d3;
          distX = distX < d4 ? distX : d4;
          if (distX < threshold) {
            int y11 = r1.y;
            int y12 = r1.y + r1.height;
            int y21 = r2.y;
            int y22 = r2.y + r2.height;
            int d1 = abs(y11 - y21);
            int d2 = abs(y11 - y22);
            int d3 = abs(y12 - y21);
            int d4 = abs(y12 - y22);
            int distY = d1 < d2 ? d1 : d2;
            distY = distY < d3 ? distY : d3;
            distY = distY < d4 ? distY : d4;
            if (distY < threshold) {
              regions[i] = boost::make_shared < ScannedRegion > (r1 | r2);
              regions[j].reset();
            }
          }
        }
      }
    }
  }

  /**
   * Vector of input image data pointers
   *
   * @var vector<uint8_t*>
   */
  static vector<uint8_t*> image;

  /**
   * Vector of gray scale image matrix of input images
   *
   * @var vector<Mat>
   */
  static vector<Mat> grayImage;

  /**
   * Vector of yuv422 image matrix of input images
   *
   * @var vector<Mat>
   */
  static vector<Mat> imageMat;

  /**
   * Vector of bgr image matrix of input images
   *
   * @var vector<Mat>
   */
  static vector<Mat> bgrMat;

  /**
   * Vector of input image widths
   *
   * @var vector<int>
   */
  static vector<int> imageWidth;

  /**
   * Vector of input image heights
   *
   * @var vector<int>
   */
  static vector<int> imageHeight;

  //! Vector for storing known landmarks observation
  static vector<boost::shared_ptr<KnownLandmark<float> >> knownLandmarks;

  //! Vector for storing unknown landmarks observation
  static vector<boost::shared_ptr<UnknownLandmark<float> >> unknownLandmarks;

  /**
   * Current image index
   */
  unsigned currentImage;

  //! Feature extraction iteration start time
  clock_t iterationStartTime;

  //! Rect defining the lower cam foot area for the robot.
  static Rect footArea;

  //! Pointer to color handler class.
  static ColorHandlerPtr colorHandler;

  //! Pointer to image transform class.
  static vector<CameraTransformPtr> cameraTransforms;

  //! Our team color
  static TNColors ourColor;

  //! Opponent team color
  static TNColors oppColor;

  //! If a black team exists in game
  static bool blackJerseyExists;

  //! Cycle time of vision module
  float cycleTime;

  //! VisionModule pointer
  VisionModule* visionModule;
private:
  //! Pointer to camera module class.
  static CameraModulePtr camModule;

  //! Pointers to camera objects.
  static vector<CameraPtr> cams;
};

typedef boost::shared_ptr<FeatureExtraction> FeatureExtractionPtr;
