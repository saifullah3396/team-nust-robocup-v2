/**
 * @file VisionModule/include/FeatureExtraction/FeatureExtraction.h
 *
 * This file declares the base class for feature extraction
 * from the image. All the functions and algorithms for detecting
 * features from the image will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/make_shared.hpp>
#include <chrono>
#include "TNRSBase/include/MemoryBase.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/EnumUtils.h"

using namespace std::chrono;
using namespace cv;

#define GET_FEATURE_EXT_CLASS(name, index) \
  boost::static_pointer_cast<name>(visionModule->getFeatureExtClass(index));

enum class TNColors : unsigned int;
class TNColor;
template <typename T>
class Camera;
class CameraTransform;
class CameraModule;
class ColorHandler;
class VisionModule;
struct ScannedRegion;
struct ScannedLine;
struct LinearScannedLine;
typedef boost::shared_ptr<Camera<float>> CameraPtr;
typedef boost::shared_ptr<CameraModule> CameraModulePtr;
typedef boost::shared_ptr<CameraTransform> CameraTransformPtr;
typedef boost::shared_ptr<ColorHandler> ColorHandlerPtr;
typedef boost::shared_ptr<ScannedRegion> ScannedRegionPtr;
typedef boost::shared_ptr<ScannedLine> ScannedLinePtr;
typedef boost::shared_ptr<LinearScannedLine> LinearScannedLinePtr;

class FeatureExtraction : public MemoryBase
{
public:
  /**
   * @brief FeatureExtraction Constructor
   * @param visionModule Pointer to base vision modul
   * @param name Module name
   */
  FeatureExtraction(VisionModule* visionModule, const string& name);

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
   * @brief updateImageMatrices Updates the image matrices from current image
   * @param activeCamera Current active image
   * @param updateBgrMat Whether to update bgr matrix
   */
  static void updateImageMatrices(
    const CameraId& activeCamera, const bool& updateBgrMat);

  /**
   * @brief createYuvHist Creates yuv histograms from lower camera image
   * @param activeCamera Current active image
   * @param drawHists Whether to draw the histograms
   */
  static void createYuvHist(
    const CameraId& activeCamera, const bool& drawHists);

  /**
   * @brief processImage Main image processing routine
   * @param activeCamera Current active image
   */
  virtual void processImage() = 0;

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
   * @brief makeYuvMat Creates a yuv matrix for the given image index
   * @param index
   * @return
   */
  static cv::Mat makeYuvMat(const CameraId index);

  /**
   * @brief clearLandmarks Clears the current information about
   *   observed landmarks
   */
  static void clearLandmarks() {
    knownLandmarks.clear();
    unknownLandmarks.clear();
  }

  ///< Setters
  void setActiveCamera(const CameraId& activeCamera)
    { this->activeCamera = activeCamera; }
  void setEnabled(const bool& enabled)
    { this->enabled = enabled; }

  ///< Getters
  const string& getName() const { return name; }
  const bool& isEnabled() const { return enabled; }
  const CameraId& getActiveCamera() const { return activeCamera; }
  static const uint8_t* getYuv422Image(const unsigned& index) { return image[index]; }
  static const size_t getBytes(const unsigned& index) { return imageWidth[index] * imageHeight[index] * 2; }
  static cv::Mat& getBgrMat(const unsigned& index) { return bgrMat[index]; }
  static const cv::Mat& getYuvImage(const unsigned& index) { return imageMat[index]; }
  static const cv::Mat& getGrayMat(const unsigned& index) { return grayImage[index]; }
  static const vector<boost::shared_ptr<KnownLandmark<float> > > getKnownLandmarks()
    { return knownLandmarks; }
  static const vector<boost::shared_ptr<UnknownLandmark<float> > > getUnknownLandmarks()
    { return unknownLandmarks; }

protected:
  /**
   * @brief getYUV Returns the YUV color of the required YUV422 raw image
   * @param index Image index
   * @param x Column index
   * @param y Row index
   * @return TNColor
   */
  static TNColor getYUV(const int& index, const int32_t& x, const int32_t& y);

  static void findRegions(
    vector<ScannedRegionPtr>& regions,
    vector<LinearScannedLinePtr>& scannedLines,
    const int& lowTol,
    const int& highTol,
    const int& lenTol,
    cv::Mat& image);

  static void findConvexHulls(
    vector<vector<Point>>& hulls,
    vector<LinearScannedLinePtr>& scannedLines,
    const int& lowTol,
    const int& highTol,
    const int& lenTol,
    cv::Mat& image);

  static void linkScannedLines(
    vector<LinearScannedLinePtr>& chains,
    vector<LinearScannedLinePtr>& scannedLines,
    const int& lowTol,
    const int& highTol,
    const int& lenTol,
    cv::Mat& image);

  /**
   * @brief makeYuvMat Makes a YUV matrix from a YUV422 compressed
   *   image matrix
   * @return cv::Mat
   */
  Mat makeYuvMat();

  /**
   * @brief drawResults The function should be used for drawing the
   *   results of feature extraction
   */
  virtual void drawResults() {}

  ///< Getters
  const uint8_t* getYuv422Image() { return image[toUType(activeCamera)]; }
  cv::Mat& getYuvImage() { return imageMat[toUType(activeCamera)]; }
  cv::Mat& getGrayImage() { return grayImage[toUType(activeCamera)]; }
  int& getImageWidth() { return imageWidth[toUType(activeCamera)]; }
  int& getImageHeight() { return imageHeight[toUType(activeCamera)]; }

  /**
   * @brief getY Returns the Y-component of YUV422 raw image
   * @param x Column index
   * @param y Row index
   * @return Y
   */
  uint8_t getY(const int32_t& x, const int32_t& y);

  /**
   * @brief getU Returns the U-component of the YUV422 raw image
   * @param x Column index
   * @param y Row index
   * @return U
   */
  uint8_t getU(const int32_t& x, const int32_t& y);

  /**
   * @brief getV Returns the V-component of the YUV422 raw image
   * @param x Column index
   * @param y Row index
   * @return V
   */
  uint8_t getV(const int32_t& x, const int32_t& y);

  /**
   * @brief getYUV Returns the YUV color of the YUV422 raw image
   * @param x Column index
   * @param y Row index
   * @return TNColor
   */
  TNColor getYUV(const int32_t& x, const int32_t& y);

  /**
   * @brief filterRegions Filters out regions based on distance threshold
   * @param regions Input regions
   * @param threshold Distance threshold in pixels
   */
  void filterScannedRegions(
    vector<ScannedRegionPtr>& regions,
    const int& xTol,
    const int& yTol);

  ///< Vector of input image data pointers
  static vector<uint8_t*> image;

  ///< Vector of gray scale image matrix of input images
  static vector<cv::Mat> grayImage;

  ///< Vector of image channels
  static vector<vector<cv::Mat>> channels;

  ///< Vector of yuv422 image matrix of input images
  static vector<cv::Mat> imageMat;

  ///< Vector of bgr image matrix of input images
  static vector<cv::Mat> bgrMat;

  ///< Vector of input image widths
  static vector<int> imageWidth;

  ///< Vector of input image heights
  static vector<int> imageHeight;

  ///< Vector for storing known landmarks observation
  static vector<boost::shared_ptr<KnownLandmark<float> >> knownLandmarks;

  ///< Vector for storing unknown landmarks observation
  static vector<boost::shared_ptr<UnknownLandmark<float> >> unknownLandmarks;

  ///< Rect defining the lower cam foot area for the robot.
  static Rect footArea;

  ///< Pointer to color handler class.
  static ColorHandlerPtr colorHandler;

  ///< Pointer to image transform class.
  static vector<CameraTransformPtr> cameraTransforms;

  ///< Our team color
  static TNColors ourColor;

  ///< Opponent team color
  static TNColors oppColor;

  ///< If a black team exists in game
  static bool blackJerseyExists;

  ///< Current image index
  CameraId activeCamera;

  ///< Whether this module is enabled
  bool enabled = {false};

  ///< Feature extraction iteration start time
  clock_t iterationStartTime;

  ///< Cycle time of vision module
  float cycleTime;

  ///< VisionModule pointer
  VisionModule* visionModule;

  ///< Class name
  string name;
private:
  ///< Pointer to camera module class.
  static CameraModulePtr camModule;

  ///< Pointers to camera objects.
  static vector<CameraPtr> cams;

  ///< CLAHE Filter object
  static cv::Ptr<cv::CLAHE> clahe;
};

typedef boost::shared_ptr<FeatureExtraction> FeatureExtractionPtr;
