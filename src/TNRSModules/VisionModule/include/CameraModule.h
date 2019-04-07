/**
 * @file VisionModule/include/CameraModule.h
 *
 * This file declares the class CameraModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <vector>
#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  #ifndef V6_CROSS_BUILD
    #include <alvision/alimage.h>
    #include <alproxies/alvideodeviceproxy.h>
  #endif
#endif
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "TNRSBase/include/DebugBase.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/alvisiondefinitions.h"

using namespace std;

class VisionModule;
template <typename T>
struct Camera;
typedef boost::shared_ptr<Camera<float> > CameraPtr;
#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  #ifndef V6_CROSS_BUILD
    typedef boost::shared_ptr<AL::ALVideoDeviceProxy> ALVideoDeviceProxyPtr;
  #endif
#endif

/**
 * @class CameraModule
 * @brief This BaseModule that updates camera information to local
 *   shared memory
 */
#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
class CameraModule
{
#else
class CameraModule : public DebugBase
{
  INIT_DEBUG_BASE_(
    (vector<int>, settingParams, vector<int>()),
  );
#endif
public:
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  /**
   * @brief CameraModule Constructor
   * @param visionModule Pointer to base vision module
   */
  CameraModule(VisionModule* visionModule);
  #else
  /**
   * @brief CameraModule Constructor
   */
  CameraModule();
  #endif
  /**
   * Destructor
   */
  ~CameraModule();

#ifdef MODULE_IS_REMOTE
  /**
   * Retrieves image from the camera
   *
   * @param index the camera index
   * @param saveImages whether to save images in respective robot dir
   * @param useLoggedImages whether to use logged images for processing
   * @return void
   */
  void updateImage(
    const CameraId& index,
    const int& saveImages = false,
    const bool& useLoggedImages = false);
#else
  /**
   * Retrieves image from the camera
   *
   * @param index the camera index
   * @return void
   */
  void updateImage(
    const CameraId& index);
#endif
  void recordVideo(const CameraId& index);
  void stopRecording();

  /**
   * Releases the image from the camera
   *
   * @param index the camera index
   * @return void
   */
  void releaseImage(const CameraId& index);

  /**
   * Gets the pointer to given camera.
   *
   * @return a pointer to required camera.
   */
  CameraPtr getCameraPtr(const CameraId& index)
  {
    return cams[static_cast<unsigned>(index)];
  }

  /**
   * Gets a vector of pointers to both cameras.
   *
   * @return a vector of pointers to both cameras.
   */
  vector<CameraPtr>& getCameraPtrs() { return cams; }

  static void
  bgrToYuv422(uint8_t* out, const cv::Mat& in, const int& width, const int& height);

  static void
  yuv422ToBgr(cv::Mat& out, const uint8_t * const in, const int& width, const int& height);

private:
  /**
   * Sets up the camera input streaming configuration and image format
   * configuration
   *
   * @return bool true if camera setup is successful
   */
  bool setupCameras();

  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
    #ifndef V6_CROSS_BUILD
      ///< Pointer to NaoQi video device proxy
      ALVideoDeviceProxyPtr camProxy;
    #else
      ///< Pointer to NaoQi video device proxy
      qi::AnyObject camProxy;
    #endif
  #endif
  ///< Vector of cameras
  vector<CameraPtr> cams;

  ///< Opencv video writer objects for top and bottom cams
  #ifndef V6_CROSS_BUILD
  vector<cv::VideoWriter*> videoWriter;
  #endif
};
