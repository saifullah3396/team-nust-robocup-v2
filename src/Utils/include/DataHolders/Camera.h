/**
 * @file Utils/includes/DataHolders/Camera.h
 *
 * This file defines the struct Camera
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#ifndef NAOQI_VIDEO_PROXY_AVAILABLE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <argp.h>
#include <pthread.h>
#else
#include "Utils/include/alvisiondefinitions.h"
#endif
#include <opencv2/core/core.hpp>
#include "Utils/include/DataHolders/DataHolder.h"
#include "Utils/include/EnumUtils.h"

namespace Json {
  class Value;
}

#ifndef NAOQI_VIDEO_PROXY_AVAILABLE
#define CLEAR_P(x, s) memset((x), 0, s)
#define CLEAR(x) CLEAR_P(&(x), sizeof(x))

//! According to bhuman-kernel for Nao robot
#define V4L2_MT9M114_FADE_TO_BLACK V4L2_CID_PRIVATE_BASE
#define V4L2_MT9M114_BRIGHTNESS_DARK (V4L2_CID_PRIVATE_BASE+1)
#define V4L2_MT9M114_AE_TARGET_GAIN (V4L2_CID_PRIVATE_BASE+2)
#define V4L2_MT9M114_AE_MIN_VIRT_AGAIN (V4L2_CID_PRIVATE_BASE+3)
#define V4L2_MT9M114_AE_MAX_VIRT_AGAIN (V4L2_CID_PRIVATE_BASE+4)
#define V4L2_MT9M114_AE_MIN_VIRT_DGAIN (V4L2_CID_PRIVATE_BASE+5)
#define V4L2_MT9M114_AE_MAX_VIRT_DGAIN (V4L2_CID_PRIVATE_BASE+6)
#define V4L2_MT9M114_AE_WEIGHT_TABLE_0_0 (V4L2_CID_PRIVATE_BASE+7)
#ifdef MODULE_IS_REMOTE
#define V4L2_CID_EXPOSURE_ALGORITHM (V4L2_CID_PRIVATE_BASE+8)
#endif

struct buffer
{
  void *start;
  size_t length;
};

static void xioctl(int fh, unsigned long int request, void *arg)
{
  int r;
  do {
    r = v4l2_ioctl(fh, request, arg);
  } while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

  if (r == -1) {
    LOG_EXCEPTION("Exception caught while setting camera control.")
    fprintf(stderr, "%s(%lu): error %d, %s\n", __func__,
      _IOC_NR(request), errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
}
#endif

#ifndef NAOQI_VIDEO_PROXY_AVAILABLE
struct V4L2Settings {
  V4L2Settings() = default;
  V4L2Settings(
    const string& name,
    const int& id,
    const int& value,
    const int& min,
    const int& max) :
    name(name),
    min(min),
    max(max)
  {
    ctrl.id = id;
    ctrl.value = value;
  }

  string name;
  struct v4l2_control ctrl;
  int min = {0};
  int max = {0};
};
#endif

#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
/**
 * @struct CameraSettings
 * @brief Holds information about internal camera settings
 */
struct CameraSettings
{
  int autoExposition = {0};
  int autoWhiteBalance = {1};
  int backlightCompensation = {0};
  int brightness = {0};
  int contrast = {32};
  int exposure = {125};
  int fadeToBlack = {0};
  int exposureAlgorithm = {0};
  int freq = {1};
  int gain = {160};
  int hue = {0};
  int saturation = {160};
  int sharpness = {2};
  int whiteBalance = {5800};
};
#else
/**
 * @brief Enumeration for internal camera settings
 *
 * @enum CameraSettings
 */
enum class CameraSettings : unsigned int
{
  autoExposition,
  autoWhiteBalance,
  backlightCompensation,
  brightness,
  contrast,
  exposure,
  exposureAlgorithm,
  gain,
  gamma,
  hue,
  freq,
  saturation,
  sharpness,
  whiteBalance,
  fadeToBlack,
  horizontalFlip,
  verticalFlip,
  //aeTargetGain,
  //minVirtAnalogGain,
  //maxVirtAnalogGain,
  //minDigAnalogGain,
  //maxDigAnalogGain,
  count,
  first = autoExposition,
  last = verticalFlip
};
DECLARE_SPECIALIZED_ENUM(CameraSettings);
#endif

/**
 * @struct Camera
 * @brief Holds information about a single camera
 */
template <typename T = float>
struct Camera : public DataHolder
{
  /**
   * @brief Camera Constructor
   * @param name Camera name
   */
  Camera(const string& name);

  /**
   * @brief ~Camera Destructor
   */
  ~Camera();

  /**
   * @brief print Self-explanatory
   */
  void print() const final;
  Json::Value getJson() const final;
  void getSettings(const string& configFile);
  #ifndef NAOQI_VIDEO_PROXY_AVAILABLE
  void setControl(const CameraSettings& id);
  void setCameraSetting(const CameraSettings& id, const int& value);
  void setCameraControls();
  bool setupDevice();
  bool getImageFromDevice();
  #endif

  string name = string{"None"}; //! Name of the camera
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  string clientName = string{"None"}; //! Camera subscribed client name
  uint8_t resolution = {0}; //! Resolution of the camera
  uint8_t colorSpace = {AL::kYUV422ColorSpace}; //! Colorspace definition of the camera
  #else
  unsigned colorSpace = {V4L2_PIX_FMT_RGB24}; //! Colorspace definition of the camera
  struct v4l2_format fmt;
  struct v4l2_streamparm parm;
  struct v4l2_buffer buf;
  struct v4l2_requestbuffers req;
  enum v4l2_buf_type bufferType;
  int fd = -1;
  unsigned int nBuffers = 2;
  struct buffer *buffers;
  struct timeval tv;
  #endif
  uint16_t fps = {30}; //! Frames per second for the video
  uint8_t* image = {nullptr}; //! Camera image
  uint32_t width = {0}; //! Image width
  uint32_t height = {0}; //! Image height
  T fovX = {0}; //! Camera field of view X
  T fovY = {0}; //! Camera field of view Y
  T focalX = {0}; //! Camera focal length X
  T focalY = {0};//! Camera focal length Y
  T centerOffX = {0}; //! Camera center offset X
  T centerOffY = {0}; //! Camera center offset Y
  cv::Mat distCoeffs = {cv::Mat_<float>(1, 5)}; //! Distortion coefficients
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
    CameraSettings settings;
  #else
  vector<V4L2Settings> settings;
  #endif
};

