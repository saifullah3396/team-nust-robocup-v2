/**
 * @file Utils/src/DataHolders/Camera.cpp
 *
 * This file implements the struct Camera
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 18 Feb 2017
 */

#include <iostream>
#include "Utils/include/DataHolders/Camera.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/PrintUtils.h"

#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
template <typename T>
Camera<T>::Camera(const string& name) :
  name(name)
{
}
#else
DEFINE_SPECIALIZED_ENUM(CameraSettings);
template <typename T>
Camera<T>::Camera(const string& name) :
  name(name)
{
  settings.resize(toUType(CameraSettings::count));
  settings[toUType(CameraSettings::autoExposition)] =
    V4L2Settings("autoExposition", V4L2_CID_EXPOSURE_AUTO, 0, 0, 3);
  settings[toUType(CameraSettings::autoWhiteBalance)] =
    V4L2Settings("autoWhiteBalance", V4L2_CID_AUTO_WHITE_BALANCE, 1, 0, 1);
  settings[toUType(CameraSettings::backlightCompensation)] =
    V4L2Settings("backlightCompensation", V4L2_CID_BACKLIGHT_COMPENSATION, 0, 0, 4);
  settings[toUType(CameraSettings::brightness)] =
    V4L2Settings("brightness", V4L2_CID_BRIGHTNESS, 0, 0, 255);
  settings[toUType(CameraSettings::contrast)] =
    V4L2Settings("contrast", V4L2_CID_CONTRAST, 32, 16, 64);
  settings[toUType(CameraSettings::exposure)] =
    V4L2Settings("exposure", V4L2_CID_EXPOSURE, 175, 1, 1000);
  settings[toUType(CameraSettings::exposureAlgorithm)] =
     V4L2Settings("exposureAlgorithm", V4L2_CID_EXPOSURE_ALGORITHM, 0, 0, 3);
  settings[toUType(CameraSettings::gain)] =
    V4L2Settings("gain", V4L2_CID_GAIN, 255, 0, 255);
  settings[toUType(CameraSettings::gamma)] =
     V4L2Settings("gamma", V4L2_CID_GAMMA, 220, 0, 1000);
  settings[toUType(CameraSettings::hue)] =
    V4L2Settings("hue", V4L2_CID_HUE, 0, -22, 22);
  settings[toUType(CameraSettings::freq)] =
     V4L2Settings("freq", V4L2_CID_POWER_LINE_FREQUENCY, 1, 1, 2);
  settings[toUType(CameraSettings::saturation)] =
    V4L2Settings("saturation", V4L2_CID_SATURATION, 160, 0, 255);
  settings[toUType(CameraSettings::sharpness)] =
    V4L2Settings("sharpness", V4L2_CID_SHARPNESS, 1, -7, 7);
  settings[toUType(CameraSettings::whiteBalance)] =
    V4L2Settings("whiteBalance", V4L2_CID_WHITE_BALANCE_TEMPERATURE, 5800, 2700, 6500);
  settings[toUType(CameraSettings::fadeToBlack)] =
    V4L2Settings("fadeToBlack", V4L2_MT9M114_FADE_TO_BLACK, 0, 0, 1);
  settings[toUType(CameraSettings::horizontalFlip)] =
    V4L2Settings("horizontalFlip", V4L2_CID_HFLIP, 1, 0, 1);
  settings[toUType(CameraSettings::verticalFlip)] =
    V4L2Settings("verticalFlip", V4L2_CID_VFLIP, 1, 0, 1);
  //settings[toUType(CameraSettings::brightnessDark)] =
  //  V4L2Settings("brightnessDark", V4L2_MT9M114_BRIGHTNESS_DARK, 27, 0, 255);
  //settings[toUType(CameraSettings::aeTargetGain)] =
  //  V4L2Settings("aeTargetGain", V4L2_MT9M114_AE_TARGET_GAIN, 128, 0, 65535);
  //settings[toUType(CameraSettings::minVirtAnalogGain)] =
  //  V4L2Settings("minVirtAnalogGain", V4L2_MT9M114_AE_MIN_VIRT_AGAIN, 32, 0, 65535);
  //settings[toUType(CameraSettings::maxVirtAnalogGain)] =
  //  V4L2Settings("maxVirtAnalogGain", V4L2_MT9M114_AE_MAX_VIRT_AGAIN, 256, 0, 65535);
  //settings[toUType(CameraSettings::minDigAnalogGain)] =
  //  V4L2Settings("minDigAnalogGain", V4L2_MT9M114_AE_MIN_VIRT_DGAIN, 128, 0, 65535);
  //settings[toUType(CameraSettings::maxDigAnalogGain)] =
  //  V4L2Settings("maxDigAnalogGain", V4L2_MT9M114_AE_MAX_VIRT_DGAIN, 132, 0, 65535);
  //settings[toUType(CameraSettings::aeWeightTable)] =
  //  V4L2Settings("verticalFlip", V4L2_MT9M114_AE_WEIGHT_TABLE_0_0, 0, 128, 65535);
}
#endif

template <typename T>
Camera<T>::~Camera() {
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  //! Here we copy data to image
  delete image;
  #else //! While in this case, image is only pointing to v4l buffer
  //! Stop streaming the video
  xioctl(fd, VIDIOC_STREAMOFF, &bufferType);

  //! Remove all video buffers from memory
  for (size_t i = 0; i < nBuffers; ++i)
    v4l2_munmap(buffers[i].start, buffers[i].length);

  //! Close the video device access
  v4l2_close(fd);
  #endif
}

template <typename T>
void Camera<T>::print() const
{
  PRINT_DATA(
    Camera,
    (name, name),
    #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
    (clientName, clientName),
    (resolution, (int)resolution),
    #endif
    (colorSpace, (int)colorSpace),
    (fps, (int)fps),
    (fovX, fovX),
    (fovY, fovY),
    (focalX, focalX),
    (focalY, focalY),
    (centerOffX, centerOffX),
    (centerOffY, centerOffY),
  );
}

template <typename T>
Json::Value Camera<T>::getJson() const
{
  Json::Value val;
  JSON_ASSIGN_(val, name, name);
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  JSON_ASSIGN_(val, clientName, clientName);
  JSON_ASSIGN_(val, resolution, (int)resolution);
  #endif
  JSON_ASSIGN_(val, colorSpace, (int)colorSpace);
  JSON_ASSIGN_(val, fps, (int)fps);
  JSON_ASSIGN_(val, fovX, fovX);
  JSON_ASSIGN_(val, fovY, fovY);
  JSON_ASSIGN_(val, focalX, focalX);
  JSON_ASSIGN_(val, focalY, focalY);
  JSON_ASSIGN_(val, centerOffX, centerOffX);
  JSON_ASSIGN_(val, centerOffY, centerOffY);
  return val;
}

template <typename T>
void Camera<T>::getSettings(const string& configFile) {
  T da, db, dc, dd, de;
  GET_CONFIG(
    configFile,
    (string, Subscription.cameraName, name),
    #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
    (int, Subscription.resolution, resolution),
    #else
    (int, Subscription.width, width),
    (int, Subscription.height, height),
    #endif
    (int, Subscription.colorSpace, colorSpace),
    (int, Subscription.fps, fps),
    (T, Intrinsic.fovX, fovX),
    (T, Intrinsic.fovY, fovY),
    (T, Intrinsic.focalX, focalX),
    (T, Intrinsic.focalY, focalY),
    (T, Intrinsic.centerOffX, centerOffX),
    (T, Intrinsic.centerOffY, centerOffY),
    (T, Distortion.a, da),
    (T, Distortion.b, db),
    (T, Distortion.c, dc),
    (T, Distortion.d, dd),
    (T, Distortion.e, de),
    #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
    (int, Calibration.autoExposition, settings.autoExposition),
    (int, Calibration.autoWhiteBalance, settings.autoWhiteBalance),
    (int, Calibration.brightness, settings.brightness),
    (int, Calibration.contrast, settings.contrast),
    (int, Calibration.saturation, settings.saturation),
    (int, Calibration.exposure, settings.exposure),
    (int, Calibration.hue, settings.hue),
    (int, Calibration.gain, settings.gain),
    (int, Calibration.exposureAlgorithm, settings.exposureAlgorithm),
    (int, Calibration.sharpness, settings.sharpness),
    (int, Calibration.whiteBalance, settings.whiteBalance),
    (int, Calibration.backlightCompensation, settings.backlightCompensation),
    #else
    (int, Calibration.autoExposition, settings[toUType(CameraSettings::autoExposition)].ctrl.value),
    (int, Calibration.autoWhiteBalance, settings[toUType(CameraSettings::autoWhiteBalance)].ctrl.value),
    (int, Calibration.brightness, settings[toUType(CameraSettings::brightness)].ctrl.value),
    (int, Calibration.contrast, settings[toUType(CameraSettings::contrast)].ctrl.value),
    (int, Calibration.saturation, settings[toUType(CameraSettings::saturation)].ctrl.value),
    (int, Calibration.exposure, settings[toUType(CameraSettings::exposure)].ctrl.value),
    (int, Calibration.hue, settings[toUType(CameraSettings::hue)].ctrl.value),
    (int, Calibration.gain, settings[toUType(CameraSettings::gain)].ctrl.value),
    (int, Calibration.exposureAlgorithm, settings[toUType(CameraSettings::exposureAlgorithm)].ctrl.value),
    (int, Calibration.sharpness, settings[toUType(CameraSettings::sharpness)].ctrl.value),
    (int, Calibration.whiteBalance, settings[toUType(CameraSettings::whiteBalance)].ctrl.value),
    (int, Calibration.backlightCompensation, settings[toUType(CameraSettings::backlightCompensation)].ctrl.value),
    (int, Calibration.verticalFlip, settings[toUType(CameraSettings::verticalFlip)].ctrl.value),
    (int, Calibration.horizontalFlip, settings[toUType(CameraSettings::horizontalFlip)].ctrl.value),
    //(int, Calibration.aeTargetGain, settings[toUType(CameraSettings::aeTargetGain)].ctrl.value),
    //(int, Calibration.minVirtAnalogGain, settings[toUType(CameraSettings::minVirtAnalogGain)].ctrl.value),
    //(int, Calibration.maxVirtAnalogGain, settings[toUType(CameraSettings::maxVirtAnalogGain)].ctrl.value),
    //(int, Calibration.minDigAnalogGain, settings[toUType(CameraSettings::minDigAnalogGain)].ctrl.value),
    //(int, Calibration.maxDigAnalogGain, settings[toUType(CameraSettings::maxDigAnalogGain)].ctrl.value),
    #endif
  );
  distCoeffs.at<float>(0, 0) = da;
  distCoeffs.at<float>(0, 1) = db;
  distCoeffs.at<float>(0, 2) = dc;
  distCoeffs.at<float>(0, 3) = dd;
  distCoeffs.at<float>(0, 4) = de;
}

#ifndef NAOQI_VIDEO_PROXY_AVAILABLE
template <typename T>
void Camera<T>::setControl(const CameraSettings& id) {
  if (settings[toUType(id)].ctrl.value == -1000)
    return;
  //LOG_INFO(
  //  "Setting control:" <<
  //  settings[toUType(id)].name <<
  //  ": " <<
  //  settings[toUType(id)].ctrl.value);
  if (id == CameraSettings::whiteBalance) {
    //! Only changeable when wb is disabled
    if (settings[toUType(CameraSettings::autoWhiteBalance)].ctrl.value == 0)
      xioctl(fd, VIDIOC_S_CTRL, &settings[toUType(id)].ctrl);
  } else if (
    id == CameraSettings::exposure ||
    id == CameraSettings::gain)
  {
    //! Only changeable when ae is disabled
      if (settings[toUType(CameraSettings::autoExposition)].ctrl.value == 0)
        xioctl(fd, VIDIOC_S_CTRL, &settings[toUType(id)].ctrl);
  } else if (id == CameraSettings::backlightCompensation) {
    //! Only changeable when ae is enabled
      if (settings[toUType(CameraSettings::autoExposition)].ctrl.value != 0)
        xioctl(fd, VIDIOC_S_CTRL, &settings[toUType(id)].ctrl);
  } else {
    xioctl(fd, VIDIOC_S_CTRL, &settings[toUType(id)].ctrl);
  }
}

template <typename T>
void Camera<T>::setCameraSetting(const CameraSettings& id, const int& value)
{
  settings[toUType(id)].ctrl.value = value;
  setControl(id);
}

template <typename T>
void Camera<T>::setCameraControls() {
  for (const auto& setting : CameraSettings())
    setControl(setting);
}

template <typename T>
bool Camera<T>::setupDevice() {
  fd = v4l2_open(name.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (fd < 0) {
    LOG_ERROR("Cannot open video device: " << name);
    return false;
  }
  CLEAR(fmt);
  bufferType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  //! Set up the video capture settings
  fmt.type = bufferType;
  fmt.fmt.pix.width       = width;
  fmt.fmt.pix.height      = height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

  parm.type = bufferType;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = fps;

  //! VIDIOC_S_FMT -> Sets the FMT in camera driver
  xioctl(fd, VIDIOC_S_FMT, &fmt);

  //! VIDIOC_S_PARM -> Sets stream parameter including video capture frame rate
  xioctl(fd, VIDIOC_S_PARM, &parm);

  //! Set additional video capture settings
  setCameraControls();

  //! Check if accepted pixel format is the requested pixel format
  if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
    LOG_ERROR(
      "Libv4l didn't accept YUY2 format." <<
      "Failed to proceed to camera subscription.");
    return false;
  }

  //! Check if accepted resolution is the requested resolution
  if (
    (fmt.fmt.pix.width != width) ||
    (fmt.fmt.pix.height != height))
  {
    LOG_ERROR(
      "Unable to subscribe at requested resolution. Driver is sending " <<
      "image at resolution: " <<
      fmt.fmt.pix.width << "x" <<
      fmt.fmt.pix.height);
    return false;
  }

  //! Request 2xbuffers for video capture
  CLEAR(req);
  req.count = nBuffers;
  req.type = bufferType;
  req.memory = V4L2_MEMORY_MMAP;
  xioctl(fd, VIDIOC_REQBUFS, &req);

  //! Allocate memory for buffers
  buffers =  static_cast<buffer*>(calloc(req.count, sizeof(*buffers)));
  for (size_t i = 0; i < nBuffers; ++i) {
    //! Clear the buffer
    CLEAR(buf);

    //! Set up the buffer for memory capture
    buf.type = bufferType;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    //! Send buffer request
    xioctl(fd, VIDIOC_QUERYBUF, &buf);

    //! Get the length of buffer assigned in memory
    buffers[i].length = buf.length;
    //! Get the start location of buffer in memory
    buffers[i].start =
      v4l2_mmap(
        NULL,
        buf.length,
        PROT_READ | PROT_WRITE, MAP_SHARED,
        fd,
        buf.m.offset);
    if (MAP_FAILED == buffers[nBuffers].start) {
      LOG_ERROR("Unable to allocate buffers for video capture.");
      return false;
    }
  }
  //! Enqueue all requested buffers
  for (size_t i = 0; i < nBuffers; ++i) {
    CLEAR(buf);
    buf.type = bufferType;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    xioctl(fd, VIDIOC_QBUF, &buf);
  }

  //! Set timeout value
  tv.tv_sec = 2;
  tv.tv_usec = 0;

  //! Start streaming the video
  xioctl(fd, VIDIOC_STREAMON, &bufferType);
  LOG_INFO("Camera " << name << " streaming is ON");
  return true;
}

template <typename T>
bool Camera<T>::getImageFromDevice()
{
  struct v4l2_buffer buf;
  int r;
  fd_set fds;
  do {
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    r = select(fd + 1, &fds, NULL, NULL, &tv);
  } while ((r == -1 && (errno == EINTR)));
  if (r == -1) {
    LOG_ERROR("Unable to access image.")
    return false;
  }

  //! Clear the buffer
  CLEAR(buf);
  buf.type = bufferType;
  buf.memory = V4L2_MEMORY_MMAP;

  //! Dequeue the video buffer from driver to this buffer
  xioctl(fd, VIDIOC_DQBUF, &buf);

  //! Get the buffer index and obtain its location in memory
  image = reinterpret_cast<uint8_t*>(buffers[buf.index].start);

  //! Enqueue the buffer for next frame
  xioctl(fd, VIDIOC_QBUF, &buf);
  return true;
}
#endif
template struct Camera<float>;
template struct Camera<double>;
