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
#include "Utils/include/JsonUtils.h"
#include "Utils/include/PrintUtils.h"

#define JSON_ASSIGN_(logRoot, key, value) logRoot[#key] = value;

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
  ///< Here we copy data to image
  delete image;
  #else ///< While in this case, image is only pointing to v4l buffer
  ///< Stop streaming the video
  xioctl(fd, VIDIOC_STREAMOFF, &bufferType);

  ///< Remove all video buffers from memory
  for (size_t i = 0; i < nBuffers; ++i)
    v4l2_munmap(buffers[i].start, buffers[i].length);

  ///< Close the video device access
  v4l2_close(fd);
  #endif
}

template <typename T>
void Camera<T>::print() const
{
  #ifndef VISUALIZER_BUILD
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
  #endif
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
void Camera<T>::getSettings() {
  std::string configName;
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  configName = ConfigManager::getConfigDirPath() + "NaoqiCameraSettings.json";
  #else
  configName = ConfigManager::getConfigDirPath() + "CameraSettings.json";
  #endif
  #ifdef VISUALIZER_BUILD
  Json::Value root;
  try {
    using namespace std;
    ifstream config(configName, ifstream::binary);
    config >> root;
    root = root[name];
  } catch (Json::Exception& e) {
    LOG_EXCEPTION("Error while reading json configuration:\n\t" << configName << "\n" << e.what());
  }
  #else
  LOG_INFO("Using camera config: " << configName);
  auto root = JsonUtils::readJson(configName)[name];
  #endif
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  resolution = root["subscription"]["resolution"].asInt();
  #else
  width = root["subscription"]["width"].asInt();
  height = root["subscription"]["height"].asInt();
  #endif
  colorSpace = root["subscription"]["colorSpace"].asInt();
  fps = root["subscription"]["fps"].asInt();
  fovX = root["intrinsic"]["fovX"].asFloat();
  fovY = root["intrinsic"]["fovY"].asFloat();
  focalX = root["intrinsic"]["focalX"].asFloat();
  focalY = root["intrinsic"]["focalY"].asFloat();
  centerOffX = root["intrinsic"]["centerOffX"].asFloat();
  centerOffY = root["intrinsic"]["centerOffY"].asFloat();
  #ifndef VISUALIZER_BUILD
  distCoeffs.at<float>(0, 0) = root["distortion"]["a"].asFloat();
  distCoeffs.at<float>(0, 1) = root["distortion"]["b"].asFloat();
  distCoeffs.at<float>(0, 2) = root["distortion"]["c"].asFloat();
  distCoeffs.at<float>(0, 3) = root["distortion"]["d"].asFloat();
  distCoeffs.at<float>(0, 4) = root["distortion"]["e"].asFloat();
  #endif

  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  settings.autoExposition = root["calibration"]["autoExposition"].asInt();
  settings.autoWhiteBalance = root["calibration"]["autoWhiteBalance"].asInt();
  settings.brightness = root["calibration"]["brightness"].asInt();
  settings.contrast = root["calibration"]["contrast"].asInt();
  settings.saturation = root["calibration"]["saturation"].asInt();
  settings.exposure = root["calibration"]["exposure"].asInt();
  settings.hue = root["calibration"]["hue"].asInt();
  settings.gain = root["calibration"]["gain"].asInt();
  settings.exposureAlgorithm = root["calibration"]["exposureAlgorithm"].asInt();
  settings.sharpness = root["calibration"]["sharpness"].asInt();
  settings.whiteBalance = root["calibration"]["whiteBalance"].asInt();
  settings.backlightCompensation = root["calibration"]["backlightCompensation"].asInt();
  #else
  for (size_t i = 0; i < toUType(CameraSettings::count); ++i) {
    settings[i].ctrl.value = root["calibration"][settings[i].name].asInt();
  }
  #endif
}

#ifndef NAOQI_VIDEO_PROXY_AVAILABLE
template <typename T>
void Camera<T>::setControl(const CameraSettings& id) {
  if (settings[toUType(id)].ctrl.value == -1000)
    return;
  cout <<
    "Setting control:" <<
    settings[toUType(id)].name <<
    ": " <<
    settings[toUType(id)].ctrl.value << endl;
  if (id == CameraSettings::whiteBalance) {
    ///< Only changeable when wb is disabled
    if (settings[toUType(CameraSettings::autoWhiteBalance)].ctrl.value == 0)
      xioctl(fd, VIDIOC_S_CTRL, &settings[toUType(id)].ctrl);
  } else if (
    id == CameraSettings::exposure ||
    id == CameraSettings::gain)
  {
    ///< Only changeable when ae is disabled
      if (settings[toUType(CameraSettings::autoExposition)].ctrl.value == 0)
        xioctl(fd, VIDIOC_S_CTRL, &settings[toUType(id)].ctrl);
  } else if (id == CameraSettings::backlightCompensation) {
    ///< Only changeable when ae is enabled
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
  string devName = name == "visionTop" ? "/dev/video0" : "/dev/video1";
  fd = v4l2_open(devName.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (fd < 0) {
    #ifndef VISUALIZER_BUILD
    LOG_ERROR("Cannot open video device: " << name);
    #endif
    return false;
  }
  CLEAR(fmt);
  bufferType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ///< Set up the video capture settings
  fmt.type = bufferType;
  fmt.fmt.pix.width       = width;
  fmt.fmt.pix.height      = height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

  parm.type = bufferType;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = fps;

  ///< VIDIOC_S_FMT -> Sets the FMT in camera driver
  xioctl(fd, VIDIOC_S_FMT, &fmt);

  ///< VIDIOC_S_PARM -> Sets stream parameter including video capture frame rate
  xioctl(fd, VIDIOC_S_PARM, &parm);

  ///< Set additional video capture settings
  setCameraControls();

  ///< Check if accepted pixel format is the requested pixel format
  if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
    #ifndef VISUALIZER_BUILD
    LOG_ERROR(
      "Libv4l didn't accept YUY2 format." <<
      "Failed to proceed to camera subscription.");
    #endif
    return false;
  }

  ///< Check if accepted resolution is the requested resolution
  if (
    (fmt.fmt.pix.width != width) ||
    (fmt.fmt.pix.height != height))
  {
    #ifndef VISUALIZER_BUILD
    LOG_ERROR(
      "Unable to subscribe at requested resolution. Driver is sending " <<
      "image at resolution: " <<
      fmt.fmt.pix.width << "x" <<
      fmt.fmt.pix.height);
    #endif
    return false;
  }

  ///< Request 2xbuffers for video capture
  CLEAR(req);
  req.count = nBuffers;
  req.type = bufferType;
  req.memory = V4L2_MEMORY_MMAP;
  xioctl(fd, VIDIOC_REQBUFS, &req);

  ///< Allocate memory for buffers
  buffers =  static_cast<buffer*>(calloc(req.count, sizeof(*buffers)));
  for (size_t i = 0; i < nBuffers; ++i) {
    ///< Clear the buffer
    CLEAR(buf);

    ///< Set up the buffer for memory capture
    buf.type = bufferType;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    ///< Send buffer request
    xioctl(fd, VIDIOC_QUERYBUF, &buf);

    ///< Get the length of buffer assigned in memory
    buffers[i].length = buf.length;
    ///< Get the start location of buffer in memory
    buffers[i].start =
      v4l2_mmap(
        NULL,
        buf.length,
        PROT_READ | PROT_WRITE, MAP_SHARED,
        fd,
        buf.m.offset);
    if (MAP_FAILED == buffers[nBuffers].start) {
      #ifndef VISUALIZER_BUILD
      LOG_ERROR("Unable to allocate buffers for video capture.");
      return false;
      #endif
    }
  }
  ///< Enqueue all requested buffers
  for (size_t i = 0; i < nBuffers; ++i) {
    CLEAR(buf);
    buf.type = bufferType;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    xioctl(fd, VIDIOC_QBUF, &buf);
  }

  ///< Set timeout value
  tv.tv_sec = 2;
  tv.tv_usec = 0;

  ///< Start streaming the video
  xioctl(fd, VIDIOC_STREAMON, &bufferType);
  #ifndef VISUALIZER_BUILD
  LOG_INFO("Camera " << name << " streaming is ON");
  #endif
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
    #ifndef VISUALIZER_BUILD
    LOG_ERROR("Unable to access image.")
    #endif
    return false;
  }

  ///< Clear the buffer
  CLEAR(buf);
  buf.type = bufferType;
  buf.memory = V4L2_MEMORY_MMAP;

  ///< Dequeue the video buffer from driver to this buffer
  xioctl(fd, VIDIOC_DQBUF, &buf);

  ///< Get the buffer index and obtain its location in memory
  image = reinterpret_cast<uint8_t*>(buffers[buf.index].start);

  ///< Enqueue the buffer for next frame
  xioctl(fd, VIDIOC_QBUF, &buf);
  return true;
}
#endif
template struct Camera<float>;
template struct Camera<double>;
