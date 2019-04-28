/**
 * @file VisionModule/src/CameraModule.cpp
 *
 * This file implements the class CameraModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#ifdef V6_CROSS_BUILD
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <sys/ioctl.h>
#include <stdint.h>
#endif
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/DataHolders/Camera.h"
#include "Utils/include/EnumUtils.h"
#include "Utils/include/FileUtils.h"
#include "Utils/include/VisionUtils.h"
#include "VisionModule/include/CameraModule.h"
#include "VisionModule/include/VisionModule.h"

#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
CameraModule::CameraModule(VisionModule* visionModule) :
  camProxy(visionModule->getCamProxy())
{
  #ifndef V6_CROSS_BUILD
    if (camProxy) {
      if (setupCameras())
        LOG_INFO("Cameras setup complete.")
    } else {
      LOG_ERROR("Could not get access to ALVideoDeviceProxy.")
    }
  #else
  if (setupCameras()) {
    LOG_INFO("Cameras setup complete.")
  } else {
    LOG_ERROR("Cameras not setup properly.")
  }
  #endif
}
#else
CameraModule::CameraModule() :
  DebugBase("CameraModule", this)
{
  initDebugBase();
  SET_DVAR(
    vector<int>,
    settingParams,
    vector<int>(toUType(CameraSettings::count) + 1, -1000)
  ); ///< First parameter is camera index
  if (setupCameras())
  LOG_INFO("Cameras setup complete.")
  #ifdef V6_CROSS_BUILD //! Fix for upper camera flip in V6
  struct uvc_xu_control_query query;
  query.unit = 3; //! Always 3 as mentioned in documenation of nao-v6-hints.pdf
  query.selector = 12;
  query.query = UVC_SET_CUR;
  query.size = 2;
  uint8_t* data = new uint8_t[2];
  data[0] = 1;
  data[1] = 1;
  query.data = data;
  xioctl(cams[toUType(CameraId::headTop)]->fd, UVCIOC_CTRL_QUERY, &query);
  query.selector = 13;
  query.query = UVC_SET_CUR;
  query.size = 2;
  query.data = data;
  xioctl(cams[toUType(CameraId::headTop)]->fd, UVCIOC_CTRL_QUERY, &query);
  #endif
}
#endif

CameraModule::~CameraModule()
{
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  for (const auto& cam : cams) {
    #ifndef V6_CROSS_BUILD
      camProxy->unsubscribe(cam->clientName);
    #else
      camProxy.call<void>("unsubscribe", cam->clientName);
    #endif
  }
  #endif
}

bool CameraModule::setupCameras()
{
  try {
    cams = vector<CameraPtr>(toUType(CameraId::count));
    cams[toUType(CameraId::headTop)] = boost::shared_ptr<Camera<float> >(new Camera<float>("visionTop"));
    cams[toUType(CameraId::headBottom)] = boost::shared_ptr<Camera<float> >(new Camera<float>("visionBottom"));
    #ifndef V6_CROSS_BUILD
    videoWriter = vector<cv::VideoWriter*> (toUType(CameraId::count));
    #endif
    #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
      #ifndef V6_CROSS_BUILD
        camProxy->unsubscribe("visionTop_0");
        camProxy->unsubscribe("visionBottom_0");
      #else
        camProxy.call<void>("unsubscribe", "visionTop_0");
        camProxy.call<void>("unsubscribe", "visionBottom_0");
      #endif
    #endif
    for (size_t i = 0; i < toUType(CameraId::count); ++i) {
      #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
      cams[i]->getSettings();
      #ifndef V6_CROSS_BUILD
      AL::ALValue size = camProxy->resolutionToSizes(cams[i]->resolution);
      #else
      vector<int> size = camProxy.call<vector<int> >("resolutionToSizes", cams[i]->resolution);
      #endif
      cams[i]->width = (int) size[0];
      cams[i]->height = (int) size[1];
      #ifndef V6_CROSS_BUILD
        cams[i]->clientName =
          camProxy->subscribeCamera(cams[i]->name, (int)i, cams[i]->resolution, cams[i]->colorSpace, cams[i]->fps);
      #else
      cams[i]->clientName =
        camProxy.call<string>("subscribeCamera", cams[i]->name, (int)i, cams[i]->resolution, cams[i]->colorSpace, cams[i]->fps);
      #endif
      //cams[i]->focalX =
      //  (((float) size[0]) * 0.5) / tan(cams[i]->fovX * 0.5 * M_PI / 180);
      //cams[i]->focalY =
      //  (((float) size[1]) * 0.5) / tan(cams[i]->fovY * 0.5 * M_PI / 180);
      LOG_INFO("Subscribed Camera:" + cams[i]->clientName)
      cams[i]->image = new uint8_t[cams[i]->width * cams[i]->height * 2];
      #ifndef V6_CROSS_BUILD
      camProxy->setAllParametersToDefault(i);
      #else
      camProxy.call<void>("setAllParametersToDefault", i);
      #endif
        #ifndef V6_CROSS_BUILD
          if (cams[i]->settings.autoExposition != -1000)
            camProxy->setCameraParameter(cams[i]->clientName, AL::kCameraAutoExpositionID, cams[i]->settings.autoExposition);
          if (cams[i]->settings.autoWhiteBalance != -1000)
            camProxy->setCameraParameter(cams[i]->clientName, AL::kCameraAutoWhiteBalanceID, cams[i]->settings.autoWhiteBalance);
          if (cams[i]->settings.brightness != -1000)
            camProxy->setCameraParameter(cams[i]->clientName, AL::kCameraBrightnessID, cams[i]->settings.brightness);
          if (cams[i]->settings.contrast != -1000)
            camProxy->setCameraParameter(cams[i]->clientName, AL::kCameraContrastID, cams[i]->settings.contrast);
          if (cams[i]->settings.saturation != -1000)
            camProxy->setCameraParameter(cams[i]->clientName, AL::kCameraSaturationID, cams[i]->settings.saturation);
          if (cams[i]->settings.exposure != -1000)
            camProxy->setCameraParameter(cams[i]->clientName, AL::kCameraExposureID, cams[i]->settings.exposure);
          if (cams[i]->settings.hue != -1000)
            camProxy->setCameraParameter(cams[i]->clientName, AL::kCameraHueID, cams[i]->settings.hue);
          if (cams[i]->settings.gain != -1000)
            camProxy->setCameraParameter(cams[i]->clientName, AL::kCameraGainID, cams[i]->settings.gain);
          if (cams[i]->settings.exposureAlgorithm != -1000)
            camProxy->setCameraParameter(cams[i]->clientName, AL::kCameraExposureAlgorithmID, cams[i]->settings.exposureAlgorithm);
          if (cams[i]->settings.sharpness != -1000)
            camProxy->setCameraParameter(cams[i]->clientName, AL::kCameraSharpnessID, cams[i]->settings.sharpness);
          if (cams[i]->settings.whiteBalance != -1000)
            camProxy->setCameraParameter(cams[i]->clientName, AL::kCameraWhiteBalanceID, cams[i]->settings.whiteBalance);
          if (cams[i]->settings.backlightCompensation != -1000)
            camProxy->setCameraParameter(cams[i]->clientName, AL::kCameraBacklightCompensationID, cams[i]->settings.backlightCompensation);
        #else
          if (cams[i]->settings.autoExposition != -1000)
            camProxy.call<void>("setCameraParameter", cams[i]->clientName, AL::kCameraAutoExpositionID, cams[i]->settings.autoExposition);
          if (cams[i]->settings.autoWhiteBalance != -1000)
            camProxy.call<void>("setCameraParameter", cams[i]->clientName, AL::kCameraAutoWhiteBalanceID, cams[i]->settings.autoWhiteBalance);
          if (cams[i]->settings.brightness != -1000)
            camProxy.call<void>("setCameraParameter", cams[i]->clientName, AL::kCameraBrightnessID, cams[i]->settings.brightness);
          if (cams[i]->settings.contrast != -1000)
            camProxy.call<void>("setCameraParameter", cams[i]->clientName, AL::kCameraContrastID, cams[i]->settings.contrast);
          if (cams[i]->settings.saturation != -1000)
            camProxy.call<void>("setCameraParameter", cams[i]->clientName, AL::kCameraSaturationID, cams[i]->settings.saturation);
          if (cams[i]->settings.exposure != -1000)
            camProxy.call<void>("setCameraParameter", cams[i]->clientName, AL::kCameraExposureID, cams[i]->settings.exposure);
          if (cams[i]->settings.hue != -1000)
            camProxy.call<void>("setCameraParameter", cams[i]->clientName, AL::kCameraHueID, cams[i]->settings.hue);
          if (cams[i]->settings.gain != -1000)
            camProxy.call<void>("setCameraParameter", cams[i]->clientName, AL::kCameraGainID, cams[i]->settings.gain);
          if (cams[i]->settings.exposureAlgorithm != -1000)
            camProxy.call<void>("setCameraParameter", cams[i]->clientName, AL::kCameraExposureAlgorithmID, cams[i]->settings.exposureAlgorithm);
          if (cams[i]->settings.sharpness != -1000)
            camProxy.call<void>("setCameraParameter", cams[i]->clientName, AL::kCameraSharpnessID, cams[i]->settings.sharpness);
          if (cams[i]->settings.whiteBalance != -1000)
            camProxy.call<void>("setCameraParameter", cams[i]->clientName, AL::kCameraWhiteBalanceID, cams[i]->settings.whiteBalance);
          if (cams[i]->settings.backlightCompensation != -1000)
            camProxy.call<void>("setCameraParameter", cams[i]->clientName, AL::kCameraBacklightCompensationID, cams[i]->settings.backlightCompensation);
        #endif
      #else
      LOG_INFO("Initializing Camera: " << i);
      cams[i]->getSettings();
      LOG_INFO("Setting up device for Camera: " << i);
      cams[i]->setupDevice();
      #endif
    }
  } catch (const exception& e) {
    LOG_ERROR("Camera subscription not possible.")
    LOG_EXCEPTION(e.what())
    return false;
  }
  return true;
}

#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  #ifdef MODULE_IS_REMOTE
  void CameraModule::updateImage(
    const CameraId& index,
    const int& saveImages,
    const bool& useLoggedImages)
  {
    string cameraClient = cams[toUType(index)]->clientName;
    try {
      /**
       * This image accessing function takes too much time.
       * Need some better way of getting images from the robot, probably,
       * making our own buffers and accessing it in raw format directly
       * from the system.
       */
      if(!useLoggedImages) {
        //high_resolution_clock::time_point tEnd, tStart;
        //tStart = high_resolution_clock::now();
        #ifndef V6_CROSS_BUILD
          AL::ALValue img = camProxy->getImageRemote(cameraClient);
        #else
          auto img = camProxy.call<vector<int> >("getImageRemote", cameraClient);
        #endif
        //tEnd = high_resolution_clock::now();
        //duration<double> time_span = tEnd - tStart;
        //LOG_INFO("Time cam " << cameraClient << "  " << time_span.count());
        ///<int width = (int) img[0];
        ///<int height = (int) img[1];
        ///<int nbLayers = (int) img[2];
        ///<int colorSpace = (int) img[3];
        ///<image[4] is the number of seconds, image[5] the
        ///<number of microseconds.
        ///<The pointer to the image data and its size:
        /*Mat bgr = Mat(Size(640,480), CV_8UC3);
         memcpy (bgr.data, (uint8_t*) img[6].GetBinary(), img[6].getSize());
         VisionUtils::displayImage(bgr, "InputBgr");
         Mat yuv;
         cvtColor(bgr, yuv, COLOR_BGR2YUV);
         VisionUtils::displayImage(yuv, "InputYuv1");*/
        //cout << yuv.at<Vec3b>(0, 0) << endl;
        //cout << yuv.at<Vec3b>(1, 0) << endl;
        //cout << yuv.at<Vec3b>(2, 0) << endl;
        //VisionUtils::bgrToYuv422(cams[toUType(index)]->image, img, img.cols, img.rows);
        memcpy (cams[toUType(index)]->image, (uint8_t*) img[6].GetBinary(), img[6].getSize());
        if(saveImages) {
          string imageStr;
          if (index == CameraId::headTop) {
            imageStr = ConfigManager::getLogsDirPath() +
            "Images/Top/" + DataUtils::getCurrentDateTime() + ".jpg";
          } else {
            imageStr = ConfigManager::getLogsDirPath() +
            "Images/Bottom/" + DataUtils::getCurrentDateTime() + ".jpg";
          }
          cout << imageStr << endl;
          cv::Mat yuv = cv::Mat(Size(img[0], img[1]), CV_8UC2);
          yuv.data = cams[toUType(index)]->image;
          cv::Mat bgr;
          cvtColor(yuv, bgr, COLOR_YUV2BGR_YUY2);
          if (index == CameraId::headTop) {
            VisionUtils::displayImage("Top", bgr);
          } else {
            VisionUtils::displayImage("Bottom", bgr);
          }
          imwrite(imageStr, bgr);
        }
        #ifndef V6_CROSS_BUILD
          camProxy->releaseImage(cameraClient);
        #else
          camProxy.call<void>("releaseImage", cameraClient);
        #endif
      } else {
        string name;
        string imageStr;
        if (index == CameraId::headTop) {
          GET_CONFIG(
            "VisionConfig",
            (string, InputImage.testImageTop, name),
          )
          imageStr = ConfigManager::getLogsDirPath() +
          "Images/Top/" + name;
        } else {
          GET_CONFIG(
            "VisionConfig",
            (string, InputImage.testImageBot, name),
          )
          imageStr = ConfigManager::getLogsDirPath() +
          "Images/Bottom/" + name;
        }
        cout << "reading image" << imageStr << endl;
        cv::Mat img = imread(imageStr);
        //Mat yuv;
        //cvtColor(img, yuv, COLOR_RGB2YUV);
        //VisionUtils::displayImage(yuv, "InputYuv1");
        //cout << yuv.at<Vec3b>(0, 0) << endl;
        //cout << yuv.at<Vec3b>(1, 0) << endl;
        //cout << yuv.at<Vec3b>(2, 0) << endl;
        bgrToYuv422(cams[toUType(index)]->image, img, img.cols, img.rows);
        //VisionUtils::displayImage(cams[toUType(index)]->image, "cams[toUType(index)]->image");
      }
    } catch (const exception& e) {
      LOG_EXCEPTION(e.what())
    }
  }
  #else
  void CameraModule::updateImage(
    const CameraId& index)
  {
    string cameraClient = cams[toUType(index)]->clientName;
    try {
      AL::ALImage* img;
      #ifndef V6_CROSS_BUILD
      img = (AL::ALImage*) camProxy->getImageLocal(cameraClient);
      #else
      img = (AL::ALImage*) camProxy.call<AL::ALImage*>("getImageLocal", cameraClient);
      #endif
      memcpy(cams[toUType(index)]->image, (uint8_t*) img->getData(), img->getSize());
      #ifndef V6_CROSS_BUILD
      camProxy->releaseImage(cameraClient);
      #else
      camProxy.call<void>("releaseImage", cameraClient);
      #endif
    } catch (const exception& e) {
      LOG_EXCEPTION("Exception raised in CameraModule:\n\t" << e.what())
    }
  }
  #endif
#else
  #ifdef MODULE_IS_REMOTE
  void CameraModule::updateImage(
    const CameraId& index,
    const int& saveImages,
    const bool& useLoggedImages)
  {
    LOG_ERROR("No implementation of remote access to camera without Naoqi's AL::ALVideoDeviceProxy.")
  }
  #else
  void CameraModule::updateImage(
    const CameraId& index)
  {
    static auto prevSettings = GET_DVAR(vector<int>, settingParams);
    auto currSettings = GET_DVAR(vector<int>, settingParams);
    if (currSettings != prevSettings) {
      for (size_t i = 0; i < toUType(CameraSettings::count); ++i) {
        cams[currSettings[0]]->setCameraSetting(static_cast<CameraSettings>(i), currSettings[i+1]);
      }
    }
    cams[toUType(index)]->getImageFromDevice();
    prevSettings = currSettings;
  }
  #endif
#endif

void CameraModule::releaseImage(const CameraId& index)
{
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  string cameraClient = cams[toUType(index)]->clientName;
  #ifndef V6_CROSS_BUILD
  camProxy->releaseImage(cameraClient);
  #else
  camProxy.call<void>("releaseImage", cameraClient);
  #endif
  #endif
}

void CameraModule::recordVideo(const CameraId& index)
{
  #ifndef V6_CROSS_BUILD
  static string logsDir = ConfigManager::getLogsDirPath();
  ///< VideoWriter object;
  if (!videoWriter[toUType(index)]) {
    try {
      string vidName;
      if (index == CameraId::headTop) {
        vidName = logsDir + "Video/Top/";
      } else {
        vidName = logsDir + "Video/Bottom/";
      }
      int cnt = FileUtils::getFileCnt(vidName);
      vidName += "vid-";
      vidName += DataUtils::varToString(cnt+1);
      vidName += ".avi";
      videoWriter[toUType(index)] = new VideoWriter();
      videoWriter[toUType(index)]->open(
        vidName,
        CV_FOURCC('M', 'P', 'E', 'G'),
        (int)cams[toUType(index)]->fps,
        Size((int)cams[toUType(index)]->width, (int)cams[toUType(index)]->height)
      );
      if (!videoWriter[toUType(index)]->isOpened()) {
        delete videoWriter[toUType(index)];
        throw "Unable to open video: \n\t" + vidName + " for write.";
      }
    } catch (exception &e) {
      LOG_EXCEPTION(e.what());
    }
  } else {
    cv::Mat yuv422 =
      cv::Mat(
        Size((int)cams[toUType(index)]->width, (int)cams[toUType(index)]->height),
        CV_8UC2
      );
    yuv422.data = cams[toUType(index)]->image;
    cv::Mat bgr;
    cvtColor(yuv422, bgr, COLOR_YUV2BGR_YUY2);
    videoWriter[toUType(index)]->write(bgr);
  }
  #else
  LOG_INFO("Video writer not yet support for V6 robots.")
  #endif
}

void CameraModule::stopRecording()
{
  #ifndef V6_CROSS_BUILD
  for (int i = 0; i < videoWriter.size(); ++i) {
    if (videoWriter[i]) {
      videoWriter[i]->release();
      delete videoWriter[i];
    }
  }
  #endif
}

void CameraModule::bgrToYuv422(uint8_t* out, const cv::Mat& in, const int& width, const int& height)
{
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int r = in.at < cv::Vec3b > (y, x)[2];
      int g = in.at < cv::Vec3b > (y, x)[1];
      int b = in.at < cv::Vec3b > (y, x)[0];
      out[x * 2 + y * width * 2] = (uint8_t)(
        0.299 * r + 0.587 * g + 0.114 * b);
      if (x % 2 == 0) {
        out[1 + x * 2 + y * width * 2] = (uint8_t)(
          -0.169 * r - 0.331 * g + 0.499 * b + 128);
      } else {
        out[1 + x * 2 + y * width * 2] = (uint8_t)(
          0.498 * r - 0.419 * g - 0.0813 * b + 128);
      }
    }
  }
}

void CameraModule::yuv422ToBgr(cv::Mat& out, const uint8_t * const in, const int& width, const int& height)
{
  out = cv::Mat(Size(width, height), CV_8UC3);
  for (int py = 0; py < height; py++) {
    int cbLast = in[(0 + py * width) * 2 + 1] & 255;
    int crLast = in[(0 + py * width) * 2 + 3] & 255;
    for (int px = 0; px < width; px++) {
      int y = in[(px + py * width) * 2] & 255;
      if ((px & 1) == 0) {
        cbLast = in[(px + py * width) * 2 + 1] & 255;
      } else {
        crLast = in[(px + py * width) * 2 + 1] & 255;
      }
      int cb = cbLast;
      int cr = crLast;
      out.at < cv::Vec3b > (py, px)[2] = VisionUtils::clip(
        y + 1.402 * (cr - 128) + 2, 0.0, 255.0);
      out.at < cv::Vec3b > (py, px)[1] = VisionUtils::clip(
        y - 0.344 * (cb - 128) - 0.714 * (cr - 128), 0.0, 255.0);
      out.at < cv::Vec3b > (py, px)[0] = VisionUtils::clip(
        y + 1.772 * (cb - 128) + 2, 0.0, 255.0);
    }
  }
}
