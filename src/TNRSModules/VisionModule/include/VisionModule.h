/**
 * @file VisionModule/VisionModule.h
 *
 * This file declares a class for vision planning.
 * All the functions and algorithms for image processing, object
 * detection and classification, and colorspaces will be defined
 * under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <Eigen/Dense>
#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
#include <alproxies/alvideodeviceproxy.h>
#endif
#include "TNRSBase/include/DebugBase.h"
#include "TNRSBase/include/BaseIncludes.h"
#include "VisionModule/include/FeatureExtractionIds.h"
#include "Utils/include/EnumUtils.h"

class CameraModule;
class CameraTransform;
class ColorHandler;
class FeatureExtraction;
typedef boost::shared_ptr<CameraModule> CameraModulePtr;
typedef boost::shared_ptr<CameraTransform> CameraTransformPtr;
typedef boost::shared_ptr<ColorHandler> ColorHandlerPtr;
typedef boost::shared_ptr<FeatureExtraction> FeatureExtractionPtr;
#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
typedef boost::shared_ptr<AL::ALVideoDeviceProxy> ALVideoDeviceProxyPtr;
#endif

/**
 * @class VisionModule
 * @brief A class for vision and feature extraction. All the functions
 *   and algorithms for image processing, object detection and
 *   classification, and colorspaces will be defined under this module.
 */
class VisionModule : public BaseModule, public DebugBase
{
  /**
   * Debug variables for this module
   */ 
  INIT_DEBUG_BASE_(
    //! Option to choose which output image should be sent for debugging.
    (int, debug, 1),
    //! Option to enable any kind of debugging.
    (int, debugImageIndex, 2),
    //! Option to enable known landmark info to be sent
    (int, sendKnownLandmarks, 1),
    //! Option to enable unknown landmark info to be sent
    (int, sendUnknownLandmarks, 1),
  )

  DECLARE_INPUT_CONNECTOR(
    visionThreadPeriod,
    upperCamInFeet,
    lowerCamInFeet,
    planningState,
    gameData,
    robotOnSideLine,
    robotPose2D
  )
  DECLARE_OUTPUT_CONNECTOR(
    ballInfo,
    goalInfo,
    landmarksFound,
    obstaclesObs
  )

public:
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  /**
   * Initializes the vision module with its thread.
   *
   * @param teamNUSTSPL pointer to base class which is in this case
   *   the TeamNUSTSPL class
   * @param camProxy: Pointer to NaoQi's camera proxy.
   */
  VisionModule(void* teamNUSTSPL, const ALVideoDeviceProxyPtr& camProxy);
  #else
  /**
   * @brief UserCommModule Constructor
   * @param teamNUSTSPL pointer to base class which is in this case
   *   the TeamNUSTSPL class
   */
  VisionModule(void* teamNUSTSPL);
  #endif

  /**
   * Defualt destructor for this class.
   */
  ~VisionModule()
  {
    delete inputConnector;
    delete outputConnector;
  }

  /**
   * Derived from BaseModule
   */
  void
  init();

  /**
   * Derived from BaseModule
   */
  void
  handleRequests();

  /**
   * Derived from BaseModule
   */
  void
  mainRoutine();

  /**
   * Derived from BaseModule
   */
  void
  initMemoryConn();

  /**
   * Derived from BaseModule
   */
  void
  setThreadPeriod();

  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  /**
   * Returns the pointer to the ALVideoDeviceProxyPtr.
   *
   * @return Pointer to ALVideoDeviceProxyPtr
   */
  ALVideoDeviceProxyPtr
  getCamProxy()
  {
    return camProxy;
  }
  #endif

  /**
   * Returns the pointer to the CameraModule class.
   *
   * @return Pointer to CameraModule
   */
  CameraModulePtr
  getCameraModule()
  {
    return cameraModule;
  }

  /**
   * Returns the pointer to the required feature extraction class.
   *
   * @return Pointer to FeatureExtraction
   */
  FeatureExtractionPtr getFeatureExtClass(const FeatureExtractionIds& index)
  {
    return featureExt[toUType(index)];
  }

  /**
   * Returns the pointer to the image transform class.
   *
   * @return Pointer to CameraTransform
   */
  CameraTransformPtr
  getCameraTransform(const unsigned& index)
  {
    return cameraTransforms[index];
  }

  /**
   * Returns the pointer to the image transform class.
   *
   * @return Pointer to CameraTransform
   */
  vector<CameraTransformPtr>
  getCameraTransforms()
  {
    return cameraTransforms;
  }

  /**
   * Returns the pointer to the ColorHandler class.
   *
   * @return Pointer to ColorHandler
   */
  ColorHandlerPtr
  getColorHandler()
  {
    return colorHandler;
  }

private:
  /**
   * @brief cameraUpdate updates images
   */
  void cameraUpdate();

  /**
   * @brief handleVideoWriting writes a video if required
   */
  void handleVideoWriting();

  /**
   * @brief featureExtractionUpdate updates feature extraction classes
   * @return true if images have been processed
   */
  bool featureExtractionUpdate();

  /**
   * @brief updateLandmarksInfo send landmarks to localization module
   */
  void updateLandmarksInfo();

  /**
   * @brief setupFeatureExtraction Sets up Feature extraction base class and child classes
   */
  void setupFeatureExtraction();

  /**
   * @brief sendUnknownLandmarksInfo Sends unknown landmarks info to user through communication
   */
  void sendUnknownLandmarksInfo();

  /**
   * @brief sendKnownLandmarksInfo Sends known landmarks info to user through communication
   */
  void sendKnownLandmarksInfo();

  /**
   * @brief sendImages Sends images to user through communication
   */
  void sendImages();

  /**
   * Sets up the field points and projects them on the field
   */ 
  void setupFieldProjection();

  //! Runs the vision module if true
  bool runVision = {false};

  //! Flag to use logged images or realtime images
  bool useLoggedImages = {false};

  //! Project field on the images
  bool projectField = {false};

  //! Field points in world coordinates
  vector<Point3f> fieldPoints;

  //! Logs images for the respective camera in respective robot dir
  vector<bool> logImages;

  //! Starts video writer for the respective camera if true
  vector<bool> writeVideo;

  //! Pointer to a vector of feature extraction classes.
  vector<FeatureExtractionPtr> featureExt;

  //! A vector determining whether to update a given feature extraction class or not
  vector<bool> featureExtToRun;

  //! Pointer to CameraModule class.
  CameraModulePtr cameraModule;

  //! Pointer to camera transform classes.
  vector<CameraTransformPtr> cameraTransforms;

  //! Pointer to ColorHandler class.
  ColorHandlerPtr colorHandler;

  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  //! Pointer to NaoQi video device proxy
  ALVideoDeviceProxyPtr camProxy;
  #endif
};
