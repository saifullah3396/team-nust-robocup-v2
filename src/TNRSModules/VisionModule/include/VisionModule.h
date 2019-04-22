/**
 * @file VisionModule/include/VisionModule.h
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
  #ifndef V6_CROSS_BUILD
    #include <alproxies/alvideodeviceproxy.h>
  #endif
#endif
#include "TNRSBase/include/DebugBase.h"
#include "TNRSBase/include/BaseIncludes.h"
#include "VisionModule/include/FeatureExtractionIds.h"
#include "Utils/include/EnumUtils.h"

class CameraModule;
class CameraTransform;
class ColorHandler;
class FeatureExtraction;
enum class TNColors : unsigned int;
enum class CameraId : unsigned int;
typedef boost::shared_ptr<CameraModule> CameraModulePtr;
typedef boost::shared_ptr<CameraTransform> CameraTransformPtr;
typedef boost::shared_ptr<ColorHandler> ColorHandlerPtr;
typedef boost::shared_ptr<FeatureExtraction> FeatureExtractionPtr;
#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  #ifndef V6_CROSS_BUILD
    typedef boost::shared_ptr<AL::ALVideoDeviceProxy> ALVideoDeviceProxyPtr;
  #endif
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
    ///< Option to enable any kind of debugging.
    (int, debug, 1),
    ///< Option to choose which output image should be sent for debugging.
    (int, debugImageIndex, 2),
    ///< Option to send a binarized image for a given color
    (int, sendBinaryImage, -1),
    ///< Option to enable known landmark info to be sent
    (int, sendKnownLandmarks, 1),
    ///< Option to enable unknown landmark info to be sent
    (int, sendUnknownLandmarks, 1),
  );

  DECLARE_INPUT_CONNECTOR(
    visionThreadPeriod,
    upperCamInFeet,
    lowerCamInFeet,
    planningState,
    gameData,
    robotOnSideLine,
    robotPose2D
  );
  DECLARE_OUTPUT_CONNECTOR(
    visionThreadTimeTaken,
    ballInfo,
    goalInfo,
    landmarksFound,
    obstaclesObs
  );

public:
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
    /**
     * Initializes the vision module with its thread.
     *
     * @param teamNUSTSPL pointer to base class which is in this case
     *   the TeamNUSTSPL class
     * @param camProxy: Pointer to NaoQi's camera proxy.
     */
    #ifndef V6_CROSS_BUILD
      VisionModule(void* teamNUSTSPL, const ALVideoDeviceProxyPtr& camProxy);
    #else
      VisionModule(void* teamNUSTSPL, const qi::AnyObject& camProxy);
    #endif
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
   * @brief init See BaseModule::init()
   */
  void init() final;

  /**
   * @brief mainRoutine See BaseModule::mainRoutine()
   */
  void mainRoutine() final;

  /**
   * @brief handleRequests See BaseModule::handleRequests()
   */
  void handleRequests() final;

  /**
   * @brief initMemoryConn See BaseModule::initMemoryConn()
   */
  void initMemoryConn() final;

  /**
   * @brief setThreadPeriod See BaseModule::setThreadPeriod()
   */
  void setThreadPeriod() final;

  /**
   * @brief setThreadTimeTaken See BaseModule::setThreadTimeTaken()
   */
  void setThreadTimeTaken() final;

  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
    #ifndef V6_CROSS_BUILD
      /**
       * Returns the pointer to the ALVideoDeviceProxy.
       *
       * @return Pointer to ALVideoDeviceProxyPtr
       */
      ALVideoDeviceProxyPtr getCamProxy()
        { return camProxy; }
    #else
      /**
       * Returns the pointer to the ALVideoDeviceProxy.
       *
       * @return Pointer to ALVideoDeviceProxyPtr
       */
      qi::AnyObject getCamProxy()
        { return camProxy; }
    #endif
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

  #ifndef MODULE_IS_REMOTE
  /**
   * @brief saveImages Saves bgr images if required
   */
  void saveImages();
  #endif

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

  ///< Runs the vision module if true
  bool runVision = {true};

  ///< Flag to use logged images or realtime images
  bool useLoggedImages = {false};

  ///< Project field on the images
  bool projectField = {false};

  ///< Field points in world coordinates
  vector<Point3f> fieldPoints;

  ///< Logs images for the respective camera in respective robot dir
  vector<bool> logImages;

  ///< Starts video writer for the respective camera if true
  vector<bool> writeVideo;

  ///< Pointer to a vector of feature extraction classes.
  vector<FeatureExtractionPtr> featureExt;

  ///< Current active camera
  CameraId activeCamera;

  ///< Pointer to CameraModule class.
  CameraModulePtr cameraModule;

  ///< Pointer to camera transform classes.
  vector<CameraTransformPtr> cameraTransforms;

  ///< Pointer to ColorHandler class.
  ColorHandlerPtr colorHandler;

  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  ///< Pointer to NaoQi video device proxy
    #ifndef V6_CROSS_BUILD
      ALVideoDeviceProxyPtr camProxy;
    #else
      qi::AnyObject camProxy;
    #endif
  #endif
};
