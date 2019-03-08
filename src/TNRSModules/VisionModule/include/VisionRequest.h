/**
 * @file VisionModule/include/VisionRequest.h
 *
 * This file defines the class VisionRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "Utils/include/SwitchRequest.h"
#include "VisionModule/include/FeatureExtractionIds.h"

/**
 * Types of request valid for VisionModule
 * 
 * @enum VisionRequestIds
 */ 
enum class VisionRequestIds {
  switchVision,
  switchVideoWriter,
  switchFieldProjection,
  switchLogImages,
  switchUseLoggedImages,
  switchFeatureExtModule
};

/**
 * @class VisionRequest
 * @brief A module request that can be handled by VisionModule
 */ 
class VisionRequest : public ModuleRequest 
{
public:
  /**
   * Constructor
   * 
   * @param id: Id of the control request
   */ 
  VisionRequest(const VisionRequestIds& id) :
    ModuleRequest(toUType(TNSPLModules::vision), toUType(id))
  {
  }
};
typedef boost::shared_ptr<VisionRequest> VisionRequestPtr;

/**
 * @class SwitchVision
 * @brief A request to switch on and off the vision module
 */ 
struct SwitchVision : public VisionRequest, SwitchRequest
{
  /**
   * Constructor
   */ 
  SwitchVision(const bool& state) :
    VisionRequest(VisionRequestIds::switchVision),
    SwitchRequest(state)
  {
  }
};
typedef boost::shared_ptr<SwitchVision> SwitchVisionPtr;

/**
 * @class SwitchVideoWriter
 * @brief A request to switch on and off the video writing process
 */ 
struct SwitchVideoWriter : public VisionRequest, SwitchRequest
{
  /**
   * Constructor
   */ 
  SwitchVideoWriter(const bool& state) :
    VisionRequest(VisionRequestIds::switchVideoWriter),
    SwitchRequest(state), camIndex(camIndex)
  {
  }
    
  unsigned camIndex;
};
typedef boost::shared_ptr<SwitchVideoWriter> SwitchVideoWriterPtr;

/**
 * @class SwitchFieldProjection
 * @brief A request to switch on and off the field projection
 */ 
struct SwitchFieldProjection : public VisionRequest, SwitchRequest
{
  /**
   * Constructor
   */ 
  SwitchFieldProjection(const bool& state) :
    VisionRequest(VisionRequestIds::switchFieldProjection),
    SwitchRequest(state)
  {
  }
};
typedef boost::shared_ptr<SwitchFieldProjection> SwitchFieldProjectionPtr;

/**
 * @class SwitchLogImages
 * @brief A request to switch on and off the images logging
 */ 
struct SwitchLogImages : public VisionRequest, SwitchRequest
{
  /**
   * Constructor
   */ 
  SwitchLogImages(const bool& state, const unsigned& camIndex) :
    VisionRequest(VisionRequestIds::switchLogImages),
    SwitchRequest(state), camIndex(camIndex)
  {
  }
  
  unsigned camIndex;
};
typedef boost::shared_ptr<SwitchLogImages> SwitchLogImagesPtr;

/**
 * @class SwitchUseLoggedImages
 * @brief A request to switch on and off the usage of logged images
 */ 
struct SwitchUseLoggedImages : public VisionRequest, SwitchRequest
{
  /**
   * Constructor
   */ 
  SwitchUseLoggedImages(const bool& state) :
    VisionRequest(VisionRequestIds::switchUseLoggedImages),
    SwitchRequest(state)
  {
  }
};
typedef boost::shared_ptr<SwitchUseLoggedImages> SwitchUseLoggedImagesPtr;

/**
 * @class SwitchFeatureExtModule
 * @brief A request to switch on and off the usage of feature extraction modules
 */
struct SwitchFeatureExtModule : public VisionRequest, SwitchRequest
{
  /**
   * Constructor
   */
  SwitchFeatureExtModule(const bool& state, const FeatureExtractionIds& id, const unsigned& camIndex = 0) :
    VisionRequest(VisionRequestIds::switchFeatureExtModule),
    SwitchRequest(state),
    id(id),
    camIndex(camIndex)
  {
  }

  unsigned camIndex;
  FeatureExtractionIds id;
};
typedef boost::shared_ptr<SwitchFeatureExtModule> SwitchFeatureExtModulePtr;
