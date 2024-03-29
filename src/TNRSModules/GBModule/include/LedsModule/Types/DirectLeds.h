/**
 * @file GBModule/include/LedsModule/Types/DirectLeds.h
 *
 * This file declares the class DirectLeds
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "GBModule/include/LedsModule/LedsModule.h"

/**
 * @class DirectLeds
 * @brief A class for directly sending leds to required intensities
 */
class DirectLeds : public LedsModule
{
public:
  /**
   * @brief DirectLeds Constructor
   * 
   * @param gbModule Pointer to base static behaviors module
   * @param config Configuration of the behavior
   */
  DirectLeds(GBModule* gbModule, const boost::shared_ptr<GBLedsConfig>& config);

  /**
   * @brief ~DirectLeds Destructor
   */
  ~DirectLeds() final {}

  /**
   * @brief initiate See Behavior::initiate()
   */
  bool initiate() final;

  /**
   * @brief update See Behavior::update()
   */
  void update() final;

  /**
   * @brief finish See Behavior::finish()
   */
  void finish() final;

  /**
   * @brief loadExternalConfig See Behavior::loadExternalConfig()
   */
  void loadExternalConfig() final {}
};

typedef boost::shared_ptr<DirectLeds> DirectLedsPtr;
