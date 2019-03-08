/**
 * @file SBModule/include/LedsModule/Types/InterpolateLeds.h
 *
 * This file declares the class InterpolateLeds
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "SBModule/include/LedsModule/LedsModule.h"

/**
 * @class InterpolateLeds
 * @brief A class for interpolating leds to required intensities
 */
class InterpolateLeds : public LedsModule
{
public:
  /**
   * @brief InterpolateLeds Constructor
   * 
   * @param sbModule Pointer to base static behaviors module
   * @param config Configuration of the behavior
   */
  InterpolateLeds(SBModule* sbModule, const boost::shared_ptr<SBLedsConfig>& config);

  /**
   * @brief ~InterpolateLeds Destructor
   */
  ~InterpolateLeds() final {}

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
  
private:
  //! Initial led intensities
  vector<float> ledsI;

  //! Difference with desired light intensity
  vector<float> ledsDelta;
};

typedef boost::shared_ptr<InterpolateLeds> InterpolateLedsPtr;
