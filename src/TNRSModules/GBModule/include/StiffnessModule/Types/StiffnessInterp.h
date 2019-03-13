/**
 * @file GBModule/include/StiffnessModule/Types/StiffnessInterp.h
 *
 * This file declares the class StiffnessInterp
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "GBModule/include/StiffnessModule/StiffnessModule.h"
#include "Utils/include/HardwareIds.h"

/**
 * @class StiffnessInterp
 * @brief A class that interpolates the joint stiffnesses from initial state to
 *   final state
 */
class StiffnessInterp : public StiffnessModule
{
public:
  /**
   * @brief StiffnessInterp Constructor
   * @param gbModule Pointer to base sb module
   * @param config Configuration of this behavior
   */
  StiffnessInterp(
    GBModule* gbModule,
    const boost::shared_ptr<GBStiffnessConfig>& config);

  /**
   * @brief ~StiffnessInterp Destructor
   */
  ~StiffnessInterp() final {}

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
  //! Joint configuration initial
  vector<float> sI = {vector<float>(toUType(Joints::count), NAN)};

  //! Joint configuration difference
  vector<float> sDelta = {vector<float>(toUType(Joints::count), NAN)};
};
