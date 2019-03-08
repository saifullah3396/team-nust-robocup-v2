/**
 * @file MotionModule/WalkZmpRefGen/WalkZmpRefGen.h
 *
 * This file declares the class WalkZmpRefGen
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include <deque>
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/BalanceModule/ZmpRefGenerator.h"

template <typename Scalar>
struct TNRSFootstep;

/**
 * @class WalkZmpRefGen
 * @brief The class for generating zmp references for walking cycle
 */
template <typename Scalar>
class WalkZmpRefGen : public ZmpRefGenerator<Scalar>
{
public:
  /**
   * @brief BalanceZmpRefGen Constructor
   * @param motionModule Pointer to base motion module
   * @param refFrame Base reference foot
   * @param nReferences Number of references to be generated
   * @param totalTime Total time for shifting balance
   * @param targetZmp Target zmp positions
   */
  WalkZmpRefGen(
    MotionModule* motionModule,
    const RobotFeet& refFrame,
    const unsigned& nReferences,
    std::deque<boost::shared_ptr<TNRSFootstep<Scalar>>>* footsteps);

  /**
   * @brief ~BalanceZmpRefGen Destructor
   */
  ~WalkZmpRefGen() final {}

  /**
   * @brief update See ZmpRefGenerator::initiate()
   */
  bool initiate() final;

  /**
   * @brief update See ZmpRefGenerator::update()
   */
  void update(const Scalar& timeStep) final;

  /**
   * @brief previewsAvailable Returns true if the queued steps
   * are sufficient to fill the preview window
   * @param timeStep Time step of walk update cycle
   * @return Boolean
   */
  bool previewsAvailable(const Scalar& timeStep);

private:
  std::deque<boost::shared_ptr<TNRSFootstep<Scalar>>>* footsteps;

  //! Pose that determines total transformation
  RobotPose2D<Scalar> transPose;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<WalkZmpRefGen<MType> > WalkZmpRefGenPtr;
