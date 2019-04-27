/**
 * @file LocalizationModule/include/Particle.h
 *
 * This file declares the struct Particle
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include "Utils/include/DataHolders/RobotPose2D.h"

/**
 * @struct Particle
 * @brief Defines a single particle of the particle filter.
 */
struct Particle : public RobotPose2D<float>
{
  Particle() = default;
  Particle(const Particle&) = default;
  Particle(Particle&&) = default;
  Particle& operator=(const Particle&) & = default;
  Particle& operator=(Particle&&) & = default;
  virtual ~Particle() {}

  /**
   * @brief Particle Constructor.
   * @param x X-coordinate of the particle.
   * @param y Y-coordinate of the particle.
   * @param theta Theta-coordinate of the particle.
   */
  Particle(const float& x, const float& y, const float& theta);

  /**
   * @brief Particle Constructor.
   * @param x X-coordinate of the particle.
   * @param y Y-coordinate of the particle.
   * @param theta Theta-coordinate of the particle.
   * @param weight Particle weight.
   */
  Particle(const float& x, const float& y, const float& theta, const double& weight);

  /**
   * @brief transform Transforms a given pose from particle pose
   * @param pose Any given pose
   * @return Particle
   */
  template <typename OtherScalar>
  Particle transform(const RobotPose2D<OtherScalar>& pose);

  /**
   * @brief transform Transforma given particle from this
   *   particle pose
   * @param p Any given particle
   * @return Particle
   */
  Particle transform(const Particle& p);

  template <typename OtherScalar>
  const Particle& operator=(const RobotPose2D<OtherScalar>& pose)
  {
    this->set(pose.get());
    this->ct = pose.getCTheta();
    this->st = pose.getSTheta();
    return *this;
  }

  double weight = {0.0}; ///< Particle weight
};
