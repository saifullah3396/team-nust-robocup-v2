/**
 * @file LocalizationModule/include/Particle.h
 *
 * This file defines the struct Particle
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include "Utils/include/DataHolders/RobotPose2D.h"

/**
 * @struct Particle
 * @brief The struct for defining a single particle of the filter.
 */
struct Particle
{
  Particle() = default;
  Particle(const Particle&) = default;
  Particle(Particle&&) = default;
  Particle& operator=(const Particle&) & = default;
  Particle& operator=(Particle&&) & = default;
  virtual ~Particle() {}

  RobotPose2D<float> state;
  double weight = {0.0};
};
