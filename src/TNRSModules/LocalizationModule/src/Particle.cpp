/**
 * @file LocalizationModule/src/Particle.cpp
 *
 * This file implements the struct Particle
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "LocalizationModule/include/Particle.h"
#include "Utils/include/MathsUtils.h"

Particle::Particle(const float& x, const float& y, const float& theta) :
  RobotPose2D<float>(x, y, theta)
{
}

template <typename OtherScalar>
Particle Particle::transform(const RobotPose2D<OtherScalar>& pose)
{
  return
    Particle(
      getX() + pose.getX() * ct - pose.getY() * st,
      getY() + pose.getX() * st + pose.getY() * ct,
      MathsUtils::rangeToPi(pose.getTheta() + getTheta())
    );
}
template Particle Particle::transform(const RobotPose2D<float>& p);
template Particle Particle::transform(const RobotPose2D<double>& p);

Particle Particle::transform(const Particle& p)
{
  return transform(p);
}