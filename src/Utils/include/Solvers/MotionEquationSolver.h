/**
 * @file PlanningBehaviors/KickSequence/MotionEquationSolver.h
 *
 * This file declares the class MotionEquationSolver
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once
#include "Utils/include/Solvers/NLOptimizer.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/Constants.h"

/**
 * @class MotionEquationSolver
 * @brief A class to solve the equation of motion for ball to estimate
 *   the time it will take to get closest to the target.
 */
class MotionEquationSolver : public NLOptimizer
{
public:
  /**
   * @brief MotionEquationSolver Constructor
   * @param target Target position
   * @param posI Initial position
   * @param velI Initial velocity
   */
  MotionEquationSolver(
    const Vector2f& target, 
    const Vector2f& posI,
    const Vector2f& velI) : 
    target(target), posI(posI), velI(velI)
  {
  }
  
  /**
   * @brief ~MotionEquationSolver Constructor
   */
  ~MotionEquationSolver() {}

  /**
   * @brief optDef Solves the optimization problem that minimizes the norm
   *   X = |pos(t) - target| -> t ~= 0 to max for given input state
   */
  virtual void optDef() final;

  Vector2f getEndPosition() const { return endPosition; }
  Vector2f getEndVelocity() const { return endVelocity; }
  float getTimeToReach() const { return timeToReach; }
  float getDistFromTarget() const { return distFromTarget; }

protected:
  /**
   * Solves the equation of motion and returns position based on time
   *
   * @param time: Current time
   * @return Position for given time
   */  
  virtual Vector2f solvePosition(const double& time) = 0;
  
  /**
   * Solves the equation of motion and returns velocity based on time
   *
   * @param time: Current relative time
   * @return Velocity for given time
   */  
  virtual Vector2f solveVelocity(const double& time) = 0;

  /**
   * Evaluates the objective function
   */
  double costFunction(const vector<double> &vars, vector<double> &grad, void *data) final;
  
  Vector2f target; //! Target to find solution for
  Vector2f posI; //! Ball initial position
  Vector2f velI; //! Ball initial velocity
  Vector2f endPosition = {Vector2f::Zero()}; //! Solved end position
  Vector2f endVelocity = {Vector2f::Zero()}; //! Solved end velocity
  float distFromTarget = {0.f}; //! Distance of the end position from the given target
  float timeToReach = {0.f}; //! Time to reach the end position
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class DampedMESolver
 * @brief A class to solve the equation of motion of a damped system
 *   without stiffness
 */
class DampedMESolver : public MotionEquationSolver {
public:
  /**
   * @brief DampedMESolver Constructor
   * @param target Target position
   * @param posI Initial position
   * @param velI Initial velocity
   * @param damping Damping coefficient
   */
  DampedMESolver(
    const Vector2f& target, 
    const Vector2f& posI,
    const Vector2f& velI,
    const double& damping) : 
    MotionEquationSolver(target, posI, velI), damping(damping) 
  {}
  
  /**
   * @brief ~DampedMESolver Destructor
   */ 
  ~DampedMESolver() {}
  
  /**
   * Derived
   */ 
  Vector2f solvePosition(const double& time) final {
    return posI + velI / damping * (1 - exp(-damping * time));
  }
  
  /**
   * Derived
   */ 
  Vector2f solveVelocity(const double& time) final {
    return velI * exp(-damping * time);
  }
  
private:
  double damping; //! Damping coefficient
};

/**
 * @class FrictionMESolver
 * @brief A class to solve the equation of motion of friction based
 *   motion equation
 */
class FrictionMESolver : public MotionEquationSolver {
public:
  /**
   * @brief FrictionMESolver Constructor
   * @param target Target position
   * @param posI Initial position
   * @param velI Initial velocity
   * @param friction Friction coefficient
   */
  FrictionMESolver(
    const Vector2f& target, 
    const Vector2f& posI,
    const Vector2f& velI,
    const double& friction) : 
    MotionEquationSolver(target, posI, velI)
  {
    // velocity direction
    auto unitVel = velI / velI.norm();
    accel[0] = -friction * Constants::gravity * unitVel[0];
    accel[1] = -friction * Constants::gravity * unitVel[1];
  }
  
  /**
   * @brief ~FrictionMESolver Destructor
   */ 
  ~FrictionMESolver() {}
  
  /**
   * Derived
   */ 
  Vector2f solvePosition(const double& time) final {
    return posI + velI * time + 0.5 * accel * time * time;
  }
  
  /**
   * Derived
   */ 
  Vector2f solveVelocity(const double& time) final {
    return velI + accel * time;
  }
  
private:
  Vector2f accel; //! Constant Deceleration due to friction
};

