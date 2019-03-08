/**
 * @file MotionModule/TrajectoryPlanner/NloptConstraint.h
 *
 * This file declares the class NloptConstraint
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#pragma once

/**
 * @class NloptConstraint
 * @brief The class to define a constraint handler for nlopt based
 *   optimization problems
 */
template <typename Scalar>
class NloptConstraint
{
public:
  /**
   * Constructor.
   */
  NloptConstraint(const string& name) : name(name)
  {
  }

  /**
   * Destructor
   */
  ~NloptConstraint()
  {
  }

  string getName() { return name; }

  virtual void init() = 0;
  virtual void update() = 0;
  virtual vector<Scalar> computeConstraintMatrix() = 0;
  virtual vector<Scalar> getTolerance() { return tols; }
  virtual unsigned getNConstraints() = 0;
  virtual Matrix<Scalar, Dynamic, 1> getValue() = 0;

protected:
  //! Tolerance vector
  vector<Scalar> tols;

private:
  string name;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
