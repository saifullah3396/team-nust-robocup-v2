/**
 * @file Utils/include/BSplineNormalFinder.h
 *
 * This file declares the class BSplineNormalFinder
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once
#include "Utils/include/Splines/BSpline.h"
#include "Utils/include/Solvers/NLOptimizer.h"
#include "Utils/include/MathsUtils.h"

/**
 * @class BSplineNormalFinder
 * @brief A class to solve for the normal to a given bspline
 */
template <typename Scalar>
class BSplineNormalFinder : public NLOptimizer
{
public:
  /**
   * @brief BSplineNormalFinder A numerical solver for finding a point on a BSpline
   *   where the given vector becomes normal to it
   * @param bSplinePtr BSpline
   * @param normal Normal vector
   * @param tBounds Maximum bounds for curve parameter
   */
  BSplineNormalFinder(
    BSpline<Scalar>* bSplinePtr,
    const Matrix<Scalar, 3, 1>& normal,
    const Matrix<Scalar, 2, 1>&tBounds) : 
    bSplinePtr(bSplinePtr),
    normal(normal),
    tBounds(tBounds)
  {
  }
  
  /**
   * @brief ~BSplineNormalFinder Destructor
   */
  ~BSplineNormalFinder()
  {
  }

  /**
   * @brief optDef Solves the optimization problem that gives
   *   BSpline'(t) . \hat{n} = M_PI / 2
   */
  virtual void optDef() final;

  Scalar getResParam() { return resT; }
  Matrix<Scalar, 3, 1> getResSplinePoint() { return resSplinePoint; }

protected:
  /**
   * Evaluates the objective function
   */
  double costFunction(const vector<double> &vars, vector<double> &grad, void *data) final;
  
  BSpline<Scalar>* bSplinePtr; //! Pointer to associated BSpline
  Matrix<Scalar, 3, 1> normal; //!  Normal vector
  Matrix<Scalar, 2, 1> tBounds; //! Bounds of 't'
  Scalar resT; //! Resulting curve paramter 't' giving the normal point
  Matrix<Scalar, Dynamic, 1> resSplinePoint; //! Resulting point on curve
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
