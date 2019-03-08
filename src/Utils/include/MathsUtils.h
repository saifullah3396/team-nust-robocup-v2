/**
 * @file Utils/include/MathsUtils.h
 *
 * This file defines the class MathsUtils.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#ifndef _MATHS_UTILS_H_
#define _MATHS_UTILS_H_

#include <opencv2/core/core.hpp>
#include "Eigen/Dense"
#include "Eigen/SVD"
#include "Eigen/StdVector"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2f)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<float, 6, 6>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<float, 3, 4>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(vector<Eigen::Matrix4f>)

using namespace Eigen;
using namespace std;

namespace MathsUtils
{
  const double DEG_TO_RAD = 0.0174532925199433;
  const double RAD_TO_DEG = 57.2957795130823;

  /**
   * @brief Return the sign of a value
   * @param var Value
   * @return Sign -1 1
   */
  template<typename Scalar>
  Scalar sign(const Scalar& var);

  /**
   * @brief Returns the degrees in radians
   * @param var Input in degrees
   * @return Radians
   */
  template<typename Scalar>
  Scalar degToRads(const Scalar& var);

  /**
   * @brief Returns the a-cotangent of a variable.
   * @param var Input variable
   * @return Scalar A-cotangent of the variable.
   */
  template<typename Scalar>
  Scalar aCotan(const Scalar& var);

  /**
   * @brief Converts an angle from range (0 to 2 * M_PI) to the range (-M_PI to M_PI).
   * @param angle Input angle between (0 to M_TWO_PI)
   * @return Scalar Output angle between (-M_PI to M_PI)
   */
  template<typename Scalar>
  Scalar rangeToPi(const Scalar& angle);

  /**
   * @brief Finds the smallest difference between two angles in range
   *   (-M_PI to M_PI)
   * @param a1 First angle
   * @param a2 Second angle
   * @return Scalar Difference of angles
   */
  template<typename Scalar>
  Scalar diffAngle(const Scalar& a1, const Scalar& a2);

  /**
   * @brief Adds two angles in range (-M_PI to M_PI)
   * @param a1 First angle
   * @param a2 Second angle
   * @return Scalar Addition of angles
   */
  template<typename Scalar>
  Scalar addAngles(const Scalar& a1, const Scalar& a2, const Scalar& factor2);

  /**
   * @brief Returns the Euclidean distance between two 2D points.
   * @param x1 X coordinate of first point
   * @param y1 Y coordinate of first point
   * @param x2 X coordinate of second point
   * @param y2 Y coordinate of second point
   * @return Scalar Euclidean distance
   */
  template<typename Scalar>
  Scalar dist(const Scalar& x1, const Scalar& y1, const Scalar& x2, const Scalar& y2);

  /**
   * @brief Returns the quaternion representation of given euler angles.
   * @param euler Euler angles with sequence of rotation ZYX.
   * @return Quaternion
   */
  template<typename Derived>
  Quaternion<typename Derived::Scalar> eulerToQuaternion(const MatrixBase<Derived>& euler);

  /**
   * @brief Returns the rotation matrix for a given quaternion.
   * @param q Input quaternion
   * @return MatrixBase<Derived> Rotation matrix
   */
  template<typename Derived>
  MatrixBase<Derived> quaternionToMat(const QuaternionBase<Derived>& q);

  /**
   * @brief Returns the euler angles for a given rotation matrix.
   * @param rot Input rotation matrix
   * @return Matrix<typename Derived::Scalar, 3, 1> Euler angles
   */
  template<typename Derived>
  Matrix<typename Derived::Scalar, 3, 1> matToEuler(const MatrixBase<Derived>& rot);

  /**
   * @brief Returns a quaternion for a given rotation matrix.
   * @param rot Input rotation matrix
   * @return Quaternion
   */
  template<typename Derived>
  Quaternion<typename Derived::Scalar> matToQuaternion(const MatrixBase<Derived>& rot);

  /**
   * @brief Returns the rotation matrix representation for the given
   *   euler angles.
   * @param euler Euler angles with sequence of rotation ZYX
   * @return MatrixBase<Derived> Rotaion matrix
   */
  template<typename Derived>
  MatrixBase<Derived> eulerToMat(const MatrixBase<Derived>& euler);

  /**
   * @brief Returns the euler representation for a given quaternion.
   * @param q Input quaternion
   * @return Matrix<Scalar, 3, 1> Euler angles
   */
  template<typename Derived>
  Matrix<typename Derived::Scalar, 3, 1>
  quaternionToEuler(const QuaternionBase<Derived>& q);

  /**
   * @brief Returns the difference of orientation between two transformation matrices
   * @param t1: First transformation matrix
   * @param t2: Second transformation matrix
   * @return Matrix<Scalar, 3, 1> Difference of orientation along rho-theta-phi
   */
  template<typename Derived>
  Matrix<typename Derived::Scalar, 3, 1>
  getOrientationDiff(const MatrixBase<Derived>& t1, const MatrixBase<Derived>& t2);

  /**
   * @brief Sets mat as a 4x4 transformation matrix based on the
   *   X-Y-Z translational coordinates.
   * @param mat: Output matrix
   * @param x X-translation
   * @param y Y-translation
   * @param z Z-translation
   * @return void
   */
  template<typename Derived>
  void makeTranslation(
    MatrixBase<Derived>& mat,
    const typename Derived::Scalar& x,
    const typename Derived::Scalar& y,
    const typename Derived::Scalar& z);

  /**
   * @brief Sets mat as a 4x4 transformation matrix or a 3x3 rotation
   *   matrix for rotation about Z-axis.
   * @param mat Output matrix
   * @param angle Angle of rotation
   * @return void
   */
  template<typename Derived>
  void makeRotationZ(
    MatrixBase<Derived>& mat,
    const typename Derived::Scalar& angle);

  /**
   * @brief Sets mat as a 4x4 transformation matrix or a 3x3 rotation
   *   matrix for rotation about Y-axis.
   * @param mat Output matrix
   * @param angle Angle of rotation
   * @return void
   */
  template<typename Derived>
  void makeRotationY(
    MatrixBase<Derived>& mat,
    const typename Derived::Scalar& angle);

  /**
   * @brief Sets mat as a 4x4 transformation matrix or a 3x3 rotation
   *   matrix for rotation about X-axis.
   * @param mat Output matrix
   * @param angle Angle of rotation
   * @return void
   */
  template<typename Derived>
  void makeRotationX(
    MatrixBase<Derived>& mat,
    const typename Derived::Scalar& angle);

  /**
   * @brief Sets mat as a 4x4 transformation matrix or a 3x3 rotation
   *   matrix for rotations about X-Y-Z axes.
   * @param mat Output matrix
   * @param xAngle Angle of rotation about the x-axis
   * @param yAngle Angle of rotation about the y-axis
   * @param zAngle Angle of rotation about the z-axis
   * @return void
   */
  template<typename Derived>
  void makeRotationXYZ(
    MatrixBase<Derived>& mat,
    const typename Derived::Scalar& xAngle,
    const typename Derived::Scalar& yAngle,
    const typename Derived::Scalar& zAngle);

  /**
   * @brief Sets mat as a 4x4 transformation matrix or a 3x3 rotation
   *   matrix for rotations about Z-Y-X axes.
   * @param mat Output matrix
   * @param zAngle Angle of rotation about the z-axis
   * @param yAngle Angle of rotation about the y-axis
   * @param xAngle Angle of rotation about the x-axis
   * @return void
   */
  template<typename Derived>
  void makeRotationZYX(
    MatrixBase<Derived>& mat,
    const typename Derived::Scalar& zAngle,
    const typename Derived::Scalar& yAngle,
    const typename Derived::Scalar& xAngle);

  /**
   * @brief Sets mat as a 4x4 transformation matrix for a given
   *   3x3 rotation matrix and X-Y-Z translational coordinates.
   * @param mat Output matrix
   * @param rot Rotation matrix
   * @param x X-translation
   * @param y Y-translation
   * @param z Z-translation
   * @return void
   */
  template<typename Derived>
  Matrix<typename Derived::Scalar, 4, 4> makeTransformation(
    const MatrixBase<Derived>& rot,
    const typename Derived::Scalar& x,
    const typename Derived::Scalar& y,
    const typename Derived::Scalar& z);

  /**
   * @brief Returns a 3x1 vector of euler angles of sequence ZYX from a
   *   given 4x4 transformation pr 3x3 rotation matrix.
   * @param mat Input matrix
   * @return Matrix<typename Derived::Scalar, 3, 1>
   */
  template<typename Derived>
  Matrix<typename Derived::Scalar, 3, 1>
  getEulerAngles(const MatrixBase<Derived>& mat);

  /**
   * @brief Returns the inverse of a 4x4 transformation matrix.
   * @param mat Input matrix
   * @return Derived
   */
  template<typename Derived>
  Derived getTInverse(const MatrixBase<Derived>& mat);

  /**
   * @brief Sets mat as a 4x4 DH-transformation matrix for the given
   *   Denavit-Hartenbarg (DH) parameters.
   * @param mat Output transformation matrix
   * @param a Link length
   * @param alpha Link twist
   * @param d Joint offset
   * @param theta Joint angle
   * @return void
   */
  template<typename Derived>
  void makeDHTransformation(
    MatrixBase<Derived>& mat,
    const typename Derived::Scalar& a,
    const typename Derived::Scalar& alpha,
    const typename Derived::Scalar& d,
    const typename Derived::Scalar& theta);

  /**
   * @brief Transforms a 3x1 vector by a 4x4 transformation matrix.
   * @param mat Transformation matrix
   * @param vec Transformed vector
   * @return Matrix<typename Derived::Scalar, 3, 1>
   */
  template<typename Derived>
  Matrix<typename Derived::Scalar, 3, 1> transformVector(
    const MatrixBase<Derived>& mat,
    const Matrix<typename Derived::Scalar, 3, 1> & vec);

  /**
   * @brief Returns a 3x3 skew symmetric matrix from a given 3x1 vector.
   * @param vec Input vector
   * @return Matrix<Derived::Scalar, 3, 3>
   */
  template<typename Derived>
  Matrix<typename Derived::Scalar, 3, 3> makeSkewMat(
    const MatrixBase<Derived>& vec);

  /**
   * @brief Sets the input transformation matrix to its mirrored
   *   transformation.
   * @param mat Transformation matrix
   * @return void
   */
  template<typename Derived>
  Derived mirrorTransformation(const MatrixBase<Derived>& mat);

  /**
   * @brief Returns true if the two matrices are almost equal.
   * @param first First matrix
   * @param second Second matrix
   * @param tol Tolerance value for magnitude comparison
   * @return boolean
   */
  template<typename Derived>
  bool almostEqual(
    const Derived& first, const Derived& second,
    const typename Derived::Scalar tol = typename Derived::Scalar(0.5));

  /**
   * @brief Returns true if the scalar values are almost equal
   * @param first First value
   * @param second Second value
   * @param tol Tolerance value for magnitude comparison
   * @return boolean
   */
  template<typename Scalar>
  bool almostEqual(const Scalar& first, const Scalar& second, const Scalar tol);

  /**
   * @brief Returns cosine of a value within range of -1 to 1.
   * @param var Input variable
   * @return Scalar
   */
  template<typename Scalar>
  Scalar safeAcos(Scalar var);

  /**
   * @brief Returns the pseudo inverse of a matrix using singular value
   *   decomposition.
   * @param mat Input matrix
   * @param tolerance The pseudoInverse result tolerance
   * @return Eigen::MatrixBase<Derived>
   */
  template<typename Derived>
  Eigen::Matrix<typename Derived::Scalar, Dynamic, Dynamic>
  pseudoInverse(
    const MatrixBase<Derived>& mat,
    const typename Derived::Scalar tolerance = typename Derived::Scalar(1e-4));

  /**
   * @brief Returns the pseudo inverse multiplication with an right hand
   *   side matrix.
   * @param mat Input matrix
   * @param rhs Right hand side matrix
   * @param tolerance PseudoInverse result tolerance.
   * @return Eigen::MatrixBase<Derived>
   */
  template<typename Derived>
  Eigen::Matrix<typename Derived::Scalar, Dynamic, 1>
  pseudoInverseSolve(
    const MatrixBase<Derived>& mat,
    const Matrix<typename Derived::Scalar, Dynamic, 1> &rhs);

  /**
   * Solves the discrete algebraic riccati equation to get the
   * optimal controller gains.
   *
   * @param stateSize: Number of state variables
   * @param A: State-Transition matrix
   * @param B: System Input matrix
   * @param Q: Cost weighting matrix
   * @param R: Cost weighting parameter
   *
   * @return Matrix<T, size>
   */
  template <typename T, size_t size>
  Matrix<T, size, size> dare(
    Matrix<T, size, size>& AA,
    Matrix<T, size, 1>& BB,
    Matrix<T, size, size>& QQ,
    const T& RR)
  {
    double R = (double)RR;
    Matrix<double, size, size> A = AA.template cast<double>();
    Matrix<double, size, 1> B = BB.template cast<double>();
    Matrix<double, size, size> Q = QQ.template cast<double>();
    bool converged = false;
    Matrix<double, size, size> X;
    X.setIdentity();
    Matrix<double, size, size> At = A.transpose();
    Matrix<double, 1, size> Bt = B.transpose();
    for (int i = 0; i < 10000; ++i) {
      Matrix<double, size, size> AX = At * X;
      Matrix<double, size, size> AXA = AX * A;
      Matrix<double, size, 1> AXB = AX * B;
      double M = ((Bt * X * B).array() + R)(0, 0);
      Matrix<double, size, size> Xnew =
        AXA - AXB * (1.0 / M) * AXB.transpose() + Q;
      double relError = (Xnew - X).norm() / Xnew.norm();
      X = Xnew;
      if (relError < 1e-10) {
        converged = true;
        break;
      }
    }
    return X.template cast<T>();
  }

  /**
   * @brief Addition operator for standard vectors.
   * @param first First vector
   * @param second Second vector
   * @return vector
   */
  template<typename Scalar>
  vector<Scalar> operator+(const vector<Scalar>& first, const vector<Scalar>& second);

  /**
   * @brief Difference operator for standard vectors.
   * @param first First vector
   * @param second Second vector
   * @return vector
   */
  template<typename Scalar>
  vector<Scalar>
  operator-(const vector<Scalar>& first, const vector<Scalar>& second);

  /**
   * @brief Scalar multiplication operator for standard vectors.
   * @param first First vector
   * @param second Second vector
   * @return vector
   */
  template<typename Scalar>
  vector<Scalar>
  operator*(const vector<Scalar>& vec, const Scalar& constant);

  /**
   * @brief Division operator for OpenCV points
   * @param p First point
   * @param value Constant value
   * @return point
   */
  template<typename Scalar>
  cv::Point_<Scalar> operator/(const cv::Point_<Scalar>& p, const Scalar value);

} //! MathsUtils
#endif //! _MATHS_UTILS_H_
