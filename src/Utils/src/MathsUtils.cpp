/**
 * @file Utils/src/MathsUtils.cpp
 *
 * This file defines the class MathsUtils.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include "Utils/include/MathsUtils.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/DebugUtils.h"

namespace MathsUtils
{

template<typename Scalar>
Scalar sign(const Scalar& var)
{
  return (var > 0) - (var < 0);
}
template int sign(const int& var);
template float sign(const float& var);
template double sign(const double& var);

template<typename Scalar>
Scalar degToRads(const Scalar& var)
{
  return var * DEG_TO_RAD;
}
template float degToRads<float>(const float& var);
template double degToRads<double>(const double& var);

template<typename Scalar>
Scalar aCotan(const Scalar& var)
{
  return atan(1 / var);
}
template float aCotan<float>(const float& var);
template double aCotan<double>(const double& var);

template<typename Scalar>
Scalar rangeToPi(const Scalar& angle)
{
  if (angle > M_PI) return angle - M_2_PI;
  else if (angle < -M_PI) return angle + M_2_PI;
  return angle;
}
template float rangeToPi<float>(const float& angle);
template double rangeToPi<double>(const double& angle);

template<typename Scalar>
Scalar diffAngle(const Scalar& a1, const Scalar& a2)
{
  ASSERT(abs(a1) < M_PI && abs(a2) < M_PI);
  if (a1 * a2 >= 0) return rangeToPi(a1 - a2);
  if (a1 < 0) return rangeToPi(M_2_PI + a1 - a2);
  return rangeToPi(a1 - (M_2_PI + a2));
}
template float diffAngle<float>(const float& a1, const float& a2);
template double diffAngle<double>(const double& a1, const double& a2);

template<typename Scalar>
Scalar addAngles(const Scalar& a1, const Scalar& a2, const Scalar& factor2)
{
  ASSERT(abs(a1) < M_PI && abs(a2) < M_PI);
  auto ca1 = rangeToPi(a1);
  auto ca2 = rangeToPi(a2);
  ca1 = (ca1 < 0) ? ca1 + M_2_PI : ca1;
  ca2 = (ca2 < 0) ? ca2 + M_2_PI : ca2;
  return rangeToPi(ca1 + factor2 * a2);
}
template float addAngles<float>(const float& a1, const float& a2, const float& factor2);
template double addAngles<double>(const double& a1, const double& a2, const double& factor2);

template<typename Scalar>
Scalar dist(const Scalar& x1, const Scalar& y1, const Scalar& x2, const Scalar& y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
template float dist<float>(const float& x1, const float& y1, const float& x2, const float& y2);
template double dist<double>(const double& x1, const double& y1, const double& x2, const double& y2);

template<typename Derived>
Quaternion<typename Derived::Scalar> eulerToQuaternion(const MatrixBase<Derived>& euler)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
        Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  typedef typename Derived::Scalar Scalar;
  Scalar cr = cos(euler[0] / 2);
  Scalar sr = sin(euler[0] / 2);
  Scalar cp = cos(euler[1] / 2);
  Scalar sp = sin(euler[1] / 2);
  Scalar cy = cos(euler[2] / 2);
  Scalar sy = sin(euler[2] / 2);
  Quaternion<Scalar> q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;
  return q;
}
template Quaternion<typename Matrix<float, 3, 1>::Scalar> eulerToQuaternion<Matrix<float, 3, 1> >(const MatrixBase<Matrix<float, 3, 1> >& euler);
template Quaternion<typename Matrix<double, 3, 1>::Scalar> eulerToQuaternion<Matrix<double, 3, 1>>(const MatrixBase<Matrix<double, 3, 1> >& euler);

template<typename Derived>
MatrixBase<Derived> quaternionToMat(const QuaternionBase<Derived>& q)
{
  return q.toRotationMatrix().matrix();
}

template<typename Derived>
Matrix<typename Derived::Scalar, 6, 1> matToVector(const MatrixBase<Derived>& tMat)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
        Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  Matrix<typename Derived::Scalar, 6, 1> out;
  out << tMat.block(0, 3, 3, 1), matToEuler((Matrix<typename Derived::Scalar, 3, 3>)tMat.block(0, 0, 3, 3));
  return out;
}
template Matrix<typename Matrix<float, 4, 4>::Scalar, 6, 1> matToVector<Matrix<float, 4, 4> >(const MatrixBase<Matrix<float, 4, 4> >& tMat);
template Matrix<typename Matrix<double, 4, 4>::Scalar, 6, 1> matToVector<Matrix<double, 4, 4> >(const MatrixBase<Matrix<double, 4, 4> >& tMat);

template<typename Derived>
Matrix<typename Derived::Scalar, 3, 1> matToEuler(const MatrixBase<Derived>& rot)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  auto r = atan2(rot(2, 1), rot(2, 2));
  auto p = asin(-rot(2, 0));
  auto y = atan2(rot(1, 0), rot(0, 0));
  return Matrix<typename Derived::Scalar, 3, 1>(r, p, y);
}
template Matrix<typename Matrix<float, 3, 3>::Scalar, 3, 1> matToEuler<Matrix<float, 3, 3> >(const MatrixBase<Matrix<float, 3, 3> >& rot);
template Matrix<typename Matrix<double, 3, 3>::Scalar, 3, 1> matToEuler<Matrix<double, 3, 3> >(const MatrixBase<Matrix<double, 3, 3> >& rot);
template Matrix<typename Matrix<float, 4, 4>::Scalar, 3, 1> matToEuler<Matrix<float, 4, 4> >(const MatrixBase<Matrix<float, 4, 4> >& rot);
template Matrix<typename Matrix<double, 4, 4>::Scalar, 3, 1> matToEuler<Matrix<double, 4, 4> >(const MatrixBase<Matrix<double, 4, 4> >& rot);

template<typename Derived>
Quaternion<typename Derived::Scalar> matToQuaternion(const MatrixBase<Derived>& rot)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
        Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  Quaternion<typename Derived::Scalar> q;
  auto q0 = sqrt(1 + rot(0, 0) + rot(1, 1) + rot(2, 2)) / 2;
  auto q1 = (rot(2, 1) - rot(1, 2)) / (4 * q0);
  auto q2 = (rot(0, 2) - rot(2, 0)) / (4 * q0);
  auto q3 = (rot(1, 0) - rot(0, 1)) / (4 * q0);
  q.w() = q0;
  q.x() = q1;
  q.y() = q2;
  q.z() = q3;
  return q;
}
template Quaternion<typename Matrix<float, 3, 3>::Scalar> matToQuaternion<Matrix<float, 3, 3>>(const MatrixBase<Matrix<float, 3, 3> >& rot);
template Quaternion<typename Matrix<double, 3, 3>::Scalar> matToQuaternion<Matrix<double, 3, 3>>(const MatrixBase<Matrix<double, 3, 3> >& rot);

template<typename Derived>
Matrix<typename Derived::Scalar, 3, 3> eulerToMat(const MatrixBase<Derived>& euler)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
    Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
    THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE
  );
  auto cr = cos(euler[0]);
  auto sr = sin(euler[0]);
  auto cp = cos(euler[1]);
  auto sp = sin(euler[1]);
  auto cy = cos(euler[2]);
  auto sy = sin(euler[2]);
  Matrix<typename Derived::Scalar, 3, 3> m;
  m << cp * cy, -cr * sy + sr * sp * cy, sr * sy + cr * sp * cy, cp * sy, cr * cy + sr * sp * sy, -sr * cy + cr * sp * sy, -sp, sr * cp, cr * cp;
  return m;
}
template Matrix<typename Matrix<float, 3, 1>::Scalar, 3, 3> eulerToMat<Matrix<float, 3, 1>>(const MatrixBase<Matrix<float, 3, 1>>& euler);
template Matrix<typename Matrix<double, 3, 1>::Scalar, 3, 3> eulerToMat<Matrix<double, 3, 1>>(const MatrixBase<Matrix<double, 3, 1>>& euler);

template<typename Derived>
Matrix<typename Derived::Scalar, 3, 1>
quaternionToEuler(const QuaternionBase<Derived>& q)
{
  return matToEuler(quaternionToMat(q));
}
//template Matrix<typename Quaternion<float>::Scalar, 3, 1> quaternionToEuler<Quaternion<float>>(const QuaternionBase<Quaternion<float>>& q);
//template Matrix<typename Quaternion<double>::Scalar, 3, 1> quaternionToEuler<Quaternion<double>>(const QuaternionBase<Quaternion<double>>& q);

template<typename Derived>
Matrix<typename Derived::Scalar, 3, 1>
getOrientationDiff(const MatrixBase<Derived>& t1, const MatrixBase<Derived>& t2)
{
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
    (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4) ||
    (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3),
    THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  Matrix<Scalar, 3, 3> skew1t, skew2t, skew3t;
  Matrix<Scalar, 3, 3> skew1i, skew2i, skew3i;
  skew1t = MathsUtils::makeSkewMat((Matrix<Scalar, 3, 1>) t2.block(0, 0, 3, 1));
  skew2t = MathsUtils::makeSkewMat((Matrix<Scalar, 3, 1>) t2.block(0, 1, 3, 1));
  skew3t = MathsUtils::makeSkewMat((Matrix<Scalar, 3, 1>) t2.block(0, 2, 3, 1));
  skew1i = MathsUtils::makeSkewMat((Matrix<Scalar, 3, 1>) t1.block(0, 0, 3, 1));
  skew2i = MathsUtils::makeSkewMat((Matrix<Scalar, 3, 1>) t1.block(0, 1, 3, 1));
  skew3i = MathsUtils::makeSkewMat((Matrix<Scalar, 3, 1>) t1.block(0, 2, 3, 1));
  Matrix<Scalar, 3, 3> L = -0.5 * (skew1t * skew1i + skew2t * skew2i + skew3t * skew3i);
  Matrix<Scalar, 3, 1> orientError =
      0.5 * (skew1i * t2.block(0, 0, 3, 1) + skew2i * t2.block(0, 1, 3, 1) + skew3i * t2.block(0, 2, 3, 1));
  return L.inverse() * orientError;
}
template Matrix<typename Matrix<float, 4, 4>::Scalar, 3, 1> getOrientationDiff<Matrix<float, 4, 4>>(const MatrixBase<Matrix<float, 4, 4>>& t1, const MatrixBase<Matrix<float, 4, 4>>& t2);
template Matrix<typename Matrix<double, 4, 4>::Scalar, 3, 1> getOrientationDiff<Matrix<double, 4, 4>>(const MatrixBase<Matrix<double, 4, 4>>& t1, const MatrixBase<Matrix<double, 4, 4>>& t2);
template Matrix<typename Matrix<float, 3, 3>::Scalar, 3, 1> getOrientationDiff<Matrix<float, 3, 3>>(const MatrixBase<Matrix<float, 3, 3>>& t1, const MatrixBase<Matrix<float, 3, 3>>& t2);
template Matrix<typename Matrix<double, 3, 3>::Scalar, 3, 1> getOrientationDiff<Matrix<double, 3, 3>>(const MatrixBase<Matrix<double, 3, 3>>& t1, const MatrixBase<Matrix<double, 3, 3>>& t2);

template<typename Derived>
void makeTranslation(
  MatrixBase<Derived>& mat,
  const typename Derived::Scalar& x,
  const typename Derived::Scalar& y,
  const typename Derived::Scalar& z)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
    Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
    THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  mat.setIdentity();
  mat(0, 3) = x;
  mat(1, 3) = y;
  mat(2, 3) = z;
}
template void makeTranslation<Matrix<float, 4, 4> >(
  MatrixBase<Matrix<float, 4, 4> >& mat,
  const typename Matrix<float, 4, 4>::Scalar& x,
  const typename Matrix<float, 4, 4>::Scalar& y,
  const typename Matrix<float, 4, 4>::Scalar& z);
template void makeTranslation<Matrix<double, 4, 4> >(
  MatrixBase<Matrix<double, 4, 4> >& mat,
  const typename Matrix<double, 4, 4>::Scalar& x,
  const typename Matrix<double, 4, 4>::Scalar& y,
  const typename Matrix<double, 4, 4>::Scalar& z);

template<typename Derived>
void makeRotationZ(
  MatrixBase<Derived>& mat,
  const typename Derived::Scalar& angle)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
    (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4) ||
    (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3) ||
    (Derived::RowsAtCompileTime == 2 && Derived::ColsAtCompileTime == 2),
    THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  mat.setIdentity();
  auto ca = cos(angle);
  auto sa = sin(angle);
  mat(0, 0) = ca;
  mat(0, 1) = -sa;
  mat(1, 0) = sa;
  mat(1, 1) = ca;
}
template void makeRotationZ<Matrix<float, 4, 4> >(
  MatrixBase<Matrix<float, 4, 4> >& mat,
  const typename Matrix<float, 4, 4>::Scalar& angle);
template void makeRotationZ<Matrix<float, 3, 3> >(
  MatrixBase<Matrix<float, 3, 3> >& mat,
  const typename Matrix<float, 3, 3>::Scalar& angle);
template void makeRotationZ<Matrix<float, 2, 2> >(
  MatrixBase<Matrix<float, 2, 2> >& mat,
  const typename Matrix<float, 2, 2>::Scalar& angle);
template void makeRotationZ<Matrix<double, 4, 4> >(
  MatrixBase<Matrix<double, 4, 4> >& mat,
  const typename Matrix<double, 4, 4>::Scalar& angle);
template void makeRotationZ<Matrix<double, 3, 3> >(
  MatrixBase<Matrix<double, 3, 3> >& mat,
  const typename Matrix<double, 3, 3>::Scalar& angle);
template void makeRotationZ<Matrix<double, 2, 2> >(
  MatrixBase<Matrix<double, 2, 2> >& mat,
  const typename Matrix<double, 2, 2>::Scalar& angle);

template<typename Derived>
void makeRotationY(
  MatrixBase<Derived>& mat,
  const typename Derived::Scalar& angle)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
   (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4) ||
   (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3),
   THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  mat.setIdentity();
  auto ca = cos(angle);
  auto sa = sin(angle);
  mat(0, 0) = ca;
  mat(0, 2) = sa;
  mat(2, 0) = -sa;
  mat(2, 2) = ca;
}
template void makeRotationY<Matrix<float, 4, 4> >(
  MatrixBase<Matrix<float, 4, 4> >& mat,
  const typename Matrix<float, 4, 4>::Scalar& angle);
template void makeRotationY<Matrix<float, 3, 3> >(
  MatrixBase<Matrix<float, 3, 3> >& mat,
  const typename Matrix<float, 3, 3>::Scalar& angle);
template void makeRotationY<Matrix<double, 4, 4> >(
  MatrixBase<Matrix<double, 4, 4> >& mat,
  const typename Matrix<double, 4, 4>::Scalar& angle);
template void makeRotationY<Matrix<double, 3, 3> >(
  MatrixBase<Matrix<double, 3, 3> >& mat,
  const typename Matrix<double, 3, 3>::Scalar& angle);

template<typename Derived>
void makeRotationX(
  MatrixBase<Derived>& mat,
  const typename Derived::Scalar& angle)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
    (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4) ||
    (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3),
    THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  mat.setIdentity();
  auto ca = cos(angle);
  auto sa = sin(angle);
  mat(1, 1) = ca;
  mat(1, 2) = -sa;
  mat(2, 1) = sa;
  mat(2, 2) = ca;
}
template void makeRotationX<Matrix<float, 4, 4> >(
  MatrixBase<Matrix<float, 4, 4> >& mat,
  const typename Matrix<float, 4, 4>::Scalar& angle);
template void makeRotationX<Matrix<float, 3, 3> >(
  MatrixBase<Matrix<float, 3, 3> >& mat,
  const typename Matrix<float, 3, 3>::Scalar& angle);
template void makeRotationX<Matrix<double, 4, 4> >(
  MatrixBase<Matrix<double, 4, 4> >& mat,
  const typename Matrix<double, 4, 4>::Scalar& angle);
template void makeRotationX<Matrix<double, 3, 3> >(
  MatrixBase<Matrix<double, 3, 3> >& mat,
  const typename Matrix<double, 3, 3>::Scalar& angle);

template<typename Derived>
void makeRotationXYZ(
  MatrixBase<Derived>& mat,
  const typename Derived::Scalar& xAngle,
  const typename Derived::Scalar& yAngle,
  const typename Derived::Scalar& zAngle)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
    (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4) ||
    (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3),
    THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  Derived rX, rY, rZ;
  makeRotationX(rX, xAngle);
  makeRotationY(rY, yAngle);
  makeRotationZ(rZ, zAngle);
  rX *= rY;
  rX *= rZ;
  mat = rX;
}
template void makeRotationXYZ<Matrix<float, 4, 4> >(
  MatrixBase<Matrix<float, 4, 4> >& mat,
  const typename Matrix<float, 4, 4>::Scalar& xAngle,
  const typename Matrix<float, 4, 4>::Scalar& yAngle,
  const typename Matrix<float, 4, 4>::Scalar& zAngle);
template void makeRotationXYZ<Matrix<float, 3, 3> >(
  MatrixBase<Matrix<float, 3, 3> >& mat,
  const typename Matrix<float, 3, 3>::Scalar& xAngle,
  const typename Matrix<float, 3, 3>::Scalar& yAngle,
  const typename Matrix<float, 3, 3>::Scalar& zAngle);
template void makeRotationXYZ<Matrix<double, 4, 4> >(
  MatrixBase<Matrix<double, 4, 4> >& mat,
  const typename Matrix<double, 4, 4>::Scalar& xAngle,
  const typename Matrix<double, 4, 4>::Scalar& yAngle,
  const typename Matrix<double, 4, 4>::Scalar& zAngle);
template void makeRotationXYZ<Matrix<double, 3, 3> >(
  MatrixBase<Matrix<double, 3, 3> >& mat,
  const typename Matrix<double, 3, 3>::Scalar& xAngle,
  const typename Matrix<double, 3, 3>::Scalar& yAngle,
  const typename Matrix<double, 3, 3>::Scalar& zAngle);

template<typename Derived>
void makeRotationZYX(
  MatrixBase<Derived>& mat,
  const typename Derived::Scalar& zAngle,
  const typename Derived::Scalar& yAngle,
  const typename Derived::Scalar& xAngle)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
    (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4) ||
    (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3),
    THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  Derived rX, rY, rZ;
  makeRotationX(rX, xAngle);
  makeRotationY(rY, yAngle);
  makeRotationZ(rZ, zAngle);
  rZ *= rY;
  rZ *= rX;
  mat = rZ;
}
template void makeRotationZYX<Matrix<float, 4, 4> >(
  MatrixBase<Matrix<float, 4, 4> >& mat,
  const typename Matrix<float, 4, 4>::Scalar& zAngle,
  const typename Matrix<float, 4, 4>::Scalar& yAngle,
  const typename Matrix<float, 4, 4>::Scalar& xAngle);
template void makeRotationZYX<Matrix<float, 3, 3> >(
  MatrixBase<Matrix<float, 3, 3> >& mat,
  const typename Matrix<float, 3, 3>::Scalar& zAngle,
  const typename Matrix<float, 3, 3>::Scalar& yAngle,
  const typename Matrix<float, 3, 3>::Scalar& xAngle);
template void makeRotationZYX<Matrix<double, 4, 4> >(
  MatrixBase<Matrix<double, 4, 4> >& mat,
  const typename Matrix<double, 4, 4>::Scalar& zAngle,
  const typename Matrix<double, 4, 4>::Scalar& yAngle,
  const typename Matrix<double, 4, 4>::Scalar& xAngle);
template void makeRotationZYX<Matrix<double, 3, 3> >(
  MatrixBase<Matrix<double, 3, 3> >& mat,
  const typename Matrix<double, 3, 3>::Scalar& zAngle,
  const typename Matrix<double, 3, 3>::Scalar& yAngle,
  const typename Matrix<double, 3, 3>::Scalar& xAngle);

template<typename Derived>
Matrix<typename Derived::Scalar, 4, 4> makeTransformation(
    const MatrixBase<Derived>& rot,
    const typename Derived::Scalar& x,
    const typename Derived::Scalar& y,
    const typename Derived::Scalar& z)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
    Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3,
    THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  Matrix<typename Derived::Scalar, 4, 4> mat;
  mat.setIdentity();
  mat.block(0, 0, 3, 3) = rot;
  mat(0, 3) = x;
  mat(1, 3) = y;
  mat(2, 3) = z;
  return mat;
}
template Matrix<typename Matrix<float, 3, 3>::Scalar, 4, 4> makeTransformation(
    const MatrixBase<Matrix<float, 3, 3> >& rot,
    const typename Matrix<float, 3, 3>::Scalar& x,
    const typename Matrix<float, 3, 3>::Scalar& y,
    const typename Matrix<float, 3, 3>::Scalar& z);
template Matrix<typename Matrix<double, 3, 3>::Scalar, 4, 4> makeTransformation(
    const MatrixBase<Matrix<double, 3, 3> >& rot,
    const typename Matrix<double, 3, 3>::Scalar& x,
    const typename Matrix<double, 3, 3>::Scalar& y,
    const typename Matrix<double, 3, 3>::Scalar& z);

template<typename Derived>
Matrix<typename Derived::Scalar, 3, 1> getEulerAngles(const MatrixBase<Derived>& mat)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
    (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4) ||
    (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3),
    THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  Matrix<typename Derived::Scalar, 3, 1> euler;
  Matrix<typename Derived::Scalar, 3, 3> rot = mat.block(0, 0, 3, 3);
  euler = rot.eulerAngles(2, 1, 0);
  return euler;
}

template<typename Derived>
Derived getTInverse(const MatrixBase<Derived>& mat)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
    Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
    THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  auto Rt = mat.block(0, 0, 3, 3).transpose();
  auto t = mat.block(0, 3, 3, 1);
  Derived inv;
  inv.setIdentity();
  inv.block(0, 0, 3, 3) = Rt;
  inv.block(0, 3, 3, 1) = -Rt * t;
  return inv;
}
template Matrix<float, 4, 4> getTInverse<Matrix<float, 4, 4>>(const MatrixBase<Matrix<float, 4, 4>>& mat);
template Matrix<double, 4, 4> getTInverse<Matrix<double, 4, 4>>(const MatrixBase<Matrix<double, 4, 4>>& mat);

template<typename Derived>
void makeDHTransformation(
  MatrixBase<Derived>& mat,
  const typename Derived::Scalar& a,
  const typename Derived::Scalar& alpha,
  const typename Derived::Scalar& d,
  const typename Derived::Scalar& theta)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
    Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
    THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  mat.setIdentity();
  auto ct = cos(theta);
  auto st = sin(theta);
  auto ca = cos(alpha);
  auto sa = sin(alpha);
  mat(0, 0) = ct;
  mat(0, 1) = -st;
  mat(0, 2) = 0;
  mat(0, 3) = a;
  mat(1, 0) = st * ca;
  mat(1, 1) = ct * ca;
  mat(1, 2) = -sa;
  mat(1, 3) = -sa * d;
  mat(2, 0) = st * sa;
  mat(2, 1) = ct * sa;
  mat(2, 2) = ca;
  mat(2, 3) = ca * d;
}
template void makeDHTransformation<Matrix<float, 4, 4>>(
  MatrixBase<Matrix<float, 4, 4>>& mat,
  const typename Matrix<float, 4, 4>::Scalar& a,
  const typename Matrix<float, 4, 4>::Scalar& alpha,
  const typename Matrix<float, 4, 4>::Scalar& d,
  const typename Matrix<float, 4, 4>::Scalar& theta);
template void makeDHTransformation<Matrix<double, 4, 4>>(
  MatrixBase<Matrix<double, 4, 4>>& mat,
  const typename Matrix<double, 4, 4>::Scalar& a,
  const typename Matrix<double, 4, 4>::Scalar& alpha,
  const typename Matrix<double, 4, 4>::Scalar& d,
  const typename Matrix<double, 4, 4>::Scalar& theta);

template<typename Derived>
Matrix<typename Derived::Scalar, 3, 1> transformVector(
  const MatrixBase<Derived>& mat,
  const Matrix<typename Derived::Scalar, 3, 1> & vec)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
    Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
    THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  Matrix<typename Derived::Scalar, 4, 1> temp;
  temp << vec, 1;
  temp = mat * temp;
  return temp.segment(0, 3);
}
template Matrix<typename Matrix<float, 4, 4>::Scalar, 3, 1> transformVector(
  const MatrixBase<Matrix<float, 4, 4>>& mat,
  const Matrix<typename Matrix<float, 4, 4>::Scalar, 3, 1> & vec);
template Matrix<typename Matrix<double, 4, 4>::Scalar, 3, 1> transformVector(
  const MatrixBase<Matrix<double, 4, 4>>& mat,
  const Matrix<typename Matrix<double, 4, 4>::Scalar, 3, 1> & vec);

template<typename Derived>
Matrix<typename Derived::Scalar, 3, 3> makeSkewMat(
  const MatrixBase<Derived>& vec)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
    Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
    THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  Matrix<typename Derived::Scalar, 3, 3> res;
  res.setZero();
  res(0, 1) = -vec[2];
  res(0, 2) = vec[1];
  res(1, 0) = vec[2];
  res(1, 2) = -vec[0];
  res(2, 0) = -vec[1];
  res(2, 1) = vec[0];
  return res;
}
template Matrix<typename Matrix<float, 3, 1>::Scalar, 3, 3> makeSkewMat<Matrix<float, 3, 1>>(const MatrixBase<Matrix<float, 3, 1>>& vec);
template Matrix<typename Matrix<double, 3, 1>::Scalar, 3, 3> makeSkewMat<Matrix<double, 3, 1>>(const MatrixBase<Matrix<double, 3, 1>>& vec);

template<typename Derived>
Derived mirrorTransformation(const MatrixBase<Derived>& mat)
{
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(
    Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
    THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  Derived tmp = mat;
  tmp(0, 1) = -mat(0, 1);
  tmp(1, 0) = -mat(1, 0);
  tmp(1, 2) = -mat(1, 2);
  tmp(2, 1) = -mat(2, 1);
  tmp(1, 3) = -mat(1, 3);
  return tmp;
}
template Matrix<float, 4, 4> mirrorTransformation<Matrix<float, 4, 4>>(const MatrixBase<Matrix<float, 4, 4>>& mat);
template Matrix<double, 4, 4> mirrorTransformation<Matrix<double, 4, 4>>(const MatrixBase<Matrix<double, 4, 4>>& mat);

template<typename Derived>
bool almostEqual(
  const Derived& first, const Derived& second,
  const typename Derived::Scalar tol)
{
  typename Derived::Scalar angleNorm =
    (getEulerAngles(first) - getEulerAngles(second)).norm();
  typename Derived::Scalar posNorm =
    (first.block(0, 3, 3, 1) - second.block(0,3,3,1)).norm();
  return (angleNorm < tol && posNorm < 1e-3);
}
template bool almostEqual<Matrix<float, 4, 4>>(
  const Matrix<float, 4, 4>& first,
  const Matrix<float, 4, 4>& second,
  const typename Matrix<float, 4, 4>::Scalar tol = typename Matrix<float, 4, 4>::Scalar(0.5));
template bool almostEqual<Matrix<double, 4, 4>>(
  const Matrix<double, 4, 4>& first,
  const Matrix<double, 4, 4>& second,
  const typename Matrix<double, 4, 4>::Scalar tol = typename Matrix<double, 4, 4>::Scalar(0.5));


template<typename Scalar>
bool almostEqual(const Scalar& first, const Scalar& second, const Scalar tol)
{
  return std::fabs(first - second) < tol;
}
template bool almostEqual<float>(
  const float& first, const float& second, const float tol);
template bool almostEqual<double>(
  const double& first, const double& second, const double tol);

template<typename Scalar>
Scalar safeAcos(Scalar var)
{
  if (var < -1.0) var = -1.0;
  else if (var > 1.0) var = 1.0;
  return acos(var);
}
template float safeAcos<float>(float var);
template double safeAcos<double>(double var);

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, Dynamic, Dynamic>
pseudoInverse(
  const MatrixBase<Derived>& mat,
  const typename Derived::Scalar tolerance)
{
typedef typename Derived::Scalar Scalar;
  auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto &singularValues = svd.singularValues();
  Eigen::Matrix<Scalar, Derived::ColsAtCompileTime, Derived::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
  singularValuesInv.setZero();
  for (unsigned int i = 0; i < singularValues.size(); ++i) {
    if (singularValues(i) > tolerance)
      singularValuesInv(i, i) = Scalar{1} / singularValues(i);
    else
      singularValuesInv(i, i) = Scalar{0};
  }
  return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}
template Eigen::Matrix<typename Matrix<float, Dynamic, Dynamic>::Scalar, Dynamic, Dynamic>
  pseudoInverse<Matrix<float, Dynamic, Dynamic>>(
    const MatrixBase<Matrix<float, Dynamic, Dynamic> >& mat,
    const typename Matrix<float, Dynamic, Dynamic>::Scalar tolerance = typename Matrix<float, Dynamic, Dynamic>::Scalar(1e-4));
template Eigen::Matrix<typename Matrix<double, Dynamic, Dynamic>::Scalar, Dynamic, Dynamic>
  pseudoInverse<Matrix<double, Dynamic, Dynamic>>(
    const MatrixBase<Matrix<double, Dynamic, Dynamic> >& mat,
    const typename Matrix<double, Dynamic, Dynamic>::Scalar tolerance = typename Matrix<double, Dynamic, Dynamic>::Scalar(1e-4));
template Eigen::Matrix<typename Matrix<float, 3, 4>::Scalar, Dynamic, Dynamic>
pseudoInverse<Matrix<float, 3, 4>>(
 const MatrixBase<Matrix<float, 3, 4> >& mat,
 const typename Matrix<float, 3, 4>::Scalar tolerance = typename Matrix<float, 3, 4>::Scalar(1e-4));


template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, Dynamic, 1>
pseudoInverseSolve(
  const MatrixBase<Derived>& mat,
  const Matrix<typename Derived::Scalar, Dynamic, 1> &rhs)
{
  JacobiSVD<Derived> svd(mat, ComputeThinU | ComputeThinV);
  return svd.solve(rhs);
}
template Eigen::Matrix<typename Matrix<float, Dynamic, Dynamic>::Scalar, Dynamic, 1>
  pseudoInverseSolve<Matrix<float, Dynamic, Dynamic> >(
    const MatrixBase<Matrix<float, Dynamic, Dynamic>>& mat,
    const Matrix<typename Matrix<float, Dynamic, Dynamic>::Scalar, Dynamic, 1> &rhs);
template Eigen::Matrix<typename Matrix<double, Dynamic, Dynamic>::Scalar, Dynamic, 1>
  pseudoInverseSolve<Matrix<double, Dynamic, Dynamic> >(
    const MatrixBase<Matrix<double, Dynamic, Dynamic>>& mat,
    const Matrix<typename Matrix<double, Dynamic, Dynamic>::Scalar, Dynamic, 1> &rhs);

template<typename Scalar>
vector<Scalar> operator+(const vector<Scalar>& first, const vector<Scalar>& second)
{
  ASSERT(first.size() == second.size());
  vector<Scalar> result;
  result.reserve(first.size());
  transform(first.begin(), first.end(), second.begin(), back_inserter(result), plus<Scalar>());
  return result;
}

template<typename Scalar>
vector<Scalar>
operator-(const vector<Scalar>& first, const vector<Scalar>& second)
{
  ASSERT(first.size() == second.size());
  vector<Scalar> result;
  result.reserve(first.size());
  transform(first.begin(), first.end(), second.begin(), back_inserter(result), minus<Scalar>());
  return result;
}

template<typename Scalar>
vector<Scalar>
operator*(const vector<Scalar>& vec, const Scalar& constant)
{
  vector<Scalar> result;
  result.reserve(vec.size());
  transform(vec.begin(), vec.end(), result.begin(), bind2nd(multiplies<Scalar>(), constant));
  return result;
}

#ifndef V6_CROSS_BUILD
template<typename Scalar>
cv::Point_<Scalar> operator/(const cv::Point_<Scalar>& p, const Scalar value)
{
 return cv::Point_<Scalar> (p.x / value, p.y / value);
}

template cv::Point_<float> operator/(const cv::Point_<float>& p, const float value);
template cv::Point_<double> operator/(const cv::Point_<double>& p, const double value);
#endif

} ///< MathsUtils
