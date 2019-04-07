/**
 * @file MotionModule/include/BalanceModule/ZmpRef.h
 *
 * This file defines the struct ZmpRef
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>

/**
 * @struct ZmpRef
 * @brief Desired zmp x-y references in future
 */
template <typename Scalar>
struct ZmpRef
{
  ZmpRef(const ZmpRef&) = default;
  ZmpRef(const unsigned& nReferences) {
    if (nReferences > 0) {
      x.set_capacity(nReferences);
      y.set_capacity(nReferences);
    }
  }
  ZmpRef(ZmpRef&&) = default;
  ZmpRef& operator=(const ZmpRef&) & = default;
  ZmpRef& operator=(ZmpRef&&) & = default;
  virtual ~ZmpRef() {}

  bool setNReferences(const unsigned& nReferences) {
    if (nReferences > 0) {
      x.set_capacity(nReferences);
      y.set_capacity(nReferences);
    } else {
      return false;
    }
  }

  ///< x references of fixed capacity
  boost::circular_buffer<Scalar> x;

  ///< y references of fixed capacity
  boost::circular_buffer<Scalar> y;
};
