/**
 * @file UserCommModule/include/UserCommTypes.h
 *
 * This file declares the enumeration UserCommTypes
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

/**
 * Enumeration for all the communication servers to be started
 *
 * @enum UserCommTypes
 */
enum class UserCommTypes : unsigned int {
  dataConn,
  imageConn,
  count
};
