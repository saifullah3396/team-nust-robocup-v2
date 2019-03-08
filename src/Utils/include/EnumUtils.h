/**
 * @file Utils/include/EnumUtils.h
 *
 * This file defines the utility functions for enums
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 07 Feb 2017
 */

#pragma once

#include <type_traits>

#define DECLARE_SPECIALIZED_ENUM(Name) \
  Name operator++(Name& j); \
   \
  Name operator*(Name j); \
   \
  Name begin(Name rj); \
   \
  Name end(Name rj);

#define DEFINE_SPECIALIZED_ENUM(Name) \
  Name operator++(Name& j) \
  { \
      return j = (Name)(toUType(j) + 1); \
  } \
   \
  Name operator*(Name j) \
  { \
      return j; \
  } \
   \
  Name begin(Name rj) \
  { \
      return Name::first; \
  } \
   \
  Name end(Name rj) \
  { \
      Name l = Name::last; \
      return ++l; \
  }

template <typename E>
constexpr typename std::underlying_type<E>::type toUType(const E& e)
{
   return static_cast<typename std::underlying_type<E>::type>(e);
}
