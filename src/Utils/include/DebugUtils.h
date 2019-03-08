/**
 * @file Utils/include/DebugUtls.h
 *
 * This file defines the macro for handling debugging
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 07 Feb 2017
 */

#pragma once

#include <assert.h>

#define ASSERT(condition) \
{ \
  assert(condition); \
}

#define ASSERT_MSG(condition, msg) \
{ \
  assert(condition && #msg); \
}

