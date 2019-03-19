/**
 * @file Utils/include/DataHolders/DataHolder.h
 *
 * This file declares and implements the struct DataHolder
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 15 Jan 2018
 */

#pragma once

#include "Utils/include/VariadicMacros.h"
#ifndef VISUALIZER_BUILD
#include "Utils/include/PrintUtils.h"
#include "Utils/include/JsonUtils.h"
#else
#include <json/json.h>
#endif


#define PRINT_VAR(NAME, VALUE) TO_STRING(NAME) << ": " << VALUE <<
#define PRINT_VARS__(NAME, VALUE) ",\n\t" << TO_STRING(NAME) << ": " << VALUE <<
#define PRINT_VARS_(NAME_VALUE) \
  PRINT_VARS__( \
    M_GET_ELEM(0,UNPAREN(NAME_VALUE)), \
    M_GET_ELEM(1,UNPAREN(NAME_VALUE)) \
  ) \

#define PRINT_DATA(NAME, FIRST, ...) \
  LOG_INFO(\
    "Printing " << TO_STRING(NAME) << " Information:\n{" << \
    "\n\t" << PRINT_VAR(M_GET_ELEM(0,UNPAREN(FIRST)), M_GET_ELEM(1,UNPAREN(FIRST))) \
    FOR_EACH(PRINT_VARS_, ##__VA_ARGS__) \
    "\n}" \
  );

/**
 * @struct DataHolder
 * @brief Base struct for all data holders
 */
struct DataHolder
{
  DataHolder() = default;
  DataHolder(const DataHolder&) = default;
  DataHolder(DataHolder&&) = default;
  DataHolder& operator=(const DataHolder&) & = default;
  DataHolder& operator=(DataHolder&&) & = default;
  virtual ~DataHolder() {}

  /**
   * @brief print Self-explanatory
   */
  virtual void print() const {}

  /**
   * @brief print Self-explanatory
   */
  virtual Json::Value getJson() const { return Json::nullValue; }
};
