/**
 * @file Utils/include/ArgParseUtils.h
 *
 * This file declares the helper functions associated with argument parsing
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#pragma once

#include <string>

namespace ArgParseUtils
{
  /**
   * @brief getCmdOption Gets the value of the option for given string
   * @param begin String beginning
   * @param end String ending
   * @param option Arugment option
   * @return
   */
  char* getCmdOption(
    char** begin, char** end, const std::string& option);

  /**
   * @brief cmdOptionExists Checks if the given option exists
   * @param begin String beginning
   * @param end String ending
   * @param option Arugment option
   * @return
   */
  bool cmdOptionExists(
    char** begin, char** end, const std::string& option);
}

