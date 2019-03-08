/**
 * @file Utils/src/ArgParseUtils.cpp
 *
 * This file implements the helper functions associated with argument parsing
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#include <algorithm>
#include "Utils/include/ArgParseUtils.h"
#include "Utils/include/Exceptions/ArgParseException.h"
#include "Utils/include/PrintUtils.h"

namespace ArgParseUtils
{
  /**
   * Gets the value of the option for given string
   */
  char* getCmdOption(
    char ** begin, char ** end, const string & option)
  {
    try {
      char ** itr = std::find(begin, end, option);
      if (itr != end && ++itr != end)
      {
        return *itr;
      } else {
        throw
          ArgParseException(
            "No argument specified for option: " + option,
            false,
            EXC_NO_ARG_SPECIFIED
          );
      }
    } catch (ArgParseException& e) {
      LOG_EXCEPTION(e.what());
    }
    return 0;
  }

  /**
   * Checks if the given option exists
   */
  bool cmdOptionExists(
    char** begin, char** end, const string& option)
  {
    return find(begin, end, option) != end;
  }
}

