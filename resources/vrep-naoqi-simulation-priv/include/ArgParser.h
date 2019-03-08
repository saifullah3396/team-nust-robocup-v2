/**
 * @file Utils/include/ArgParser.h
 *
 * This file declares the class ArgParser.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

using namespace std;

/**
 * @class ArgParser
 * @brief A class for argument parsing.
 */
class ArgParser {
public:
	/**
	 * Gets the value of the option for given string
	 */
	static char* getCmdOption(
		char ** begin, char ** end, const string & option)
	{
    char ** itr = find(begin, end, option);
    if (itr != end && ++itr != end)
    {
      return *itr;
    }
		return 0;
	}

	/**
	 * Checks if the given option exists
	 */
	static bool cmdOptionExists(
		char** begin, char** end, const string& option)
	{
		return find(begin, end, option) != end;
	}
};

