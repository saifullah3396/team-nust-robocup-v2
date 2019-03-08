/**
 * @file Utils/include/DataUtils.h
 *
 * This file declares the helper functions for data handling
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace Eigen;
using namespace std;
using namespace cv;

/**
 * @class DataUtils
 * @brief Class that provides functions for variables data type
 *   conversions or handling.
 */
namespace DataUtils
{
  /**
   * @brief varToString Converts a given variable to string.
   * @param var Input variable
   * @return string Output string
   */
  template<typename T>
  string varToString(const T& var);

  /**
   * @brief stringToVar Converts a given string to variable
   * @param str Input string
   */
  template<typename T>
  void stringToVar(const string& str, T& var);

  /**
   * @brief stringToVar Converts a given string to variable.
   * @param str Input string
   */
  template<typename T>
  void stringToVar(const string& str, vector<T>& var);

  /**
   * @brief bytesToHexString Converts a given byte buffer to hex string.
   * @param buffer Input byte buffer
   * @param size Size of the buffer
   * @return string Buffer contents as hex string
   */
  string bytesToHexString(unsigned char* buffer, const int& size = -1);

  /**
   * @brief convertBytesToString Converts a given byte buffer to string.
   * @param buffer Input byte buffer
   * @param size Size of the buffer
   * @return string Buffer contents
   */
  string convertBytesToString(const unsigned char*& buffer, const int& size);

  /**
   * @brief splitString Returns a vector of splitted strings with respect to the
   *   given delimiter.
   * @param string Input string
   * @param delim Delimiter
   * @param elems Vector of splitted string components
   * @return vector Vector of splitted string components
   */
  vector<string>& splitString(
    const string& s, const char& delim, vector<string>& elems);

  /**
   * @brief splitString Returns a vector of splitted strings with respect to the
   *   given delimiter.
   * @param string Input string
   * @param delim Delimiter
   * @return vector Vector of splitted string components
   */
  vector<string> splitString(const string& s, const char& delim);

  /**
   * @brief getCurrentDate Returns the current time and date
   * @return string
   */
  string getCurrentDateTime();
}
