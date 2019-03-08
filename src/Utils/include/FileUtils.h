/**
 * @file Utils/include/FileUtils.h
 *
 * This file declares the file handling utility functions
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include "boost/filesystem.hpp"
#include <iostream>

namespace FileUtils
{
  #define TRUNC_LOG_FILE(name, str) \
    name.open( \
      (ConfigManager::getLogsDirPath() + string(str)).c_str(),\
      std::ofstream::out | std::ofstream::trunc\
    );\
    name.close();\

  #define OPEN_LOG_FILE(name, str) \
    name.open( \
      (ConfigManager::getLogsDirPath() + string(str)).c_str(),\
      std::ofstream::out | std::ofstream::app \
    );\

  /*
    void bool find_file(
    const path& dir_path,         // in this directory,
    const std::string & file_name, // search for this name,
    path & path_found);
  */

  /**
   * @brief getFileCnt Returns the number of files in a directory
   * @param dir Directory path
   * @return int
   */
  int getFileCnt(const string& dir);
}
