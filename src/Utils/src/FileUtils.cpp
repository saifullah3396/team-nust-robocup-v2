/**
 * @file Utils/src/FileUtils.cpp
 *
 * This file implements the file handling utility functions
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include <boost/filesystem.hpp>
#include <iostream>
#include <string>

namespace FileUtils
{
  /*void bool find_file(
    const path & dir_path,         // in this directory,
    const std::string & file_name, // search for this name,
    path & path_found)             // placing path here if found
  {
    if (!exists(dir_path))
      return false;

    directory_iterator end_itr; // default construction yields past-the-end

    for (directory_iterator itr(dir_path); itr != end_itr; ++itr)
    {
      if (is_directory(itr->status()))
      {
        if (find_file(itr->path(), file_name, path_found))
          return true;
      }
      else if (itr->leaf() == file_name) // see below
      {
        path_found = itr->path();
        return true;
      }
    }
    return false;
  }*/

  int getFileCnt(const std::string& dir)
  {
    boost::filesystem::path dirPath(dir);
    if (!boost::filesystem::exists(dirPath))
      return -1;
    int cnt = 0;
    boost::filesystem::directory_iterator endItr;
    for (boost::filesystem::directory_iterator itr(dirPath); itr != endItr; ++itr)
      cnt++;
    return cnt;
  }
}

