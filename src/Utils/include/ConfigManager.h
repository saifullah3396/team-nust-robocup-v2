/**
 * @file Utils/include/ConfigManager.h
 *
 * This file declares the class ConfigManager
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <boost/exception/diagnostic_information.hpp> 
#include <boost/exception/all.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include "Utils/include/PrintUtils.h"
#include "Utils/include/VariadicMacros.h"

using namespace std;

/**
 * @class ConfigManager
 * @brief Class for parsing the contents of a config file.
 */
class ConfigManager
{
public:
  ConfigManager() = default;
  ConfigManager(const ConfigManager&) = delete;
  ConfigManager(ConfigManager&&) = delete;
  ConfigManager& operator=(const ConfigManager&) & = delete;
  ConfigManager& operator=(ConfigManager&&) & = delete;

  /**
   * @brief ~ConfigManager Destructor
   */
  virtual ~ConfigManager() {}

  /**
   * @brief parseFile Parses an .ini cfg file using boost property trees
   * @param cfgFile File path
   */
  void parseFile(const string& cfgFile);

  /**
   * @brief createDirs Creates the logs and configuration directories
   */
  static void createDirs();

  /**
   * @brief getValueOfKey Returns the value of the given key
   * @param key Name of the key
   * @return valueType Value of the key
   */
  template<typename valueType>
  valueType	getValueOfKey(const string& key) const;

  //! Getters
  static string getConfigDirPath() { return configDirPath; }
  static string getCommonConfigDirPath() { return commonConfigDirPath; }
  static string getLogsDirPath() { return logsDirPath; }
  static string getRobotDirPath() { return robotDirPath; }
  static string getPBConfigsPath() { return pbConfigsDirPath; }
  static string getMBConfigsPath() { return mbConfigsDirPath; }
  static string getGBConfigsPath() { return gbConfigsDirPath; }

  //! Sets the path to the directory of configuration/logs/robot files.
  static void setDirPaths(const string& robotDir);
private:
  static string configDirPath; //! Path to the directory of configuration files
  static string commonConfigDirPath; //! Path to the directory of configuration files that are common to all robots
  static string logsDirPath; //! Path to the directory of log  files
  static string robotDirPath; //! Path to robot directory.
  static string pbConfigsDirPath; //! Path to planning behavior configurations
  static string mbConfigsDirPath; //! Path to motion behavior configurations
  static string gbConfigsDirPath; //! Path to static behavior configurations
  boost::property_tree::ptree iniConfig; //! Property tree filled with parsed data
  string cfgFile; //! The config file path
};
