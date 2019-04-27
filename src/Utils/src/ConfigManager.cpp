/**
 * @file Utils/src/ConfigManager.cpp
 *
 * This file implements the class ConfigManager.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include "Utils/include/ConfigManager.h"
#ifndef VISUALIZER_BUILD
#include "Utils/include/PrintUtils.h"
#endif

string ConfigManager::pbConfigsDirPath;
string ConfigManager::mbConfigsDirPath;
string ConfigManager::gbConfigsDirPath;
#if defined(MODULE_IS_REMOTE) || defined(MODULE_IS_LOCAL_SIMULATED) || defined(VISUALIZER_BUILD)
string ConfigManager::configDirPath = GET_STRINGS(ROOT_DIR "/../config/");
string ConfigManager::commonConfigDirPath = GET_STRINGS(ROOT_DIR "/../config/Common/");
string ConfigManager::logsDirPath = GET_STRINGS(ROOT_DIR "/../logs/");
string ConfigManager::robotDirPath = "";
#else
string ConfigManager::configDirPath = "/home/nao/team_nust/config/";
string ConfigManager::commonConfigDirPath = "/home/nao/team_nust/config/";
string ConfigManager::logsDirPath = "/home/nao/team_nust/logs/";
string ConfigManager::robotDirPath = "";
#endif

void ConfigManager::parseFile(const string& cfgFile)
{
  try {
    this->cfgFile = cfgFile;
    boost::property_tree::ini_parser::read_ini(cfgFile, iniConfig);
  } catch (boost::exception &e) {
    #ifndef VISUALIZER_BUILD
    LOG_EXCEPTION("Error while reading file: " << cfgFile);
    #endif
  }
}

void ConfigManager::createDirs()
{
  if (!boost::filesystem::exists(configDirPath)) {
    boost::filesystem::create_directory(configDirPath);
  }

  if (!boost::filesystem::exists(logsDirPath)) {
    boost::filesystem::create_directory(logsDirPath);
  }
}

template<typename valueType>
valueType ConfigManager::getValueOfKey(const string& key) const
{
  valueType var = valueType();
  try {
    var = iniConfig.get < valueType > (key);
  } catch (boost::exception &e) {
    #ifndef VISUALIZER_BUILD
    LOG_EXCEPTION(
      "Error while reading the key: " << key << " from file: " << cfgFile);
    #endif
  }
  return var;
}

void ConfigManager::setDirPaths(const string& robotDir)
{
  pbConfigsDirPath = configDirPath + "BehaviorConfigs/PBConfigs/";
  mbConfigsDirPath = configDirPath + "BehaviorConfigs/MBConfigs/";
  gbConfigsDirPath = configDirPath + "BehaviorConfigs/GBConfigs/";
  robotDirPath = configDirPath + robotDir;
  if (!boost::filesystem::exists(logsDirPath)) {
    boost::filesystem::create_directory(logsDirPath);
  }
  if (!boost::filesystem::exists(logsDirPath + "Robots")) {
    boost::filesystem::create_directory(logsDirPath + "Robots");
  }
  logsDirPath = logsDirPath + robotDir;
}

template bool ConfigManager::getValueOfKey<bool>(const string& key) const;
template int ConfigManager::getValueOfKey<int>(const string& key) const;
template unsigned ConfigManager::getValueOfKey<unsigned>(const string& key) const;
template float ConfigManager::getValueOfKey<float>(const string& key) const;
template double ConfigManager::getValueOfKey<double>(const string& key) const;
template string ConfigManager::getValueOfKey<string>(const string& key) const;
