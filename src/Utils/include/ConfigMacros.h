/**
 * @file Utils/ConfigMacros.h
 *
 * The file defines the macro GET_CONFIG().
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#ifndef _CONFIG_MACROS_H_
#define _CONFIG_MACROS_H_

#include "ConfigManager.h"
#ifndef VISUALIZER_BUILD
#include "Utils/include/PrintUtils.h"
#else
#include <iostream>
#endif

/**
 * \def GET_VAR_3(TYPE, VAR_NAME, VAR)
 * @brief This macro gets the value from the given variable file name
 *   and saves it in the given variable.
 * @param TYPE: The variable type.
 * @param VAR_NAME: The variable name in file.
 * @param VAR: The output variable name.
 */
#define GET_VAR_3(TYPE, VAR_NAME, VAR) \
  VAR = configManager.getValueOfKey<TYPE>(#VAR_NAME);

/**
 * \def GET_VAR_2(TYPE, VAR_NAME, VAR)
 * @brief This macro gets the value from the given variable file name
 *   and calls GET_VAR_2.
 * @param TYPE: The variable type.
 * @param VAR_NAME: The variable name in file.
 * @param VAR: The output variable name.
 */
#define GET_VAR_2(TYPE, VAR_NAME, VAR) \
  GET_VAR_3(TYPE, VAR_NAME, VAR)

/**
 * \def GET_VAR_1(TYPE_VAR)
 * @brief This macro removes the parenthesis from the given variable
 *   data and sets its elements to be extracted.
 * @param TYPE_VAR: The variable type, its name in file, and output
 *   variable.
 */
#define GET_VAR_1(TYPE_VAR) \
  GET_VAR_2( \
          M_GET_ELEM(0,UNPAREN(TYPE_VAR)), \
          M_GET_ELEM(1,UNPAREN(TYPE_VAR)), \
          M_GET_ELEM(2,UNPAREN(TYPE_VAR)) \
  )

/**
 * \def GET_CONFIG(configFile, ...)
 * @brief This macro takes the file name and the variables to be
 *   extracted from the file.
 * @param configFile: Name of the configuration file.
 * @param ... : Paranthesis enclosed array of variables to be
 *   defined (type, variableNameInFile, variable), ...
 */
#ifndef VISUALIZER_BUILD
  #define GET_CONFIG(configFile, ...) \
  { \
    ConfigManager configManager; \
    string cDirPath = string(ConfigManager::getConfigDirPath() + configFile + string(".ini")); \
    if (boost::filesystem::exists(cDirPath)) { \
      configManager.parseFile(cDirPath); \
      FOR_EACH(GET_VAR_1, __VA_ARGS__) \
    } else { \
      cDirPath = string(ConfigManager::getCommonConfigDirPath() + configFile + string(".ini")); \
      if (boost::filesystem::exists(cDirPath)) { \
        configManager.parseFile(cDirPath); \
        FOR_EACH(GET_VAR_1, __VA_ARGS__) \
      } else { \
        LOG_ERROR("Configuration file: " << cDirPath <<" does not exist."); \
      } \
    } \
  }
#else
  #define GET_CONFIG(configFile, ...) \
  { \
    ConfigManager configManager; \
    string cDirPath = string(ConfigManager::getConfigDirPath() + configFile + string(".ini")); \
    std::cout << cDirPath << std::endl; \
    if (boost::filesystem::exists(cDirPath)) { \
      configManager.parseFile(cDirPath); \
      FOR_EACH(GET_VAR_1, __VA_ARGS__) \
    } else { \
      cDirPath = string(ConfigManager::getCommonConfigDirPath() + configFile + string(".ini")); \
      if (boost::filesystem::exists(cDirPath)) { \
        configManager.parseFile(cDirPath); \
        FOR_EACH(GET_VAR_1, __VA_ARGS__) \
      } else { \
        std::cout << "Configuration file: " << cDirPath <<" does not exist." << std::endl; \
      } \
    } \
  }
#endif
#endif //!_CONFIGMACROS_H_
