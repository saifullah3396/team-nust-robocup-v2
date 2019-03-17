/**
 * @file BehaviorManager/include/BehaviorConfigMacros.h
 *
 * The file defines the macros for easier behavior config generation.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include <boost/make_shared.hpp>
#include <json/json.h>
#include "Utils/include/JsonUtils.h"
#include "Utils/include/VariadicMacros.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/EnumUtils.h"
#include "Utils/include/PrintUtils.h"

#define GET_JSON_VAR_3(TYPE, VAR_NAME) \
  obj[#VAR_NAME] = JsonUtils::getJson(VAR_NAME);

#define GET_JSON_VAR_2(TYPE, VAR_NAME) \
  GET_JSON_VAR_3(TYPE, VAR_NAME)

#define GET_JSON_VAR_1(TYPE_VAR) \
  GET_JSON_VAR_2( \
    M_GET_ELEM(0,UNPAREN(TYPE_VAR)), \
    M_GET_ELEM(1,UNPAREN(TYPE_VAR))  \
  )

#define GET_BEHAVIOR_CONFIG_JSON(...) \
  try { \
    FOR_EACH(GET_JSON_VAR_1, __VA_ARGS__); \
  } catch (Json::Exception& e) { \
   LOG_EXCEPTION("Exception caught in behavior config \n\tType: " \
      << static_cast<unsigned>(baseType) \
      << ", Id: "  \
      << DataUtils::varToString(id)  \
      << ";\t"); \
    LOG_EXCEPTION(e.what()); \
  }

#define ASSIGN_FROM_JSON_VAR_3(TYPE, VAR, VALUE) \
  JsonUtils::jsonToType(VAR, Json::Value(obj[#VAR]), VALUE);

#define ASSIGN_FROM_JSON_VAR_2(TYPE, VAR, VALUE) \
  ASSIGN_FROM_JSON_VAR_3(TYPE, VAR, VALUE)

#define ASSIGN_FROM_JSON_VAR_1(TYPE_VAR_VALUE) \
  ASSIGN_FROM_JSON_VAR_2( \
    M_GET_ELEM(0,UNPAREN(TYPE_VAR_VALUE)), \
    M_GET_ELEM(1,UNPAREN(TYPE_VAR_VALUE)),  \
    M_GET_ELEM(2,UNPAREN(TYPE_VAR_VALUE))  \
  )

#define ASSIGN_CONFIG_FROM_JSON(ConfigName, ...) \
  try { \
    FOR_EACH(ASSIGN_FROM_JSON_VAR_1, __VA_ARGS__) \
  } catch (Json::Exception& e) { \
    LOG_EXCEPTION("Exception caught in " #ConfigName << ": "); \
    LOG_EXCEPTION(e.what()); \
    return false; \
  } \
  return true;

#define DECLARE_CONFIG_VAR_3(TYPE, VAR_NAME) \
  TYPE VAR_NAME;

#define DECLARE_CONFIG_VAR_2(TYPE, VAR_NAME) \
  DECLARE_CONFIG_VAR_3(TYPE, VAR_NAME)

#define DECLARE_CONFIG_VAR_1(TYPE_VAR) \
  DECLARE_CONFIG_VAR_2( \
    M_GET_ELEM(0,UNPAREN(TYPE_VAR)), \
    M_GET_ELEM(1,UNPAREN(TYPE_VAR)) \
  )

#define DECLARE_CONFIG_VAR_0(...) \
  DECLARE_CONFIG_VAR_1(__VA_ARGS__)

#define DEFINE_CONFIG_VAR_3(TYPE, VAR_NAME, VALUE) \
  VAR_NAME = VALUE;

#define DEFINE_CONFIG_VAR_2(TYPE, VAR_NAME, VALUE) \
  DEFINE_CONFIG_VAR_3(TYPE, VAR_NAME, VALUE)

#define DEFINE_CONFIG_VAR_1(TYPE_VAR) \
  DEFINE_CONFIG_VAR_2( \
    M_GET_ELEM(0,UNPAREN(TYPE_VAR)), \
    M_GET_ELEM(1,UNPAREN(TYPE_VAR)), \
    M_GET_ELEM(2,UNPAREN(TYPE_VAR)) \
  )

#define DEFINE_CONFIG_VAR_0(...) \
  DEFINE_CONFIG_VAR_1(__VA_ARGS__)

#define DECLARE_BEHAVIOR_CONFIG_WITH_VARS( \
  ConfigName, \
  BaseConfigName, \
  PointerName, \
  IdEnum, \
  Runtime, \
  ChildType, \
  ...) \
  struct ConfigName : BaseConfigName \
  { \
    FOR_EACH(DECLARE_CONFIG_VAR_0, __VA_ARGS__) \
    /** \
     * Constructor \
     * \
     * @param type: Behavior type \
     */ \
    ConfigName(const ChildType& type = (ChildType) 0) : \
      BaseConfigName(IdEnum, Runtime, (int) type) \
    { \
      FOR_EACH(DEFINE_CONFIG_VAR_0, __VA_ARGS__) \
    } \
    /** \
     * @brief makeFromJson Returns a child config of given type \
     * @param obj Json object of configuration \
     * @return ConfigPtr \
     */ \
    static boost::shared_ptr<ConfigName> makeFromJson(const Json::Value& obj); \
    /** \
     * @brief assignFromJson Assigns configuration parameters from json \
     * @param obj Json configuration \
     * @return true if successful \
     */ \
    virtual bool assignFromJson(const Json::Value& obj) { \
      if (!BaseConfigName::assignFromJson(obj)) \
        return false; \
      ASSIGN_CONFIG_FROM_JSON(ConfigName, __VA_ARGS__); \
    } \
    /** \
     * @brief getJson Makes a json object from config paramters \
     * @return Json object \
     */ \
    virtual Json::Value getJson() { \
      Json::Value obj = BaseConfigName::getJson(); \
      GET_BEHAVIOR_CONFIG_JSON(__VA_ARGS__); \
      return obj; \
    } \
  }; \
  typedef boost::shared_ptr<ConfigName> PointerName;

#define DECLARE_BEHAVIOR_CONFIG( \
  ConfigName, \
  BaseConfigName, \
  PointerName, \
  IdEnum, \
  Runtime, \
  ChildType) \
  struct ConfigName : BaseConfigName \
  { \
    /** \
     * Constructor \
     * \
     * @param type: Behavior type \
     */ \
    ConfigName(const ChildType& type = (ChildType) 0) : \
      BaseConfigName(IdEnum, Runtime, (int) type) \
    { \
    } \
    \
    /** \
     * @brief makeFromJson Returns a child config of given type \
     * @param obj Json object of configuration \
     * @return ConfigPtr \
     */ \
    static boost::shared_ptr<ConfigName> makeFromJson(const Json::Value& obj); \
  }; \
  typedef boost::shared_ptr<ConfigName> PointerName;

#define DECLARE_BEHAVIOR_CONFIG_TYPE( \
  TypeName, \
  BaseConfigName, \
  EnumId, \
  PointerName) \
  struct TypeName : BaseConfigName \
  { \
    /** \
     * Constructor \
     */ \
    TypeName() : \
      BaseConfigName(EnumId) \
    { \
    } \
   \
    /** \
     * @derived \
     */ \
    void validate(); \
    /** \
     * @derived \
     */ \
    virtual void init();\
  }; \
  typedef boost::shared_ptr<TypeName> PointerName;


#define DECLARE_BEHAVIOR_CONFIG_BASE_TYPE_WITH_VARS( \
  TypeName, \
  BaseConfigName, \
  EnumType, \
  PointerName, \
  ...) \
  struct TypeName : BaseConfigName \
  { \
    FOR_EACH(DECLARE_CONFIG_VAR_0, __VA_ARGS__); \
    \
    /** \
     * Constructor \
     */ \
    TypeName(const EnumType& id) : \
      BaseConfigName(id) \
    { \
      FOR_EACH(DEFINE_CONFIG_VAR_0, __VA_ARGS__); \
    } \
    /** \
     * @brief assignFromJson Assigns configuration parameters from json \
     * @param obj Json configuration \
     * @return true if successful \
     */ \
    virtual bool assignFromJson(const Json::Value& obj) { \
      if (!BaseConfigName::assignFromJson(obj)) \
        return false; \
      ASSIGN_CONFIG_FROM_JSON(ConfigName, __VA_ARGS__); \
    } \
    /** \
     * @brief getJson Makes a json object from config paramters \
     * @return Json object \
     */ \
    virtual Json::Value getJson() { \
      Json::Value obj = BaseConfigName::getJson(); \
      GET_BEHAVIOR_CONFIG_JSON(__VA_ARGS__); \
      return obj; \
    } \
  }; \
  typedef boost::shared_ptr<TypeName> PointerName;

#define DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS( \
  TypeName, \
  BaseConfigName, \
  EnumId, \
  PointerName, \
  ...) \
  struct TypeName : BaseConfigName \
  { \
    FOR_EACH(DECLARE_CONFIG_VAR_0, __VA_ARGS__); \
    \
    /** \
     * Constructor \
     */ \
    TypeName() : \
      BaseConfigName(EnumId) \
    { \
      FOR_EACH(DEFINE_CONFIG_VAR_0, __VA_ARGS__); \
    } \
    \
    /** \
     * @derived \
     */ \
    void validate(); \
    /** \
     * @derived \
     */ \
    virtual void init();\
    /** \
     * @brief assignFromJson Assigns configuration parameters from json \
     * @param obj Json configuration \
     * @return true if successful \
     */ \
    virtual bool assignFromJson(const Json::Value& obj) { \
      if (!BaseConfigName::assignFromJson(obj)) \
        return false; \
      ASSIGN_CONFIG_FROM_JSON(ConfigName, __VA_ARGS__); \
    } \
    /** \
     * @brief getJson Makes a json object from config paramters \
     * @return Json object \
     */ \
    virtual Json::Value getJson() { \
      Json::Value obj = BaseConfigName::getJson(); \
      GET_BEHAVIOR_CONFIG_JSON(__VA_ARGS__); \
      return obj; \
    } \
  }; \
  typedef boost::shared_ptr<TypeName> PointerName;


#define GET_CHILD_TYPE_3(ENUM, ID, NAME) \
  case toUType(ENUM::ID): \
    LOG_INFO("id: " << toUType(ENUM::ID)) \
    config = boost::make_shared<NAME>(); \
    break;

#define GET_CHILD_TYPE_2(ENUM, ID, NAME) \
  GET_CHILD_TYPE_3(ENUM, ID, NAME)

#define GET_CHILD_TYPE_1(ENUM_ID_NAME) \
  GET_CHILD_TYPE_2( \
    M_GET_ELEM(0,UNPAREN(ENUM_ID_NAME)), \
    M_GET_ELEM(1,UNPAREN(ENUM_ID_NAME)), \
    M_GET_ELEM(2,UNPAREN(ENUM_ID_NAME)) \
  )

#define GET_CHILD_TYPE_0(...) \
  GET_CHILD_TYPE_1(__VA_ARGS__)

#define DEFINE_BEHAVIOR_CONFIG( \
  ConfigName, \
  BaseConfigName, \
  PointerName, \
  ...) \
  PointerName ConfigName::makeFromJson(const Json::Value& obj) \
  { \
    PointerName config; \
    try { \
      LOG_INFO("obj: " << obj) \
      if (!obj.isNull()) { \
        LOG_INFO("type: " << obj["type"].asUInt()) \
        switch (obj["type"].asUInt()) { \
          FOR_EACH(GET_CHILD_TYPE_1, __VA_ARGS__); \
        } \
      } \
      if (!config->assignFromJson(obj)) { \
        throw TNRSException( \
          "Error creating a configuration from given Json object"); \
      } \
    } catch (Json::Exception& e) { \
      LOG_EXCEPTION(\
        "Exception caught making a balance behavior config from json object;\t"); \
      LOG_EXCEPTION(e.what()); \
      config.reset(); \
    } catch (TNRSException& e) { \
      LOG_EXCEPTION(e.what()); \
      config.reset(); \
    } \
    return config; \
  }



