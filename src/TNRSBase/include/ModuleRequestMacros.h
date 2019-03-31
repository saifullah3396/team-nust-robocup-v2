/**
 * @file TNRSBase/include/ModuleRequestMacros.h
 *
 * The file defines the macros for easier module request generation.
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
#include "Utils/include/Exceptions/TNRSException.h"
#include "Utils/include/PrintUtils.h"

#define DECLARE_MODULE_REQUEST( \
  RequestName, \
  PointerName, \
  BaseRequestName, \
  BaseEnumType, \
  BaseId, \
  RequestEnumType) \
  class RequestName : public BaseRequestName \
  { \
  public: \
    RequestName(const RequestEnumType& id) : \
      BaseRequestName(BaseEnumType::BaseId, toUType(id)) \
    { \
    } \
     \
    static boost::shared_ptr<RequestName> makeFromJson(const Json::Value& obj); \
     \
    /** \
     * @brief assignFromJson Assigns request parameters from json \
     * @param obj Json configuration \
     * @return true if successful \
     */ \
    virtual bool assignFromJson(const Json::Value& obj) \
    { \
      if (!BaseRequestName::assignFromJson(obj)) \
        return false; \
      return true; \
    } \
    \
    /** \
     * @brief getJson Makes a json object from request paramters \
     * @return Json object \
     */ \
    virtual Json::Value getJson() { \
      return BaseRequestName::getJson(); \
    } \
  }; \
  typedef boost::shared_ptr<RequestName> PointerName;

#define DECLARE_MODULE_REQUEST_TYPE( \
  TypeName, \
  PointerName, \
  BaseRequestName, \
  EnumName, \
  EnumId) \
  struct TypeName : BaseRequestName \
  { \
    /** \
     * Constructor \
     */ \
    TypeName() : \
      BaseRequestName(EnumName::EnumId) \
    { \
    } \
    /** \
     * @brief assignFromJson Assigns requesturation parameters from json \
     * @param obj Json requesturation \
     * @return true if successful \
     */ \
    virtual bool assignFromJson(const Json::Value& obj) { \
      if (!BaseRequestName::assignFromJson(obj)) \
        return false; \
      return true; \
    } \
    \
    /** \
     * @brief getJson Makes a json object from request paramters \
     * @return Json object \
     */ \
    \
    virtual Json::Value getJson() { \
      Json::Value obj = BaseRequestName::getJson(); \
      return obj; \
    } \
    public: \
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW \
  }; \
  typedef boost::shared_ptr<TypeName> PointerName;

#define DECLARE_MODULE_REQUEST_TYPE_WITH_VARS( \
  TypeName, \
  PointerName, \
  BaseRequestName, \
  EnumName, \
  EnumId, \
  ...) \
  struct TypeName : BaseRequestName \
  { \
    FOR_EACH(DECLARE_VAR_0, __VA_ARGS__); \
    \
    /** \
     * Constructor \
     */ \
    TypeName(FOR_EACH_IN_PLACE(DEFAULT_VAR_0, __VA_ARGS__)) : \
      BaseRequestName(EnumName::EnumId) \
    { \
      FOR_EACH(DEFINE_VAR_0, __VA_ARGS__) \
    } \
    \
    /** \
     * @brief assignFromJson Assigns requesturation parameters from json \
     * @param obj Json requesturation \
     * @return true if successful \
     */ \
    virtual bool assignFromJson(const Json::Value& obj) { \
      if (!BaseRequestName::assignFromJson(obj)) \
        return false; \
      try { \
        FOR_EACH(ASSIGN_FROM_JSON_VAR_1, __VA_ARGS__) \
      } catch (Json::Exception& e) { \
        LOG_EXCEPTION("Exception caught in " #TypeName << ": "); \
        LOG_EXCEPTION(e.what()); \
        return false; \
      } \
      return true; \
    } \
    /** \
     * @brief getJson Makes a json object from request paramters \
     * @return Json object \
     */ \
    virtual Json::Value getJson() { \
      Json::Value obj = BaseRequestName::getJson(); \
      try { \
        FOR_EACH(GET_JSON_VAR_1, __VA_ARGS__); \
      } catch (Json::Exception& e) { \
       LOG_EXCEPTION( \
         "Exception caught in module request" #BaseRequestName "::" #TypeName) \
        LOG_EXCEPTION(e.what()); \
      } \
      return obj; \
    } \
    public: \
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW \
  }; \
  typedef boost::shared_ptr<TypeName> PointerName;

#define GET_REQUEST_CHILD_TYPE_3(ENUM, ID, NAME) \
  case toUType(ENUM::ID): \
    request = boost::make_shared<NAME>(); \
    break;

#define GET_REQUEST_CHILD_TYPE_2(ENUM, ID, NAME) \
  GET_REQUEST_CHILD_TYPE_3(ENUM, ID, NAME)

#define GET_REQUEST_CHILD_TYPE_1(ENUM_ID_NAME) \
  GET_REQUEST_CHILD_TYPE_2( \
    M_GET_ELEM(0,UNPAREN(ENUM_ID_NAME)), \
    M_GET_ELEM(1,UNPAREN(ENUM_ID_NAME)), \
    M_GET_ELEM(2,UNPAREN(ENUM_ID_NAME)) \
  )

#define GET_REQUEST_CHILD_TYPE_0(...) \
  GET_REQUEST_CHILD_TYPE_1(__VA_ARGS__)

#define DEFINE_MODULE_REQUEST( \
  RequestName, \
  BaseRequestName, \
  PointerName, \
  ...) \
  PointerName RequestName::makeFromJson(const Json::Value& obj) \
  { \
    PointerName request; \
    try { \
      if (!obj.isNull()) { \
        switch (obj["requestId"].asUInt()) { \
          FOR_EACH(GET_REQUEST_CHILD_TYPE_1, __VA_ARGS__); \
        } \
      } \
      if (!request->assignFromJson(obj)) { \
        throw TNRSException( \
          "Error creating a requesturation from given Json object"); \
      } \
    } catch (Json::Exception& e) { \
      LOG_EXCEPTION(\
        "Exception caught making a balance module request from json object;\t"); \
      LOG_EXCEPTION(e.what()); \
      request.reset(); \
    } catch (TNRSException& e) { \
      LOG_EXCEPTION(e.what()); \
      request.reset(); \
    } \
    return request; \
  }

#define DEFINE_MODULE_REQUEST_WITHOUT_TYPES( \
  RequestName, \
  BaseRequestName, \
  PointerName) \
  PointerName RequestName::makeFromJson(const Json::Value& obj) \
  { \
    PointerName request; \
    LOG_ERROR(\
      "No module request defined under " #BaseRequestName "::" #RequestName "."); \
    request.reset(); \
    return request; \
  }

#define DECLARE_SWITCH_REQUEST_TYPE( \
  TypeName, \
  PointerName, \
  BaseRequestName, \
  EnumName, \
  EnumId) \
  struct TypeName : BaseRequestName, SwitchRequest \
  { \
    /** \
     * Constructor \
     */ \
    TypeName(const bool& state = false) : \
      BaseRequestName(EnumName::EnumId), \
      SwitchRequest(state) \
    { \
    } \
    /** \
     * @brief assignFromJson Assigns requesturation parameters from json \
     * @param obj Json requesturation \
     * @return true if successful \
     */ \
    virtual bool assignFromJson(const Json::Value& obj) { \
      if (!BaseRequestName::assignFromJson(obj)) \
        return false; \
      try { \
        FOR_EACH(ASSIGN_FROM_JSON_VAR_1, (bool, state, false),) \
      } catch (Json::Exception& e) { \
        LOG_EXCEPTION("Exception caught in " #TypeName << ": "); \
        LOG_EXCEPTION(e.what()); \
        return false; \
      } \
      return true; \
    } \
    \
    /** \
     * @brief getJson Makes a json object from request paramters \
     * @return Json object \
     */ \
    \
    virtual Json::Value getJson() { \
      Json::Value obj = BaseRequestName::getJson(); \
      try { \
        FOR_EACH(GET_JSON_VAR_1, (bool, state, false),); \
      } catch (Json::Exception& e) { \
       LOG_EXCEPTION( \
         "Exception caught in module request" #BaseRequestName "::" #TypeName) \
        LOG_EXCEPTION(e.what()); \
      } \
      return obj; \
    } \
    public: \
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW \
  }; \
  typedef boost::shared_ptr<TypeName> PointerName;

#define DECLARE_SWITCH_REQUEST_TYPE_WITH_VARS( \
  TypeName, \
  PointerName, \
  BaseRequestName, \
  EnumName, \
  EnumId, \
  ...) \
  struct TypeName : BaseRequestName, SwitchRequest \
  { \
    FOR_EACH(DECLARE_VAR_0, __VA_ARGS__); \
    \
    /** \
     * Constructor \
     */ \
    TypeName( \
      const bool& state = false,  \
      FOR_EACH_IN_PLACE(DEFAULT_VAR_0, __VA_ARGS__)) : \
      BaseRequestName(EnumName::EnumId), \
      SwitchRequest(state) \
    { \
      FOR_EACH(DEFINE_VAR_0, __VA_ARGS__) \
    } \
    \
    /** \
     * @brief assignFromJson Assigns requesturation parameters from json \
     * @param obj Json requesturation \
     * @return true if successful \
     */ \
    virtual bool assignFromJson(const Json::Value& obj) { \
      if (!BaseRequestName::assignFromJson(obj)) \
        return false; \
      try { \
        FOR_EACH(ASSIGN_FROM_JSON_VAR_1, (bool, state, false), __VA_ARGS__) \
      } catch (Json::Exception& e) { \
        LOG_EXCEPTION("Exception caught in " #TypeName << ": "); \
        LOG_EXCEPTION(e.what()); \
        return false; \
      } \
      return true; \
    } \
    /** \
     * @brief getJson Makes a json object from request paramters \
     * @return Json object \
     */ \
    virtual Json::Value getJson() { \
      Json::Value obj = BaseRequestName::getJson(); \
      try { \
        FOR_EACH(GET_JSON_VAR_1, (bool, state, false), __VA_ARGS__); \
      } catch (Json::Exception& e) { \
       LOG_EXCEPTION( \
         "Exception caught in module request" #BaseRequestName "::" #TypeName) \
        LOG_EXCEPTION(e.what()); \
      } \
      return obj; \
    } \
    public: \
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW \
  }; \
  typedef boost::shared_ptr<TypeName> PointerName;
