/**
 * @file TNRSBase/src/DebugBase.cpp
 *
 * This file implements the class DebugBase.
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 09 Nov 2017
 */

#include "TNRSBase/include/DebugBase.h"
#include "Utils/include/JsonUtils.h"
#include "Utils/include/DebugUtils.h"

map<string, DebugBase*> DebugBase::classMap = map<string, DebugBase*>();
pthread_mutex_t DebugBase::classMapAccessMutex = PTHREAD_MUTEX_INITIALIZER;

DebugBase::DebugBase(const string& name, DebugBase* ptr)
{
  updateClassMap(name, ptr);
}

void DebugBase::updateClassMap(const string& name, DebugBase* ptr)
{
  pthread_mutex_lock(&classMapAccessMutex);
  classMap.insert(pair<string, DebugBase*>(name, ptr));
  pthread_mutex_unlock(&classMapAccessMutex);
}

string DebugBase::processDebugMsg(const Json::Value& root)
{
  string response;
  for (const auto& className : root.getMemberNames()) {
    if (classMap.find(className) == classMap.end()) {
      response += string("[DebugBase] Class \'" + className + "\' does not exist in debug layer.");
      continue;
    }
    DebugBase* classPtr;
    pthread_mutex_lock(&classMapAccessMutex);
    classPtr = classMap[className];
    auto vars = root[className];
    for (const auto& var : vars.getMemberNames()) {
      try {
        classPtr->updateDebugVar(var, vars[var]);
        response += "[DebugBase] Variable \'" + className + "::" + var + string("\' set.");
      } catch (...) {
        response += "[DebugBase] Invalid value passed to the variable \'" + className + "::" + var + "\'";
      }
      response += "\n";
    }
    response += classPtr->parseJSONMessage(root["message"]);
    pthread_mutex_unlock(&classMapAccessMutex);
  }
  return response;
}

string DebugBase::parseJSONMessage(const Json::Value msg) {
  return "";
}
