/**
 * @file TNRSBase/include/DebugBase.h
 *
 * This file implements the class DebugBase.
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 09 Nov 2017
 */

#pragma once

#include "TNRSBase/include/MemoryVariable.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/VariadicMacros.h"

/**
 * \def DVAR(type, var)
 *
 * This macro gets the access to the local debug variable.
 *
 * @param type: Type of the variable.
 * @param var: Index of the variable.
 */
#define DVAR(type, var, value) static_cast<Variable<type>*>(variables[static_cast<int>(DebugVar::var)])->getAccess()

/**
 * \def DVAR(type, var)
 *
 * This macro sets the value of the local debug variable.
 *
 * @param type: Type of the variable.
 * @param var: Index of the variable.
 */
#define SET_DVAR(type, var, value) static_cast<Variable<type>*>(variables[static_cast<int>(DebugVar::var)])->setValue(value)

/**
 * \def DVAR(type, var)
 *
 * This macro returns the value of the local debug variable.
 *
 * @param type: Type of the variable.
 * @param var: Index of the variable.
 */
#define GET_DVAR(type, var) static_cast<Variable<type>*>(variables[static_cast<int>(DebugVar::var)])->getValue()

#define DEFINE_DBG_VARIABLE(type, name, value) \
{ \
  Variable<type> * obj = new Variable<type>(#name, value); \
  variables[static_cast<int>(DebugVar::name)] = obj; \
}

/**
 * \def DEFINE_VARIABLE_2(type, name, value)
 * This macro takes the inputs type, name, value sends them to
 * DEFINE_VARIABLE()
 *
 * @param type: Variable type
 * @param name: Variable name
 * @param type: Variable value
 */
#define DEFINE_VARIABLE_2(type, name, value) \
  DEFINE_DBG_VARIABLE(type, name, value)

/**
 * \def DEFINE_VARIABLE_1(TYPE_VAR)
 * This macro takes a single input variable (type, name, value), removes
 * paranthesis and sends it to DEFINE_VARIABLE_1()
 *
 * @param TYPE_VAR: (type, name, value) in paranthesis.
 */
#define DEFINE_VARIABLE_1(TYPE_VAR) \
  DEFINE_VARIABLE_2( \
  M_GET_ELEM(0,UNPAREN(TYPE_VAR)), \
  M_GET_ELEM(1,UNPAREN(TYPE_VAR)), \
  M_GET_ELEM(2,UNPAREN(TYPE_VAR)) \
  )

/**
 * \def INIT_DEBUG_BASE_(...)
 *
 * Wrapper for macro INIT_DEBUG_BASE_()
 *
 * @param ... : (type, name), (type, name), ... (typelast, namelast), .
 */
#define INIT_DEBUG_BASE(...) INIT_DEBUG_BASE_(__VA_ARGS__)

#define PUSH_TO_MAP_2(name) varMap.insert(pair<string, unsigned>(#name, static_cast<unsigned>(DebugVar::name)));
#define PUSH_TO_MAP_1(name) PUSH_TO_MAP_2(name)
#define PUSH_TO_MAP(TYPE_VAR) \
  PUSH_TO_MAP_1( \
  M_GET_ELEM(1,UNPAREN(TYPE_VAR)) \
  )

/**
 * \def INIT_DEBUG_BASE_(...)
 *
 * This macro initiates the debug base class with given debug variables.
 *
 * @param ... : (type, name), (type, name), ... (typelast, namelast), .
 */
#define INIT_DEBUG_BASE_(...) \
  protected: \
  /** \
  * Enumerator for all the debug variables of the class \
  * \
  * @enum DebugVar \
  */ \
  DEFINE_ENUM(DebugVar, 0, SEPARATE(__VA_ARGS__), COUNT); \
  void initDebugBase() { \
    variables.resize(static_cast<int>(DebugVar::COUNT)); \
    FOR_EACH(DEFINE_VARIABLE_1, __VA_ARGS__); \
    FOR_EACH(PUSH_TO_MAP, __VA_ARGS__) \
  }

namespace Json
{
  class Value;
}

/**
 * @class DebugBase
 * @brief A class that initiates a debug interface for the child class.
 */
class DebugBase
{
public:
  /**
   * Default constructor for this class.
   */
  DebugBase(const string& name, DebugBase* ptr);

  /**
   * Default destructor for this class.
   */
  virtual
  ~DebugBase()
  {
    for (auto it = variables.begin(); it != variables.end(); ++it) {
      delete *it;
    }
  }

  /**
   * Initiates the debug base with given debug variables.
   * Use macro INIT_DEBUG_BASE in child class for definition.
   */
  virtual void
  initDebugBase() = 0;

  /**
   * Updates the given debug variable with value provided as string.
   * Use macro INIT_DEBUG_BASE in child class for definition.
   */
  virtual void updateDebugVar(
    const string& varName, const Json::Value& jsonValue)
  {
    variables[varMap[varName]]->setValueFromJson(jsonValue);
  }

  /**
   * This function processes the incoming message, finds which class
   * it corresponds to and assigns input values to the debugInfo
   * variables of the class.
   *
   * @param msg: Input message recieved from the debugger.
   */
  static string processDebugMsg(const Json::Value& msg);

  /**
   * Returns the memory variable by matching its name.
   *
   * @param string name: Name of the variable to be returned.
   * @return Pointer to MemoryVariable.
   */
  MemoryVariable*
  findVariableFromName(const string& name)
  {
    for (int i = 0; i < variables.size(); i++) {
      if (variables[i]->getVariableName() == name) return variables[i];
    }
    return NULL;
  }
protected:
  //! Any message sent to the specified class
  string parseJSONMessage(const Json::Value msg);

  //! Vector of pointers to threadsafe memory variables.
  vector<MemoryVariable*> variables;

  //! String to enum map for variable names.
  map<string, unsigned> varMap;
private:
  //! Updates the class map with class name and its pointer.
  void
  updateClassMap(const string& name, DebugBase* ptr);

  //! Name to pointer map for all the childs of this class.
  static map<string, DebugBase*> classMap;

  //! Image queue thread-safe access mutex.
  static pthread_mutex_t classMapAccessMutex;
};

