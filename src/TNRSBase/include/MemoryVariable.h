/**
 * @file TNRSBase/include/MemoryVariable.h
 *
 * This file declares the class Variable
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

#include "Utils/include/JsonUtils.h"

/**
 * Macro that defines a new variable of given type, name and value in
 * the SharedMemory.
 *
 * @param type: Type of the variable.
 * @param name: Name of the variable.
 * @param value: Value of the variable.
 */
#define DEFINE_VARIABLE(type, name, value) \
{ \
  Variable<type> * obj = new Variable<type>(#name, value); \
  variables[static_cast<int>(MemoryVariableIds::name)] = obj; \
}

/**
 * Macro that declares a new variable of given type and name in the
 * SharedMemory.
 *
 * @param type: Type of the variable.
 * @param name: Name of the variable.
 */
#define DECLARE_VARIABLE(type, name) \
{ \
  Variable<type>* obj = new Variable<type>(#name); \
  variables[static_cast<int>(MemoryVariableIds::name)] = obj; \
}

template <typename T>
class Variable;

class MemoryVariable
{
public:
  MemoryVariable(const MemoryVariable&) = default;
  MemoryVariable(MemoryVariable&&) = default;
  MemoryVariable& operator=(const MemoryVariable&) & = default;
  MemoryVariable& operator=(MemoryVariable&&) & = default;
  virtual ~MemoryVariable() {}
  MemoryVariable(const string& name) : name(name) {}

  virtual Json::Value getJson() = 0;
  virtual void setValueFromJson(const Json::Value& json) = 0;
  virtual void setValueFromPtr(MemoryVariable* ptr) = 0;
  virtual void getValueIntoPtr(MemoryVariable* ptr) = 0;

  ///< Getters
  const string& getVariableName() { return name; }
private:
  string name; ///< Variable name
};

/**
 * @class Variable
 * @brief Base class for all thread-safe template variables
 */
template<typename T>
class Variable : public MemoryVariable
{
public:
  /**
   * @brief Variable Constructor
   * @param name Variable name
   * @param value Variable value
   */
  Variable(const string& name, const T& value = T()) :
    MemoryVariable(name), value(value)
  {
    pthread_mutex_init(&accessMutex, NULL);
  }

  /**
   * @brief ~Variable Destructor
   */
  virtual ~Variable() {
    pthread_mutex_destroy(&accessMutex);
  }

  /**
   * @brief getValue Returns the value of the variable
   * @return T
   */
  auto getValueSafe() -> T
  {
    pthread_mutex_lock(&accessMutex);
    T Tempvalue = value;
    pthread_mutex_unlock(&accessMutex);
    return Tempvalue;
  }

  /**
   * @brief getValue Returns the value of the variable
   * @return T
   */
  const T& getValue()
  {
    return value;
  }

  /**
   * @brief getValue Returns the reference to the value of the variable
   * @return T
   */
  T& getValueRef()
  {
    return value;
  }

  /**
   * @brief getValuePtr Returns the pointer to the value
   * @return T
   */
  T* getValuePtr()
  {
    return &value;
  }

  /**
   * @brief setValue Sets the variable value
   * @param value New value
   */
  void setValueSafe(const T& value)
  {
    pthread_mutex_lock(&accessMutex);
    this->value = value;
    pthread_mutex_unlock(&accessMutex);
  }

  /**
   * @brief setValue Sets the variable value
   * @param value New value
   */
  void setValue(const T& value)
  {
    this->value = value;
  }

  /**
   * @brief setValueFromPtr Sets the value from input pointer
   * @param ptr Pointer
   */
  void setValueFromPtr(MemoryVariable* ptr) final
  {
    pthread_mutex_lock(&accessMutex);
    this->value = static_cast<Variable<T>*>(ptr)->getValue();
    pthread_mutex_unlock(&accessMutex);
  }

  /**
   * @brief getValueIntoPtr Assigns the variable to pointer
   * @param ptr Pointer
   */
  void getValueIntoPtr(MemoryVariable* ptr) final
  {
    pthread_mutex_lock(&accessMutex);
    static_cast<Variable<T>*>(ptr)->setValue(value);
    pthread_mutex_unlock(&accessMutex);
  }

  Json::Value getJson() override { return JsonUtils::getJson(value); }
  void setValueFromJson(const Json::Value& json) override {
    pthread_mutex_lock(&accessMutex);
    JsonUtils::jsonToType(this->value, json, this->value);
    pthread_mutex_unlock(&accessMutex);
  }

protected:
  T value; ///< Value of the variable.
  pthread_mutex_t accessMutex; ///< Mutex for thread safe access to variable

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
