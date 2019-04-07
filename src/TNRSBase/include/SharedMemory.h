/**
 * @file TNRSBase/include/SharedMemory.h
 *
 * This file declares the class SharedMemory
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

#include "TNRSBase/include/MemoryConnector.h"
#include "TNRSBase/include/MemoryDefinition.h"
#include "TNRSBase/include/MemoryVariable.h"

/**
 * @class SharedMemory
 * @brief The class that holds all the shared data from different threads
 */
class SharedMemory
{
public:
  /**
   * Constructor
   */
  SharedMemory() {}

  /**
   * Destructor
   */
  ~SharedMemory()
  {
    for (size_t i = 0; i < variables.size(); ++i)
      delete variables[i];
    variables.clear();
  }

  /**
   * Initializes the memory blackboard. All the memory variables are
   * declared/defined in this function.
   *
   * @return void
   */
  void init();

  /**
   * @brief getJson Returns the memory data in JSON format
   * @return Json::Value
   */
  Json::Value getJson();

  /**
   * Returns the vector of all the memory variables
   *
   * @return vector<MemoryVariable*>
   */
  vector<MemoryVariable*> getVariables() { return variables; }

  ///< Vector of pointers to threadsafe memory variables
  vector<MemoryVariable*> variables;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<SharedMemory> SharedMemoryPtr;
