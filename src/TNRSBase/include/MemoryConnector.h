/**
 * @file TNRSBase/include/MemoryConnector.h
 *
 * This file declares the classes MemoryConnector, InputMemoryConnector,
 * and OutputMemoryConnector
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

#include "TNRSBase/include/MemoryDefinition.h"
#include "TNRSBase/include/MemoryVariable.h"

class BaseModule;
class SharedMemory;

/**
 * @class MemoryConnector
 * @brief Base class for the definition of connection to memory
 */
class MemoryConnector
{
public:
  /**
   * @brief MemoryConnector Constructor
   *
   * @param connectingThread BaseModule that wants to connect to memory
   * @param connecterName name of the memory connector
   */
  MemoryConnector(
    BaseModule* connectingThread, const string& connectorName);

  /**
   * @brief ~MemoryConnector Destructor
   */
  virtual ~MemoryConnector();

  /**
   * Pure virtual function for the initialization of input/output
   * connections to memory
   *
   * @return void
   */
  virtual void initConnector() = 0;

  ///< Getters
  vector<MemoryVariable*>& getVariables() { return variables; }
  vector<MemoryVariable*>& getMemoryVariables() { return memoryVariables; }

protected:
  ///< Vector of pointers to memory variables
  vector<MemoryVariable*> variables;

  ///< Vector of pointers to memory variables
  vector<MemoryVariable*> memoryVariables;

  ///< Name of this memory connector
  string connectorName;

  ///< The thread that connects with the memory through this
  ///< memory connector
  BaseModule* connectingThread;
};

/**
 * @class InputMemoryConnector
 * @brief A class that is defines an input connection to SharedMemory
 */
class InputMemoryConnector : public MemoryConnector
{
public:
  /**
   * @brief InputMemoryConnector Constructor
   *
   * @param connectingThread BaseModule that wants to connect to memory
   * @param connecterName name of the memory connector
   */
  InputMemoryConnector(
    BaseModule* connectingThread,
    const string& connectorName) :
    MemoryConnector(connectingThread, connectorName)
  {
  }

  /**
   * @brief syncFromMemory Syncs the input variables of the connected
   *    module from memory
   *
   * @return void
   */
  void syncFromMemory();
};

/**
 * @class OutputMemoryConnector
 * @brief A class that defines an output connection to SharedMemory
 */
class OutputMemoryConnector : public MemoryConnector
{
public:
  /**
   * Constructor
   *
   * @param connectingThread BaseModule that wants to connect to memory
   * @param connecterName name of the memory connector
   */
  OutputMemoryConnector(
    BaseModule* connectingThread,
    const string& connectorName) :
    MemoryConnector(connectingThread, connectorName)
  {
  }

  /**
   * @brief syncToMemory Syncs the output variables from the
   *   connected module to memory
   */
  void syncToMemory();
};
