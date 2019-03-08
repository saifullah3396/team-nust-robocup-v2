/**
 * @file TNRSBase/include/MemoryBase.h
 *
 * This file declares the class MemoryBase
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author Team-Nust 2015
 * @date 10 Jun 2017
 */

#pragma once

#include "TNRSBase/include/MemoryConnector.h"

class BaseModule;

/**
 * @class MemoryBase
 * @brief An interface that allows access to shared memory
 */
class MemoryBase
{
public:
  /**
   * @brief MemoryBase Constructor
   *
   * @param connectingModule the BaseModule through which the class will
   *   be accessing the memory
   */
  MemoryBase(BaseModule* connectingModule);

  /**
   * @brief MemoryBase Constructor
   */
  MemoryBase() = default;

  /**
   * @brief ~MemoryBase Destructor
   */
  virtual ~MemoryBase() {}

  //! Getters
  InputMemoryConnector* getInputConnector() { return inputConnector; }
  OutputMemoryConnector* getOutputConnector() { return outputConnector; }
protected:
  //! Shared memory input connector of the thread
  InputMemoryConnector* inputConnector;

  //! Shared memory output connector of the thread
  OutputMemoryConnector* outputConnector;
};
