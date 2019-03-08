/**
 * @file TNRSBase/src/MemoryConnector.cpp
 *
 * This file implements the classes MemoryConnector, InputMemoryConnector,
 * and OutputMemoryConnector
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#include "TNRSBase/include/BaseModule.h"
#include "TNRSBase/include/MemoryConnector.h"
#include <iostream>
using namespace std;

MemoryConnector::MemoryConnector(
  BaseModule* connectingThread,
  const string& connectorName) :
  connectingThread(connectingThread),
  connectorName(connectorName)
{
}

MemoryConnector::~MemoryConnector()
{
  for (size_t i = 0; i < variables.size(); ++i) {
    delete variables[i];
    variables[i] = NULL;
  }
}

void InputMemoryConnector::syncFromMemory()
{
  for (size_t i = 0; i < memoryVariables.size(); ++i) {
    memoryVariables[i]->getValueIntoPtr(variables[i]);
  }
}

void OutputMemoryConnector::syncToMemory()
{
  for (size_t i = 0; i < memoryVariables.size(); ++i) {
    memoryVariables[i]->setValueFromPtr(variables[i]);
  }
}
