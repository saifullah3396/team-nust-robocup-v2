/**
 * @file TNRSBase/include/ConnectorMacro.h
 *
 * This file declares the macros for defining SharedMemory connectors,
 * and for defining and providing access to the SharedMemory variables
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

//#include "TNRSBase/include/BaseModule.h"
//#include "TNRSBase/include/SharedMemory.h"
//#include "TNRSBase/include/MemoryVariable.h"
//#include "Utils/include/DataUtils.h"
#include "Utils/include/VariadicMacros.h"

/**
 * \def DECLARE_IVARS_(TYPE_VAR)
 *
 * Takes a single input variable (type, name), removes
 * paranthesis and sends it to declaration
 *
 * @param TYPE_VAR (typeOfVarX, indexOfVarX) in paranthesis
 */
#define DECLARE_IVARS_(TYPE_VAR) \
{ \
  DECLARE_IVAR( \
    M_GET_ELEM(0,UNPAREN(TYPE_VAR)), \
    M_GET_ELEM(1,UNPAREN(TYPE_VAR)) \
  ) \
}

/**
 * \def DECLARE_IVAR(type, LOCAL_INDEX)
 *
 * Separates local and global memory indexes for input variables
 *
 * @param type variable type
 * @param LOCAL_INDEX local index name or enum of the variable
 */
#define DECLARE_IVAR(type, LOCAL_INDEX) \
{ \
  IVAR_EXTENDED( \
    type, \
    Input::LOCAL_INDEX, \
    MemoryVariableIds::LOCAL_INDEX \
  ) \
}

/**
 * \def DECLARE_OVARS_(TYPE_VAR)
 *
 * Takes a single output variable (type, name), removes
 * paranthesis and sends it to declaration
 *
 * @param TYPE_VAR (typeOfVarX, indexOfVarX) in paranthesis
 */
#define DECLARE_OVARS_(TYPE_VAR) \
{ \
  DECLARE_OVAR( \
    M_GET_ELEM(0,UNPAREN(TYPE_VAR)), \
    M_GET_ELEM(1,UNPAREN(TYPE_VAR)) \
  ) \
}

/**
 * \def DECLARE_OVAR(type, LOCAL_INDEX)
 *
 * Separates local and global memory indexes for output variables
 *
 * @param type variable type
 * @param LOCAL_INDEX local index name or enum of the variable
 */
#define DECLARE_OVAR(type, LOCAL_INDEX) \
{ \
  OVAR_EXTENDED( \
    type, \
    Output::LOCAL_INDEX, \
    MemoryVariableIds::LOCAL_INDEX \
  ) \
}

/**
 * \def IVAR_EXTENDED(type, LOCAL_INDEX, SHAREDMEMORY_INDEX)
 *
 * Defines a local input variable and its relationship with global
 * memory variable
 *
 * @param type variable type
 * @param LOCAL_INDEX local index name or enum of the variable
 * @param SHAREDMEMORY_INDEX shared memory index name or enum of the
 *   variable defined
 */
#define IVAR_EXTENDED(type, LOCAL_INDEX, SHAREDMEMORY_INDEX) \
{ \
    memoryVariables[static_cast<int>(LOCAL_INDEX)] = \
      sharedMemory->variables[static_cast<int>(SHAREDMEMORY_INDEX)]; \
    variables[static_cast<int>(LOCAL_INDEX)] = \
      new Variable<type>(sharedMemory->variables[static_cast<int>(SHAREDMEMORY_INDEX)]->getVariableName()); \
}

/**
 * \def OVAR_EXTENDED(type, LOCAL_INDEX, SHAREDMEMORY_INDEX)
 *
 * Defines a local output variable and its relationship with global
 * memory variable
 *
 * @param type variable type
 * @param LOCAL_INDEX local index name or enum of the variable
 * @param SHAREDMEMORY_INDEX shared memory index name or enum of the
 *   variable defined
 */
#define OVAR_EXTENDED(type, LOCAL_INDEX, SHAREDMEMORY_INDEX) \
{ \
  memoryVariables[static_cast<int>(LOCAL_INDEX)] = \
    sharedMemory->variables[static_cast<int>(SHAREDMEMORY_INDEX)]; \
  variables[static_cast<int>(LOCAL_INDEX)] = \
    new Variable<type>(sharedMemory->variables[static_cast<int>(SHAREDMEMORY_INDEX)]->getVariableName()); \
  static_cast<Variable<type>*>(variables[static_cast<int>(LOCAL_INDEX)])-> \
    setValue( \
      (static_cast<Variable<type>*>(memoryVariables[static_cast<int>(LOCAL_INDEX)]))->getValueSafe() \
    ); \
}

/**
 * \def IVAR(type, var)
 *
 * Provides access to a local input variable
 *
 * @param type variable type
 * @param var variable index
 */
#define IVAR(type, var) (static_cast<Variable<type>*>(this->inputConnector->getVariables()[static_cast<int>(var)]))->getValue()

/**
 * \def OVAR(type, var)
 *
 * Provides access to a local output variable
 *
 * @param type variable type
 * @param var variable index
 */
#define OVAR(type, var) (static_cast<Variable<type>*>(this->outputConnector->getVariables()[static_cast<int>(var)]))->getValueRef()

/**
 * \def IVAR_REL(base, type, var)
 *
 * Provides access to a input variable relative to given class memory connector
 *
 * @param base base class with memory connectors
 * @param type variable type
 * @param var variable index
 */
#define IVAR_REL(base, type, var) (static_cast<Variable<type>*>(base->inputConnector->getVariables()[static_cast<int>(var)]))->getValue()

/**
 * \def OVAR_REL(base, type, var)
 *
 * Provides access to a ounput variable relative to given class memory connector
 *
 * @param base base class with memory connectors
 * @param type variable type
 * @param var variable index
 */
#define OVAR_REL(base, type, var) (static_cast<Variable<type>*>(base->outputConnector->getVariables()[static_cast<int>(var)]))->getValueRef()

/**
 * \def IVAR_PTR(type, var)
 *
 * Provides pointer access to a local input variable
 *
 * @param type variable type
 * @param var variable index
 */
#define IVAR_PTR(type, var) static_cast<Variable<type>*>(this->inputConnector->getVariables()[static_cast<int>(var)])->getValuePtr()

/**
 * \def OVAR_PTR(type, var)
 *
 * Provides pointer access to a local output variable
 *
 * @param type variable type
 * @param var variable index
 */
#define OVAR_PTR(type, var) static_cast<Variable<type>*>(this->outputConnector->getVariables()[static_cast<int>(var)])->getValuePtr()
