/**
 * @file TNRSBase/include/InoutConnectors.h
 *
 * This file declares the macros for declaring the shared memory
 * connections for each class
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#include "TNRSBase/include/ConnectorMacro.h"

#define DECLARE_INPUT_CONNECTOR(...) DECLARE_INPUT_CONNECTOR_(__VA_ARGS__)
#define DEFINE_INPUT_CONNECTOR(BaseClass, ...) DEFINE_INPUT_CONNECTOR_(BaseClass, __VA_ARGS__)
#define DECLARE_OUTPUT_CONNECTOR(...) DECLARE_OUTPUT_CONNECTOR_(__VA_ARGS__)
#define DEFINE_OUTPUT_CONNECTOR(BaseClass, ...) DEFINE_OUTPUT_CONNECTOR_(BaseClass, __VA_ARGS__)

/**
 * \def DECLARE_INPUT_CONNECTOR_(...)
 *
 * Declares the input connector struct for the inialization
 * of input/output connection of a BaseModule with the shared memory
 *
 * @param ... paranthesis enclosed array of variables to be
 *   defined (type, name), (type, name), ... (typelast, namelast),
 */
#define DECLARE_INPUT_CONNECTOR_(...) \
  public: \
  /** \
   * Enumerator for all the input variables of the class \
   * written to shared memory \
   * \
   * @enum InputEnum \
   */ \
  DEFINE_ENUM(Input, 0, __VA_ARGS__, COUNT); \
  struct InputConnector : public InputMemoryConnector \
  { \
    /** \
     * Initializes the thread connection for shared memory read \
     * access \
     * \
     * @param connectingThread pointer to BaseModule \
     * @param connectorName name of the module \
     */ \
    InputConnector( \
      BaseModule* connectingThread, string connectorName);\
    \
    /** \
    * Derived from MemoryConnector \
    */ \
    void initConnector(); \
  };

/**
 * \def DEFINE_INPUT_CONNECTOR_(...)
 *
 * Implements the input connector struct for the inialization
 * of input/output connection of a BaseModule with the shared memory
 *
 * @param ... paranthesis enclosed array of variables to be
 *   defined (type, name), (type, name), ... (typelast, namelast),
 */
#define DEFINE_INPUT_CONNECTOR_(BaseClass, ...) \
  BaseClass::InputConnector::InputConnector( \
    BaseModule* connectingThread, string connectorName) : \
    InputMemoryConnector(connectingThread, connectorName) \
  {} \
  \
  void BaseClass::InputConnector::initConnector() \
  { \
    SharedMemoryPtr sharedMemory = \
      connectingThread->getLocalSharedMemory(); \
    variables.assign(static_cast<int>(BaseClass::Input::COUNT), NULL); \
    memoryVariables.assign(static_cast<int>(BaseClass::Input::COUNT), NULL);  \
    FOR_EACH(DECLARE_IVARS_, __VA_ARGS__) \
  } \

/**
 * \def DECLARE_OUTPUT_CONNECTOR_(...)
 *
 * Declares the output connector struct for the inialization
 * of input/output connection of a BaseModule with the shared memory
 *
 * @param ... paranthesis enclosed array of variables to be
 *   defined (type, name), (type, name), ... (typelast, namelast),
 */
#define DECLARE_OUTPUT_CONNECTOR_(...) \
  public: \
  /** \
   * Enumerator for all the output variables of the class \
   * written to shared memory \
   * \
   * @enum OutputEnum \
   */ \
  DEFINE_ENUM(Output, 0, __VA_ARGS__, COUNT); \
  struct OutputConnector : public OutputMemoryConnector \
  { \
    /** \
     * Initializes the BaseModule connection for shared memory read \
     * access \
     * \
     * @param connectingThread pointer to BaseModule \
     * @param connectorName name of the module \
     */ \
    OutputConnector( \
      BaseModule* connectingThread, string connectorName); \
    \
    /** \
    * Derived from MemoryConnector \
    */ \
    void initConnector(); \
  };


/**
 * \def DEFINE_OUTPUT_CONNECTOR_(...)
 *
 * Implements the output connector struct for the inialization
 * of input/output connection of a BaseModule with the shared memory
 *
 * @param ... paranthesis enclosed array of variables to be
 *   defined (type, name), (type, name), ... (typelast, namelast),
 */
#define DEFINE_OUTPUT_CONNECTOR_(BaseClass, ...) \
    BaseClass::OutputConnector::OutputConnector( \
      BaseModule* connectingThread, string connectorName) : \
      OutputMemoryConnector(connectingThread, connectorName) \
    {} \
    \
    void BaseClass::OutputConnector::initConnector() \
    { \
      boost::shared_ptr<SharedMemory> sharedMemory = \
        connectingThread->getLocalSharedMemory(); \
      variables.assign(static_cast<int>(BaseClass::Output::COUNT), NULL); \
      memoryVariables.assign(static_cast<int>(BaseClass::Output::COUNT), NULL);  \
      FOR_EACH(DECLARE_OVARS_, __VA_ARGS__) \
    }
