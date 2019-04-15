/**
 * @file LolaModule/include/LolaModule.h
 *
 * This file declares the class LolaModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <msgpack/include/msgpack.hpp>
#include <opencv2/core/core.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <type_traits>
#include "TNRSBase/include/BaseIncludes.h"
using namespace boost::asio;
using namespace std;
class LolaFrameHandler;
class SensorLayer;
class ActuatorLayer;

/**
 * @class LolaModule
 * @brief A module that handles lola communication and updates
 *   sensors/actuators
 */
class LolaModule : public BaseModule
{
  DECLARE_INPUT_CONNECTOR(
    lolaThreadPeriod
  );
  DECLARE_OUTPUT_CONNECTOR(
    lolaThreadTimeTaken,
    jointPositionSensors,
    jointStiffnessSensors,
    handSensors,
    inertialSensors,
    fsrSensors,
    ledSensors,
    jointTemperatureSensors,
    jointCurrentSensors,
    touchSensors,
    switchSensors,
    batterySensors,
    sonarSensors
  );
public:
  /**
   * @brief LolaModule Constructor
   * @param teamNUSTSPL pointer to base class which is in this case
   *   the TeamNUSTSPL class
   */
  LolaModule(void* teamNUSTSPL);

  /**
   * @brief ~LolaModule Destructor
   */
  ~LolaModule();

  /**
   * See BaseModule::init()
   */
  void init() final;

  /**
   * See BaseModule::mainRoutine()
   */
  void mainRoutine() final;

  /**
   * See BaseModule::handleRequests()
   */
  void handleRequests() final;

  /**
   * See BaseModule::initMemoryConn()
   */
  void initMemoryConn() final;

  /**
   * See BaseModule::setThreadPeriod()
   */
  void setThreadPeriod() final;

  /**
   * @brief setThreadTimeTaken See BaseModule::setThreadTimeTaken()
   */
  void setThreadTimeTaken() final;

  /**
   * @brief join Calls join on child threads along with its own thread
   */
  void join() override {
    BaseModule::join();
  }

private:
  #ifdef V6_CROSS_BUILD
  /**
   * @brief sensorsUpdate Updates sensor values from NaoQi
   *   ALMemory to our local shared memory
   */
  void sensorsUpdate();

  /**
   * @brief actuatorsUpdate Sends the requested actuator commands
   *   to NaoQi DCM for execution
   */
  void actuatorsUpdate();
  #endif

  static constexpr int maxLen = {100000};
  char data[maxLen] = {'\0'};
  msgpack::sbuffer inBuffer;
  msgpack::sbuffer outBuffer;

  vector<boost::shared_ptr<SensorLayer> > sensorLayers;
  vector<boost::shared_ptr<ActuatorLayer> > actuatorLayers;

  ///< Boost io service for Lola connection
  boost::shared_ptr<boost::asio::io_service > ioService;

  ///< Socket for communication
  boost::shared_ptr<local::stream_protocol::socket> socket;

  #ifdef V6_CROSS_BUILD
  /**
   * @enum LolaSensors
   * @brief Enumeration for the sensors handled by this class
   */
  enum class LolaSensors : unsigned {
    jointPosition,
    jointStiffnesses,
    handSensors,
    inertial,
    fsr,
    led,
    jointTemps,
    jointCurrents,
    touchSensors,
    switchSensors,
    batterySensors,
    count
  };

  /**
   * @enum LolaActuators
   * @brief Enumeration for the actuators handled by this class
   */
  enum class LolaActuators : unsigned {
    jointActuators,
    handActuators,
    jointStiffnesses,
    led,
    count
  };
  #endif
};
