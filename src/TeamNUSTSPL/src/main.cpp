/**
 * @file TeamNUSTSPL/src/main.cpp
 *
 * The start point for this software architecture.
 * Either connects to the robot of given ip and port or to
 * the robot in a simulator (vrep/gazebo-ros support available).
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 01 Feb 2017
 */

#ifdef V6_CROSS_BUILD
  #include <qi/applicationsession.hpp>
  #include <boost/shared_ptr.hpp>
#endif
#include <string>
#include "Utils/include/ConfigManager.h"
#include "Utils/include/PrintUtils.h"
#include "TeamNUSTSPL.h"

#ifndef V6_CROSS_BUILD
  #ifndef _WIN32
  #include <signal.h>
  #endif

  #ifdef MODULE_IS_REMOTE
  #define ALCALL
  #else
  #ifdef _WIN32
  #define ALCALL __declspec(dllexport)
  #else
  #define ALCALL
  #endif
  #endif
#endif

#ifdef MODULE_IS_REMOTE
  #ifndef V6_CROSS_BUILD
    #include "Utils/include/DataUtils.h"
    #include "Utils/include/ArgParseUtils.h"

    int USE_LOGGED_IMAGES = 0;
    int SAVE_IMAGES = -1;
    int PROJECT_FIELD = 0;
    string ROBOT_NAME = "Sim";

    int main(int argc, char* argv[]) {
      /**
       * The ip and port on which the robot is connected
       */
      int pport = 9559;
      string pip = "127.0.0.1";

      if(ArgParseUtils::cmdOptionExists(argv, argv+argc, "--pip"))
      {
        pip = ArgParseUtils::getCmdOption(argv, argv + argc, "--pip");
        if (!ArgParseUtils::cmdOptionExists(argv, argv+argc, "--robot"))
        {
          LOG_ERROR("Please give the argument --robot")
          exit(1);
        }
      }

      if(ArgParseUtils::cmdOptionExists(argv, argv+argc, "--pport"))
      {
        DataUtils::stringToVar(
          ArgParseUtils::getCmdOption(argv, argv + argc, "--pport"), pport);
      }

      if(ArgParseUtils::cmdOptionExists(argv, argv+argc, "--use-logged-images"))
      {
        USE_LOGGED_IMAGES = 1;
      }

      if(ArgParseUtils::cmdOptionExists(argv, argv+argc, "--test-projection"))
      {
        PROJECT_FIELD = 1;
      }

      if(ArgParseUtils::cmdOptionExists(argv, argv+argc, "--save-images"))
      {
        string option = ArgParseUtils::getCmdOption(argv, argv + argc, "--save-images");
        cout << "option: " << option << endl;
        if (option == "Top") {
          SAVE_IMAGES = 0;
        } else if (option == "Bottom") {
          SAVE_IMAGES = 1;
        }
        if(SAVE_IMAGES != 0 && SAVE_IMAGES != 1) {
          LOG_ERROR("Please add argument value Top/Bottom for required camera input.")
          exit(1);
        }
        LOG_INFO("Press ENTER key to start receiving image input from the concerned camera."
          "Please save images of a checker board from the concerned camera for calibration.\n"
          "Before Continuing, don't forget to update the file: \nConfig/CameraCalibration.xml.\n");
        if(cin.get() == '\n');
      }

      if(ArgParseUtils::cmdOptionExists(argv, argv+argc, "--robot"))
      {
        ROBOT_NAME = ArgParseUtils::getCmdOption(argv, argv + argc, "--robot");
        if (!(ROBOT_NAME == "Sim" ||
            ROBOT_NAME == "Nu-11" ||
            ROBOT_NAME == "Nu-12" ||
            ROBOT_NAME == "Nu-13" ||
            ROBOT_NAME == "Nu-14" ||
            ROBOT_NAME == "Nu-15"))
        {
          LOG_ERROR(
            "Invalid robot name: '" <<
            ROBOT_NAME <<
            "'. Possible names are: " <<
            "\n1. Nu-11\n2. Nu-12\n3. Nu-13\n4. Nu-14\n5. Nu-15\n6. Sim")
          exit(1);
        }
      }

      ConfigManager::setDirPaths("Robots/" + ROBOT_NAME + "/");
      ConfigManager::createDirs();

      /**
       * Need this to for SOAP serialization of floats to work. Not sure
       * what this is about as it comes from NaoQi.
       */
      setlocale(LC_NUMERIC, "C");

      /**
       * Broker name and listen port/ip
       */
      const string brokerName = "TeamNUSTSPL";
      int brokerPort = 54000;
      const string brokerIp = "0.0.0.0";

      /**
       * Broker definition with default configuration
       */
      boost::shared_ptr<AL::ALBroker> broker;
      try {
        broker =
        AL::ALBroker::createBroker(
          brokerName,
          brokerIp,
          brokerPort,
          pip,
          pport,
          0
        );
      } catch (...) {
        LOG_ERROR(
          "Fail to connect broker to: " + DataUtils::varToString(pip) +
          ":" + DataUtils::varToString(pport)
        )
        AL::ALBrokerManager::getInstance()->killAllBroker();
        AL::ALBrokerManager::kill();
        return 1;
      }

      /**
       * Deals with ALBrokerManager singleton (add your borker into NAOqi)
       */
      AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
      AL::ALBrokerManager::getInstance()->addBroker(broker);

      /**
       * Loading our local module
       * Usage: createModule<our_module>(<broker_create>, <our_module>)
       */
      AL::ALModule::createModule<TeamNUSTSPL>(broker, "TeamNUSTSPL");
      while (true);
      return 0;
    }
  #else
  ///< No capability of remote access in V6 robots for now
  #endif
#else
  #ifndef V6_CROSS_BUILD
    extern "C"
    {
      ALCALL int
      _createModule(boost::shared_ptr<AL::ALBroker> pBroker)
      {

        /**
         * Deals with ALBrokerManager singleton (add your borker into NAOqi)
         */
        AL::ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
        AL::ALBrokerManager::getInstance()->addBroker(pBroker);

        /**
         * Loading our local module
         * Usage: createModule<our_module>(<broker_create>, <our_module>)
         */
        AL::ALModule::createModule < TeamNUSTSPL > (pBroker, "TeamNUSTSPL");
        return 0;
      }

      ALCALL int
      _closeModule()
      {
        return 0;
      }
    }
  #else
  ///< New naoqi way of starting a module
  int main(int argc, char* argv[])
  {
    qi::ApplicationSession app(argc, argv);
    app.start();
    qi::SessionPtr session = app.session();
    session->registerService("TeamNUSTSPL", qi::AnyObject(boost::make_shared<TeamNUSTSPL>(session)));
    qi::AnyObject teamNUSTSPL = session->service("TeamNUSTSPL");
    teamNUSTSPL.async<void>("init");
    app.run();
    cout << "TeamNUSTSPL session finished." << endl;
    app.run();
    return 0;
  }
  #endif
#endif
