/**
 * @file GameCtrl.cpp
 * Implementation of a NAOqi library that communicates with the GameController.
 * It provides the data received in ALMemory.
 * It also implements the official button interface and sets the LEDs as
 * specified in the rules.
 *
 * @author Thomas Röfer
 */

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wconversion"
#pragma clang diagnostic ignored "-Wunused-variable"
#pragma clang diagnostic ignored "-Wunused-local-typedef"
#endif
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#ifndef V6_CROSS_BUILD
#include <alcommon/albroker.h>
#include <alcommon/alproxy.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>
#else
#include <iostream>
#include <qi/alvalue.h>
#include <qi/anyobject.hpp>
#include <qi/applicationsession.hpp>
#include <boost/shared_ptr.hpp>
#endif
#undef BOOST_SIGNALS_NO_DEPRECATION_WARNING
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include <arpa/inet.h>
#include <RoboCupGameControlData.h>
#include "UdpComm.h"

using namespace std;

static const int BUTTON_DELAY = 30; /**< Button state changes are ignored when happening in less than 30 ms. */
static const int GAMECONTROLLER_TIMEOUT = 2000; /**< Connected to GameController when packet was received within the last 2000 ms. */
static const int ALIVE_DELAY = 1000; /**< Send an alive signal every 1000 ms. */

enum Button
{
  chest,
  leftFootLeft,
  leftFootRight,
  rightFootLeft,
  rightFootRight,
  numOfButtons
};

static const char* buttonNames[] =
  { "Device/SubDeviceList/ChestBoard/Button/Sensor/Value",
    "Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value",
    "Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value",
    "Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value",
    "Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value" };

enum LED
{
  chestRed,
  chestGreen,
  chestBlue,
  leftFootRed,
  leftFootGreen,
  leftFootBlue,
  rightFootRed,
  rightFootGreen,
  rightFootBlue,
  numOfLEDs
};

static const char* ledNames[] =
  { "ChestBoard/Led/Red/Actuator/Value", "ChestBoard/Led/Green/Actuator/Value",
    "ChestBoard/Led/Blue/Actuator/Value", "LFoot/Led/Red/Actuator/Value",
    "LFoot/Led/Green/Actuator/Value", "LFoot/Led/Blue/Actuator/Value",
    "RFoot/Led/Red/Actuator/Value", "RFoot/Led/Green/Actuator/Value",
    "RFoot/Led/Blue/Actuator/Value" };

#ifndef V6_CROSS_BUILD
#define PLAYER_NUMBER *playerNumber
#define DEFAULT_TEAM_COLOR *defaultTeamColour
#define BUTTONS *buttons
#else
#define PLAYER_NUMBER playerNumber
#define DEFAULT_TEAM_COLOR defaultTeamColour
#define BUTTONS buttons
#endif

#ifndef V6_CROSS_BUILD
class GameCtrl : public AL::ALModule
#else
class GameCtrl
#endif
{
private:
  static GameCtrl* theInstance; /**< The only instance of this class. */

  #ifndef V6_CROSS_BUILD
  AL::DCMProxy* proxy; /**< Gives access to the DCM. */
  #else
  qi::AnyObject ledsProxy; /**< Gives access to the leds. */
  #endif
  #ifndef V6_CROSS_BUILD
  AL::ALMemoryProxy* memory; /**< Give access to ALMemory. */
  #else
  qi::SessionPtr session; /**< Naoqi module session. */
  qi::AnyObject memory; /**< Give access to ALMemory. */
  #endif
  AL::ALValue ledRequest; /**< Prepared request to set the LEDs. */
  UdpComm* udp; /**< The socket used to communicate. */
  in_addr gameControllerAddress; /**< The address of the GameController PC. */
  #ifndef V6_CROSS_BUILD
  const float* buttons[numOfButtons]; /**< Pointers to where ALMemory stores the current button states. */
  const int* playerNumber; /** Points to where ALMemory stores the player number. */
  const int* teamNumberPtr; /** Points to where ALMemory stores the team number. The number be set to 0 after it was read. */
  const int* defaultTeamColour; /** Points to where ALMemory stores the default team color. */
  #else
  float buttons[numOfButtons];
  int playerNumber;
  int defaultTeamColour;
  int prevPlayerNumber;
  int prevTeamColour;
  int prevTeamNumber;
  #endif
  int teamNumber; /**< The team number. */
  RoboCupGameControlData gameCtrlData; /**< The local copy of the GameController packet. */
  uint8_t previousState; /**< The game state during the previous cycle. Used to detect when LEDs have to be updated. */
  uint8_t previousSecondaryState; /**< The secondary game state during the previous cycle. Used to detect when LEDs have to be updated. */
  uint8_t previousKickOffTeam; /**< The kick-off team during the previous cycle. Used to detect when LEDs have to be updated. */
  uint8_t previousTeamColour; /**< The team colour during the previous cycle. Used to detect when LEDs have to be updated. */
  uint8_t previousPenalty; /**< The penalty set during the previous cycle. Used to detect when LEDs have to be updated. */
  bool previousChestButtonPressed; /**< Whether the chest button was pressed during the previous cycle. */
  bool previousLeftFootButtonPressed; /**< Whether the left foot bumper was pressed during the previous cycle. */
  bool previousRightFootButtonPressed; /**< Whether the right foot bumper was pressed during the previous cycle. */
  unsigned whenChestButtonStateChanged; /**< When last state change of the chest button occured (DCM time). */
  unsigned whenLeftFootButtonStateChanged; /**< When last state change of the left foot bumper occured (DCM time). */
  unsigned whenRightFootButtonStateChanged; /**< When last state change of the right foot bumper occured (DCM time). */
  unsigned whenPacketWasReceived; /**< When the last GameController packet was received (DCM time). */
  unsigned whenPacketWasSent; /**< When the last return packet was sent to the GameController (DCM time). */

  /**
   * Resets the internal state when an application was just started.
   */
  void
  init()
  {
    memset(&gameControllerAddress, 0, sizeof(gameControllerAddress));
    #ifdef V6_CROSS_BUILD
    prevPlayerNumber = -1;
    prevTeamColour = -1;
    #endif
    previousState = (uint8_t) - 1;
    previousSecondaryState = (uint8_t) - 1;
    previousKickOffTeam = (uint8_t) - 1;
    previousTeamColour = (uint8_t) - 1;
    previousPenalty = (uint8_t) - 1;
    previousChestButtonPressed = false;
    previousLeftFootButtonPressed = false;
    previousRightFootButtonPressed = false;
    whenChestButtonStateChanged = 0;
    whenLeftFootButtonStateChanged = 0;
    whenRightFootButtonStateChanged = 0;
    whenPacketWasReceived = 0;
    whenPacketWasSent = 0;
    memset(&gameCtrlData, 0, sizeof(gameCtrlData));
  }

  /**
   * Sets the LEDs whenever the state they visualize changes.
   * Regularily sends the return packet to the GameController.
   */
  void handleOutput()
  {
    //cout << "Handle output..." << endl;
    #ifndef V6_CROSS_BUILD
      unsigned now = (unsigned) proxy->getTime(0);
    #else
      unsigned now = qi::os::ustime() / 1e3;
    #endif
    /*cout << "teamNumber: " << teamNumber << endl;
    cout << "PLAYER_NUMBER: " << PLAYER_NUMBER << endl;
    cout << "gameCtrlData.playersPerTeam: " << (int)gameCtrlData.playersPerTeam << endl;
    cout << "gameCtrlData.teams[0].teamNumber:" << (int)gameCtrlData.teams[0].teamNumber << endl;*/
    if (teamNumber && PLAYER_NUMBER && PLAYER_NUMBER <= gameCtrlData.playersPerTeam && (gameCtrlData.teams[0].teamNumber == teamNumber || gameCtrlData.teams[1].teamNumber == teamNumber)) {
      const TeamInfo& team = gameCtrlData.teams[
        gameCtrlData.teams[0].teamNumber == teamNumber ? 0 : 1];
      /*cout << "gameCtrlData.state: " << (int)gameCtrlData.state << endl;
      cout << "gameCtrlData.previousState: " <<(int) previousState << endl;
      cout << "gameCtrlData.secondaryState: " << (int)gameCtrlData.secondaryState << endl;
      cout << "gameCtrlData.previousSecondaryState: " <<(int) previousSecondaryState << endl;
      cout << "gameCtrlData.kickOffTeam: " << (int)gameCtrlData.kickOffTeam << endl;
      cout << "gameCtrlData.previousKickOffTeam: " <<(int) previousKickOffTeam << endl;
      cout << "gameCtrlData.teamColour: " << (int)gameCtrlData.teams[0].teamColour << endl;
      cout << "gameCtrlData.previousTeamColour: " <<(int) previousTeamColour << endl;
      cout << "gameCtrlData.penalty: " << (int)team.players[PLAYER_NUMBER - 1].penalty << endl;
      cout << "gameCtrlData.previousPenalty: " <<(int) previousPenalty << endl;*/
      if (gameCtrlData.state != previousState || gameCtrlData.secondaryState != previousSecondaryState || gameCtrlData.kickOffTeam != previousKickOffTeam || team.teamColour != previousTeamColour || team.players[PLAYER_NUMBER - 1].penalty != previousPenalty) {
        switch (team.teamColour) {
        case TEAM_BLUE:
          setLED(leftFootRed, 0.f, 0.f, 1.f);
          break;
        case TEAM_RED:
          setLED(leftFootRed, 1.f, 0.f, 0.f);
          break;
        case TEAM_YELLOW:
          setLED(leftFootRed, 1.f, 1.f, 0.f);
          break;
        default:
          setLED(leftFootRed, 0.f, 0.f, 0.f);
        }

        if (gameCtrlData.state == STATE_INITIAL && gameCtrlData.secondaryState == STATE2_PENALTYSHOOT && gameCtrlData.kickOffTeam == team.teamNumber) setLED(
          rightFootRed,
          0.f,
          1.f,
          0.f);
        else if (gameCtrlData.state == STATE_INITIAL && gameCtrlData.secondaryState == STATE2_PENALTYSHOOT && gameCtrlData.kickOffTeam != team.teamNumber) setLED(
          rightFootRed,
          1.f,
          1.0f,
          0.f);
        else if (now - whenPacketWasReceived < GAMECONTROLLER_TIMEOUT && gameCtrlData.state <= STATE_SET && gameCtrlData.kickOffTeam == team.teamNumber) setLED(
          rightFootRed,
          1.f,
          1.f,
          1.f);
        else setLED(rightFootRed, 0.f, 0.f, 0.f);

        if (team.players[PLAYER_NUMBER - 1].penalty != PENALTY_NONE) setLED(
          chestRed,
          1.f,
          0.f,
          0.f);
        else switch (gameCtrlData.state) {
        case STATE_READY:
          setLED(chestRed, 0.f, 0.f, 1.f);
          break;
        case STATE_SET:
          setLED(chestRed, 1.f, 1.0f, 0.f);
          break;
        case STATE_PLAYING:
          setLED(chestRed, 0.f, 1.f, 0.f);
          break;
        default:
          setLED(chestRed, 0.f, 0.f, 0.f);
        }

        #ifndef V6_CROSS_BUILD
        ledRequest[4][0] = (int) now;
        proxy->setAlias(ledRequest);
        #endif

        previousState = gameCtrlData.state;
        previousSecondaryState = gameCtrlData.secondaryState;
        previousKickOffTeam = gameCtrlData.kickOffTeam;
        previousTeamColour = team.teamColour;
        previousPenalty = team.players[PLAYER_NUMBER - 1].penalty;
      }
      if (now - whenPacketWasReceived < GAMECONTROLLER_TIMEOUT && now - whenPacketWasSent >= ALIVE_DELAY && send(
        GAMECONTROLLER_RETURN_MSG_ALIVE)) whenPacketWasSent = now;
    }
  }

  /**
   * Sets states in the LED request.
   * @param led The index of the red channel of an RGB LED.
   * @param red The red intensity [0..1].
   * @param green The green intensity [0..1].
   * @param blue The blue intensity [0..1].
   */
  void
  setLED(LED led, float red, float green, float blue)
  {
    #ifndef V6_CROSS_BUILD
    ledRequest[5][led][0] = red;
    ledRequest[5][led + 1][0] = green;
    ledRequest[5][led + 2][0] = blue;
    #else
    ledsProxy.call<void>("setIntensity", ledNames[led], red);
    ledsProxy.call<void>("setIntensity", ledNames[led + 1], green);
    ledsProxy.call<void>("setIntensity", ledNames[led + 2], blue);
    #endif
  }

  /**
   * Handles the button interface.
   * Resets the internal state when a new team number was set.
   * Receives packets from the GameController.
   * Initializes gameCtrlData when teamNumber and playerNumber are available.
   */
  void
  handleInput()
  {
    #ifndef V6_CROSS_BUILD
      unsigned now = (unsigned) proxy->getTime(0);
    #else
      unsigned now = qi::os::ustime() / 1e3;
    #endif

    #ifdef V6_CROSS_BUILD
      for (int i = 0; i < numOfButtons; ++i)
        buttons[i] = memory.call<float>("getData", buttonNames[i]);
    #endif

    #ifndef V6_CROSS_BUILD
    if (*teamNumberPtr != 0) { // new team number was set -> reset internal structure
      teamNumber = *teamNumberPtr;
      memory->insertData("GameCtrl/teamNumber", 0);
      init();
    }
    #else
    teamNumber = memory.call<int>("getData", "GameCtrl/teamNumber");
    if (teamNumber != 0 && teamNumber != prevTeamNumber) { // new team number was set -> reset internal structure
      init();
      prevTeamNumber = teamNumber;
    }
    #endif

    #ifdef V6_CROSS_BUILD
    auto pn = memory.call<int>("getData", "GameCtrl/playerNumber");
    auto tc = memory.call<int>("getData", "GameCtrl/teamColour");
    if (pn != prevPlayerNumber) {
      playerNumber = pn;
      prevPlayerNumber = playerNumber;
    }
    if (tc != prevTeamColour) {
      defaultTeamColour = tc;
      prevTeamColour = defaultTeamColour;
    }
    #endif

    if (receive()) {
      if (!whenPacketWasReceived) previousState = (uint8_t) - 1; // force LED update on first packet received
      whenPacketWasReceived = now;
      publish();
    }

    if (teamNumber && PLAYER_NUMBER) {
      // init gameCtrlData if invalid
      if (gameCtrlData.teams[0].teamNumber != teamNumber && gameCtrlData.teams[1].teamNumber != teamNumber) {
        uint8_t teamColour = (uint8_t) DEFAULT_TEAM_COLOR;
        if (teamColour != TEAM_BLUE && teamColour != TEAM_RED && teamColour != TEAM_YELLOW) teamColour =
          TEAM_BLACK;
        gameCtrlData.teams[0].teamNumber = (uint8_t) teamNumber;
        gameCtrlData.teams[0].teamColour = teamColour;
        gameCtrlData.teams[1].teamColour = teamColour ^ 1; // we don't know better
        if (!gameCtrlData.playersPerTeam) gameCtrlData.playersPerTeam =
          (uint8_t) PLAYER_NUMBER; // we don't know better
        publish();
      }
      TeamInfo& team = gameCtrlData.teams[
        gameCtrlData.teams[0].teamNumber == teamNumber ? 0 : 1];

      if (PLAYER_NUMBER <= gameCtrlData.playersPerTeam) {
        bool chestButtonPressed = BUTTONS[chest] != 0.f;
        if (chestButtonPressed != previousChestButtonPressed && now - whenChestButtonStateChanged >= BUTTON_DELAY) {
          if (chestButtonPressed && whenChestButtonStateChanged) // ignore first press, e.g. for getting up
          {
            RobotInfo& player = team.players[PLAYER_NUMBER - 1];
            if (player.penalty == PENALTY_NONE) {
              player.penalty = PENALTY_MANUAL;
              if (now - whenPacketWasReceived < GAMECONTROLLER_TIMEOUT && send(
                GAMECONTROLLER_RETURN_MSG_MAN_PENALISE)) whenPacketWasSent =
                now;
            } else {
              player.penalty = PENALTY_NONE;
              if (now - whenPacketWasReceived < GAMECONTROLLER_TIMEOUT && send(
                GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE)) whenPacketWasSent =
                now;
              else gameCtrlData.state = STATE_PLAYING;
            }
            publish();
          }

          previousChestButtonPressed = chestButtonPressed;
          whenChestButtonStateChanged = now;
        }

        if (gameCtrlData.state == STATE_INITIAL) {
          bool leftFootButtonPressed =
            BUTTONS[leftFootLeft] != 0.f || BUTTONS[leftFootRight] != 0.f;
          if (leftFootButtonPressed != previousLeftFootButtonPressed && now - whenLeftFootButtonStateChanged >= BUTTON_DELAY) {
            if (leftFootButtonPressed) {
              team.teamColour = (team.teamColour + 1) & 3; // cycle between TEAM_BLUE .. TEAM_BLACK
              publish();
            }
            previousLeftFootButtonPressed = leftFootButtonPressed;
            whenLeftFootButtonStateChanged = now;
          }

          bool rightFootButtonPressed =
            BUTTONS[rightFootLeft] != 0.f || BUTTONS[rightFootRight] != 0.f;
          if (rightFootButtonPressed != previousRightFootButtonPressed && now - whenRightFootButtonStateChanged >= BUTTON_DELAY) {
            if (rightFootButtonPressed) {
              if (gameCtrlData.secondaryState == STATE2_NORMAL) {
                gameCtrlData.secondaryState = STATE2_PENALTYSHOOT;
                gameCtrlData.kickOffTeam = team.teamNumber;
              } else if (gameCtrlData.kickOffTeam == team.teamNumber) gameCtrlData.kickOffTeam =
                0;
              else gameCtrlData.secondaryState = STATE2_NORMAL;
              publish();
            }
            previousRightFootButtonPressed = rightFootButtonPressed;
            whenRightFootButtonStateChanged = now;
          }
        }
      } else fprintf(
        stderr,
        "Player number %d too big. Maximum number is %d.\n",
        PLAYER_NUMBER,
        gameCtrlData.playersPerTeam);
    }
  }

  /**
   * Sends the return packet to the GameController.
   * @param message The message contained in the packet (GAMECONTROLLER_RETURN_MSG_MAN_PENALISE,
   *                GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE or GAMECONTROLLER_RETURN_MSG_ALIVE).
   */
  bool
  send(uint8_t message)
  {
    RoboCupGameControlReturnData returnPacket;
    returnPacket.team = (uint8_t) teamNumber;
    returnPacket.player = (uint8_t) PLAYER_NUMBER;
    returnPacket.message = message;
    return !udp || udp->write((const char*) &returnPacket, sizeof(returnPacket));
  }

  /**
   * Receives a packet from the GameController.
   * Packets are only accepted when the team number is know (nonzero) and
   * they are addressed to this team.
   */
  bool
  receive()
  {
    bool received = false;
    int size;
    RoboCupGameControlData buffer;
    struct sockaddr_in from;
    while (udp && (size = udp->read((char*) &buffer, sizeof(buffer), from)) > 0) {
      if (size == sizeof(buffer) && !std::memcmp(
        &buffer,
        GAMECONTROLLER_STRUCT_HEADER,
        4) && buffer.version == GAMECONTROLLER_STRUCT_VERSION && teamNumber && (buffer.teams[0].teamNumber == teamNumber || buffer.teams[1].teamNumber == teamNumber)) {
        gameCtrlData = buffer;
        if (memcmp(&gameControllerAddress, &from.sin_addr, sizeof(in_addr))) {
          memcpy(&gameControllerAddress, &from.sin_addr, sizeof(in_addr));
          udp->setTarget(
            inet_ntoa(gameControllerAddress),
            GAMECONTROLLER_RETURN_PORT);
        }

        received = true;
      }
    }
    return received;
  }

  /**
   * Publishes the current state of the GameController packet in ALMemory.
   */
  void
  publish()
  {
    AL::ALValue value((const char*) &gameCtrlData, sizeof(gameCtrlData));
    #ifndef V6_CROSS_BUILD
    memory->insertData("GameCtrl/RoboCupGameControlData", value);
    #else
    memory.call<void>("insertData","GameCtrl/RoboCupGameControlData", value);
    #endif
  }

  /**
   * Close all resources acquired.
   * Called when initialization failed or during destruction.
   */
  void
  close()
  {
    if (udp) delete udp;
    #ifndef V6_CROSS_BUILD
    if (proxy) {
      proxy->getGenericProxy()->getModule()->removeAllPreProcess();
      proxy->getGenericProxy()->getModule()->removeAllPostProcess();
      delete proxy;
    }
    if (memory) delete memory;
    #endif
  }

  #ifndef V6_CROSS_BUILD
  /**
   * The method is called by NAOqi immediately before it communicates with the chest board.
   * It sets all the actuators.
   */
  static void
  onPreProcess()
  {
    theInstance->handleOutput();
  }

  /**
   * The method is called by NAOqi immediately after it communicated with the chest board.
   * It reads all sensors.
   */
  static void
  onPostProcess()
  {
    theInstance->handleInput();
  }
  #endif
public:
  /**
   * The constructor sets up the structures required to communicate with NAOqi.
   * @param pBroker A NAOqi broker that allows accessing other NAOqi modules.
   */
  #ifndef V6_CROSS_BUILD
  GameCtrl(boost::shared_ptr<AL::ALBroker> pBroker) :
    ALModule(pBroker, "GameCtrl"), proxy(0), memory(0), udp(0), teamNumber(0)
  #else
  GameCtrl(qi::SessionPtr session) :
    session(session), udp(0), teamNumber(0)
  #endif
  {
    #ifndef V6_CROSS_BUILD
      setModuleDescription(
        "A module that provides packets from the GameController.");
    #endif

    assert(numOfButtons == sizeof(buttonNames) / sizeof(*buttonNames));
    assert(numOfLEDs == sizeof(ledNames) / sizeof(*ledNames));

    init();

    try {
      #ifndef V6_CROSS_BUILD
      memory = new AL::ALMemoryProxy(pBroker);
      proxy = new AL::DCMProxy(pBroker);
      #else
      memory = session->service("ALMemory");
      ledsProxy = session->service("ALLeds");
      #endif

      #ifndef V6_CROSS_BUILD
      AL::ALValue params;
      AL::ALValue result;
      params.arraySetSize(1);
      params.arraySetSize(2);

      params[0] = std::string("leds");
      params[1].arraySetSize(numOfLEDs);
      for (int i = 0; i < numOfLEDs; ++i)
        params[1][i] = std::string(ledNames[i]);
      result = proxy->createAlias(params);
      assert(result == params);

      ledRequest.arraySetSize(6);
      ledRequest[0] = std::string("leds");
      ledRequest[1] = std::string("ClearAll");
      ledRequest[2] = std::string("time-separate");
      ledRequest[3] = 0;
      ledRequest[4].arraySetSize(1);
      ledRequest[5].arraySetSize(numOfLEDs);
      for (int i = 0; i < numOfLEDs; ++i)
        ledRequest[5][i].arraySetSize(1);
      #endif

      #ifndef V6_CROSS_BUILD
        for (int i = 0; i < numOfButtons; ++i)
          buttons[i] = (float*) memory->getDataPtr(buttonNames[i]);
      #else
        for (int i = 0; i < numOfButtons; ++i)
          buttons[i] = memory.call<float>("getData", buttonNames[i]);
      #endif

      // If no color was set, set it to black (no LED).
      // This actually has a race condition.
      #ifndef V6_CROSS_BUILD
      if (memory->getDataList("GameCtrl/teamColour").empty())
        memory->insertData(
        "GameCtrl/teamColour",
        TEAM_BLACK);
      #else
      if (memory.call<AL::ALValue>("getDataList", "GameCtrl/teamColour").getSize() == 0)
        memory.call<void>("insertData", "GameCtrl/teamColour", TEAM_BLACK);
      #endif

      #ifndef V6_CROSS_BUILD
      playerNumber = (int*) memory->getDataPtr("GameCtrl/playerNumber");
      teamNumberPtr = (int*) memory->getDataPtr("GameCtrl/teamNumber");
      defaultTeamColour = (int*) memory->getDataPtr("GameCtrl/teamColour");
      #else
      playerNumber = 1;
      teamNumber = 30;
      defaultTeamColour = 0;
      memory.call<void>("insertData", "GameCtrl/playerNumber", playerNumber);
      memory.call<void>("insertData", "GameCtrl/teamNumber", teamNumber);
      memory.call<void>("insertData", "GameCtrl/teamColour", defaultTeamColour);
      #endif

      theInstance = this;
      #ifndef V6_CROSS_BUILD
      // register "onPreProcess" and "onPostProcess" callbacks
      proxy->getGenericProxy()->getModule()->atPreProcess(&onPreProcess);
      proxy->getGenericProxy()->getModule()->atPostProcess(&onPostProcess);
      #endif

      udp = new UdpComm();
      if (!udp->setBlocking(false) || !udp->setBroadcast(true) || !udp->bind(
        "0.0.0.0",
        GAMECONTROLLER_DATA_PORT) || !udp->setLoopback(false)) {
        fprintf(stderr, "libgamectrl: Could not open UDP port\n");
        delete udp;
        udp = 0;
        // continue, because button interface will still work
      }

      publish();
    } catch (std::exception& e) {
      fprintf(stderr, "libgamectrl: %s\n", e.what());
      close();
    }
  }

  #ifdef V6_CROSS_BUILD
  void mainRoutine()
  {
    while(true) {
      this->handleInput();
      this->handleOutput();
      qi::os::sleep(0.1);
    }
  }
  #endif


  /**
   * Close all resources acquired.
   */
  ~GameCtrl()
  {
    close();
  }
};

GameCtrl* GameCtrl::theInstance = 0;

/**
 * This method is called by NAOqi when loading this library.
 * Creates an instance of class GameCtrl.
 * @param pBroker A NAOqi broker that allows accessing other NAOqi modules.
 */
#ifndef V6_CROSS_BUILD
extern "C" int
_createModule(boost::shared_ptr<AL::ALBroker> pBroker)
{
  AL::ALModule::createModule < GameCtrl > (pBroker);
  return 0;
}
#else
///< New naoqi way of starting a module
int main(int argc, char* argv[])
{
  qi::ApplicationSession app(argc, argv);
  app.start();
  qi::SessionPtr session = app.session();
  session->registerService("GameCtrl", qi::AnyObject(boost::make_shared<GameCtrl>(session)));
  qi::AnyObject gameCtrl = session->service("GameCtrl");
  gameCtrl.async<void>("mainRoutine");
  app.run();
  cout << "GameCtrl session finished." << endl;
  return 0;
}
#endif

#ifdef V6_CROSS_BUILD
QI_REGISTER_MT_OBJECT(GameCtrl, mainRoutine);
#endif
