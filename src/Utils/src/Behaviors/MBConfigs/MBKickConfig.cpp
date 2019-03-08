/**
 * @file MotionModule/src/MotionConfigs/MBKickConfig.cpp
 *
 * This file implements the structs MBKickConfig, JSKickConfig, 
 * JSE2DImpKickConfig and JSOImpKickConfig
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "Utils/include/Behaviors/MBConfigs/MBKickConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBBalanceConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBPostureConfig.h"

MBKickConfig::MBKickConfig(
  const MBKickTypes& type) :
  MBConfig(MBIds::kick, 20.f, (int)type)
{
}

MBKickConfig::MBKickConfig(
  const MBKickTypes& type,
  const Point2f& ball) :
  MBConfig(MBIds::kick, 20.f, (int)type),
  ball(ball)
{
}

MBKickConfig::MBKickConfig(
  const MBKickTypes& type,
  const Point2f& ball, 
  const MBPostureConfigPtr& postureConfig) :
  MBConfig(MBIds::kick, 20.f, (int)type),
  ball(ball),
  postureConfig(postureConfig)
{
}


MBKickConfig::MBKickConfig(
  const MBKickTypes& type,
  const Point2f& ball,
  const MBPostureConfigPtr& postureConfig,
  const MBBalanceConfigPtr& balanceConfig) :
  MBConfig(MBIds::kick, 20.f, (int)type),
  ball(ball),
  postureConfig(postureConfig),
  balanceConfig(balanceConfig)
{
}

MBKickConfig::MBKickConfig(
  const MBKickTypes& type,
  const Point2f& ball,
  const MBBalanceConfigPtr& balanceConfig) :
  MBConfig(MBIds::kick, 20.f, (int)type),
  ball(ball),
  balanceConfig(balanceConfig)
{
}


bool MBKickConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBConfig::assignFromJson(obj))
    return false;
  //LOG_INFO("MBKickConfig::assignFromJson() called...")
  try {
    ball.x = obj["ball"][0].asFloat();
    ball.y = obj["ball"][1].asFloat();
    reqVel.x = obj["reqVel"][0].asFloat();
    reqVel.y = obj["reqVel"][1].asFloat();
    target.x = obj["target"][0].asFloat();
    target.y = obj["target"][1].asFloat();
    targetDistAngle[0] = obj["targetDistAngle"][0].asFloat();
    targetDistAngle[1] = obj["targetDistAngle"][1].asFloat();
    const Json::Value& cfgObj = obj["postureConfig"];
    if (!cfgObj.isNull()) {
      auto pcfg = MBPostureConfig::makeFromJson(cfgObj);
      if(pcfg->assignFromJson(cfgObj)) {
        postureConfig = pcfg;
      }
    }
    const Json::Value& bCfgObj = obj["balanceConfig"];
    if (!bCfgObj.isNull()) {
      auto bcfg = MBBalanceConfig::makeFromJson(bCfgObj);
      if(bcfg->assignFromJson(bCfgObj)) {
        balanceConfig = bcfg;
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught in behavior config \n\tType: "
      << static_cast<unsigned>(baseType)
      << ", Id: " 
      << DataUtils::varToString(id) 
      << ";\t";
    LOG_EXCEPTION(e.what());
    return false;
  }
  return true;
} 

Json::Value MBKickConfig::getJson() 
{
  Json::Value obj = MBConfig::getJson();
  try {
      obj["ball"].append(ball.x);
      obj["ball"].append(ball.y);
      obj["reqVel"].append(reqVel.x);
      obj["reqVel"].append(reqVel.y);
      obj["target"].append(target.x);
      obj["target"].append(target.y);
      obj["targetDistAngle"].append(targetDistAngle[0]);
      obj["targetDistAngle"].append(targetDistAngle[1]);
      if (postureConfig)
        obj["postureConfig"] = postureConfig->getJson();
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught in behavior config \n\tType: "
      << static_cast<unsigned>(baseType)
      << ", Id: " 
      << DataUtils::varToString(id) 
      << ";\t";
    LOG_EXCEPTION(e.what());
  }
  return obj;
}

MBKickConfigPtr MBKickConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBKickConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == toUType(MBIds::kick) &&
        obj["baseType"].asUInt() == toUType(BaseBehaviorType::motion) &&
        obj["type"].asUInt() <= toUType(MBKickTypes::count)
    ) {
      switch (obj["type"].asUInt()) {
        case toUType(MBKickTypes::jsoImpKick):
          config = 
            boost::make_shared<JSOImpKickConfig>(); break;
        case toUType(MBKickTypes::jse2DImpKick):
          config = 
            boost::make_shared<JSE2DImpKickConfig>(); break;
        case toUType(MBKickTypes::cSpaceBSplineKick):
          config =
            boost::make_shared<CSpaceBSplineKickConfig>(); break;
      }
      if (!config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a configuration from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a kick behavior config from json object;\t";
    LOG_EXCEPTION(e.what());
    config.reset();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    config.reset();
  }
  return config;
}

JSKickConfig::JSKickConfig(
  const MBKickTypes& type,
  const Point2f& ball, 
  const boost::shared_ptr<MBPostureConfig>& postureConfig,
  const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
  const float& minTimeToKick) :
  MBKickConfig(type, ball, postureConfig, balanceConfig),
  minTimeToKick(minTimeToKick)
{
  target = Point2f(-1.f, -1.f);
  reqVel = Point2f(-1.f, -1.f);
}

JSKickConfig::JSKickConfig(
const MBKickTypes& type,
  const Point2f& ball, 
  const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
  const float& minTimeToKick) :
  MBKickConfig(type, ball, balanceConfig),
  minTimeToKick(minTimeToKick)
{
  target = Point2f(-1.f, -1.f);
  reqVel = Point2f(-1.f, -1.f);
}

JSKickConfig::JSKickConfig(
  const MBKickTypes& type,
  const Point2f& ball, 
  const float& minTimeToKick) :
  MBKickConfig(type, ball),
  minTimeToKick(minTimeToKick)
{
  target = Point2f(-1.f, -1.f);
  reqVel = Point2f(-1.f, -1.f);
}

bool JSKickConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBKickConfig::assignFromJson(obj))
    return false;
  //LOG_INFO("JSKickConfig::assignFromJson() called...")
  try {
    minTimeToKick = obj.get("minTimeToKick", -1.f).asFloat();
    inKickBalance = obj["inKickBalance"].asBool();
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught in behavior config \n\tType: "
      << static_cast<unsigned>(baseType)
      << ", Id: " 
      << DataUtils::varToString(id) 
      << ";\t";
    LOG_EXCEPTION(e.what());
    return false;
  }
  return true;
}

Json::Value JSKickConfig::getJson() { 
  Json::Value obj = MBKickConfig::getJson();
  try {
    obj["minTimeToKick"] = minTimeToKick; // Reassign name
    obj["inKickBalance"] = inKickBalance;
    if (balanceConfig)
      obj["balanceConfig"] = balanceConfig->getJson();
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught in behavior config \n\tType: "
      << static_cast<unsigned>(baseType)
      << ", Id: " 
      << DataUtils::varToString(id) 
      << ";\t";
    LOG_EXCEPTION(e.what());
  }
  return obj;
}

JSOImpKickConfig::JSOImpKickConfig(
  const Point2f& ball, 
  const boost::shared_ptr<MBPostureConfig>& postureConfig,
  const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
  const float& minTimeToKick) :
  JSKickConfig(
    MBKickTypes::jsoImpKick,
    ball, 
    postureConfig,
    balanceConfig,
    minTimeToKick
  )
{
}

JSOImpKickConfig::JSOImpKickConfig(
  const Point2f& ball,  
  const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
  const float& minTimeToKick) :
  JSKickConfig(
    MBKickTypes::jsoImpKick,
    ball,
    balanceConfig,
  minTimeToKick
  )
{
}

JSOImpKickConfig::JSOImpKickConfig(
  const Point2f& ball,  
  const float& minTimeToKick) :
  JSKickConfig(
    MBKickTypes::jsoImpKick,
    ball,
    minTimeToKick
  )
{
  //! Default config for balancing
  balanceConfig = boost::make_shared<MPComControlConfig>();
}

void JSOImpKickConfig::validate() 
{
  if (ball.x < 0.f ||
      //(reqVel.x == -1.f && target.x == -1.f) ||
      minTimeToKick <= 0.f)
  { 
    cout << "Parameters: \n" << endl;
    cout << "Ball: " << ball << endl;
    cout << "ReqVel: " << reqVel << endl;
    cout << "target: " << target << endl;
    cout << "minTimeToKick: " << minTimeToKick << endl;
    throw 
      BConfigException(
        this, 
        "Invalid behavior configuration parameters passed.", 
        false, 
        EXC_INVALID_BCONFIG_PARAMETERS
      );
  }
}

bool JSOImpKickConfig::assignFromJson(const Json::Value& obj)
{
  if (!JSKickConfig::assignFromJson(obj))
    return false;
  return true;
}

Json::Value JSOImpKickConfig::getJson() 
{ 
  Json::Value obj = JSKickConfig::getJson();
  return obj;
}

JSE2DImpKickConfig::JSE2DImpKickConfig(
  const Point2f& ball, 
  const Point2f& ballVel,
  const double& timeUntilImpact,
  const high_resolution_clock::time_point& timeAtEstimation,
  const boost::shared_ptr<MBPostureConfig>& postureConfig,
  const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
  const float& minTimeToKick) :
  JSKickConfig(
    MBKickTypes::jse2DImpKick,
    ball, 
    postureConfig,
    balanceConfig,
    minTimeToKick
  ),
  ballVel(ballVel),
  timeUntilImpact(timeUntilImpact),
  timeAtEstimation(timeAtEstimation)
{
}

JSE2DImpKickConfig::JSE2DImpKickConfig(
  const Point2f& ball,  
  const Point2f& ballVel,
  const double& timeUntilImpact,
  const high_resolution_clock::time_point& timeAtEstimation,
  const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
  const float& minTimeToKick) :
  JSKickConfig(
    MBKickTypes::jse2DImpKick,
    ball,
    balanceConfig,
  minTimeToKick
  ),
  ballVel(ballVel),
  timeUntilImpact(timeUntilImpact),
  timeAtEstimation(timeAtEstimation)
{
}

JSE2DImpKickConfig::JSE2DImpKickConfig(
  const Point2f& ball,  
  const Point2f& ballVel,
  const double& timeUntilImpact,
  const high_resolution_clock::time_point& timeAtEstimation,
  const float& minTimeToKick) :
  JSKickConfig(
    MBKickTypes::jse2DImpKick,
    ball,
    minTimeToKick
  ),
  ballVel(ballVel),
  timeUntilImpact(timeUntilImpact),
  timeAtEstimation(timeAtEstimation)
{
}

JSE2DImpKickConfig::JSE2DImpKickConfig(
  const Point2f& ball,  
  const Point2f& ballVel,
  const double& timeUntilImpact,
  const float& minTimeToKick) :
  JSKickConfig(
    MBKickTypes::jse2DImpKick,
    ball,
    minTimeToKick
  ),
  ballVel(ballVel),
  timeUntilImpact(timeUntilImpact),
  timeAtEstimation(timeAtEstimation)
{
}

void JSE2DImpKickConfig::validate() 
{
  if (ball.x < 0.f ||
      //(target.x == -1.f && targetDistAngle[0] == -1.f) ||
      minTimeToKick <= 0.f || 
      timeUntilImpact < 0.5f) // Minimum time given to kick should be 0.5 secs
  { 
    cout << "Parameters: \n" << endl;
    cout << "Ball: " << ball << endl;
    cout << "BallVel: " << ballVel << endl;
    cout << "ReqVel: " << reqVel << endl;
    cout << "Target: " << target << endl;
    cout << "MinTimeToKick: " << minTimeToKick << endl;
    cout << "TimeUntilImpact: " << timeUntilImpact << endl;
    throw 
      BConfigException(
        this, 
        "Invalid behavior configuration parameters passed.", 
        false, 
        EXC_INVALID_BCONFIG_PARAMETERS
      );
  }
}

bool JSE2DImpKickConfig::assignFromJson(const Json::Value& obj)
{
  if (!JSKickConfig::assignFromJson(obj))
    return false;
  try {
    ballVel.x = obj["ballVel"][0].asFloat();
    ballVel.y = obj["ballVel"][1].asFloat();
    timeUntilImpact = obj.get("timeUntilImpact", 0.f).asFloat();
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught in behavior config \n\tType: "
      << static_cast<unsigned>(baseType)
      << ", Id: " 
      << DataUtils::varToString(id) 
      << ";\t";
    LOG_EXCEPTION(e.what());
    return false;
  }
  return true;
}

Json::Value JSE2DImpKickConfig::getJson() 
{ 
  Json::Value obj = JSKickConfig::getJson();
  try {
    obj["ballVel"].append(ballVel.x);
    obj["ballVel"].append(ballVel.y);
    obj["timeUntilImpact"] = timeUntilImpact;
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught in behavior config \n\tType: "
      << static_cast<unsigned>(baseType)
      << ", Id: " 
      << DataUtils::varToString(id) 
      << ";\t";
    LOG_EXCEPTION(e.what());
  }
  return obj;
}

void CSpaceBSplineKickConfig::validate() {}
