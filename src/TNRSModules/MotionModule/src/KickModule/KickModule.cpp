/**
 * @file MotionModule/src/KickModule/KickModule.cpp
 *
 * This file implements the class KickModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 15 May 2017
 */

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KickModule/KickModule.h"
#include "MotionModule/include/KickModule/Types/JSOImpKick.h"
#include "MotionModule/include/KickModule/Types/JSE2DImpKick.h"
#include "MotionModule/include/KickModule/Types/CSpaceBSplineKick.h"
#include "MotionModule/include/KickModule/KickFootMap.h"
#include "MotionModule/include/BalanceModule/BalanceDefinitions.h"
#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBKickConfig.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/Constants.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/JsonLogger.h"
#include "Utils/include/Splines/BSpline.h"

using namespace Constants;

template <typename Scalar>
Scalar KickModule<Scalar>::ballMass;
template <typename Scalar>
Scalar KickModule<Scalar>::ballRadius;
template <typename Scalar>
Scalar KickModule<Scalar>::sf;
template <typename Scalar>
Scalar KickModule<Scalar>::rf;
template <typename Scalar>
BSpline<Scalar>* KickModule<Scalar>::lFootContour;
template <typename Scalar>
BSpline<Scalar>* KickModule<Scalar>::rFootContour;
template <typename Scalar>
vector<Matrix<Scalar, 3, 1>> KickModule<Scalar>::footRect;

template <typename Scalar>
KickModule<Scalar>::KickModule(
  MotionModule* motionModule,
  const boost::shared_ptr<MBKickConfig>& config,
  const string& name) :
  MotionBehavior<Scalar>(motionModule, config, name)
{
  kickLeg = LinkChains::count;
  supportLeg = LinkChains::count;
  endEffector.setIdentity();
  supportToKick.setIdentity();
  torsoToSupport.setIdentity();
  kickTaskJoints = vector<bool>(toUType(Joints::count), false);
}

template <typename Scalar>
boost::shared_ptr<KickModule<Scalar> > KickModule<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg)
{
  KickModule<Scalar>* km;
  switch (cfg->type) {
    case toUType(MBKickTypes::jsoImpKick):
      km = new JSOImpKick<Scalar>(motionModule, SPC(JSOImpKickConfig, cfg)); break;
    case toUType(MBKickTypes::jse2DImpKick):
      km = new JSE2DImpKick<Scalar>(motionModule, SPC(JSE2DImpKickConfig, cfg)); break;
    case toUType(MBKickTypes::cSpaceBSplineKick):
      km = new CSpaceBSplineKick<Scalar>(motionModule, SPC(CSpaceBSplineKickConfig, cfg)); break;
  }
  return boost::shared_ptr<KickModule<Scalar> >(km);
}

template <typename Scalar>
void KickModule<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG("EnvProperties",
      (Scalar, ballRadius, ballRadius),
      (Scalar, ballMass, ballMass),
      (Scalar, coeffSF, sf),
      (Scalar, coeffRF, rf),
      (Scalar, coeffDamping, coeffDamping),
    )
    lFootContour =
      new BSpline<Scalar>(
        ConfigManager::getCommonConfigDirPath() + "left_foot_contour.xml");
    rFootContour =
      new BSpline<Scalar>(
        ConfigManager::getCommonConfigDirPath() + "right_foot_contour.xml");
    Matrix<Scalar, 3, 1> tl, tr, bl ,br;
    tl(0, 0) = footSizeX / 2 + footOriginShiftX;
    tl(1, 0) = footSizeY / 2 - footOriginShiftY;
    tl(2, 0) = -footHeight;
    tr(0, 0) = footSizeX / 2 + footOriginShiftX;
    tr(1, 0) = -footSizeY / 2 - footOriginShiftY;
    tr(2, 0) = -footHeight;
    bl(0, 0) = -footSizeX / 2 + footOriginShiftX;
    bl(1, 0) = footSizeY / 2 - footOriginShiftY;
    bl(2, 0) = -footHeight;
    br(0, 0) = -footSizeX / 2 + footOriginShiftX;
    br(1, 0) = -footSizeY / 2 - footOriginShiftY;
    br(2, 0) = -footHeight;
    footRect.push_back(tl);
    footRect.push_back(tr);
    footRect.push_back(bl);
    footRect.push_back(br);
    logFootContours();
    loaded = true;
  }
}

template <typename Scalar>
MBKickConfigPtr KickModule<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <MBKickConfig> (this->config);
}

template <typename Scalar>
bool KickModule<Scalar>::setKickSupportLegs()
{
  kickLeg = LinkChains::rLeg;
  supportLeg = kickLeg == LinkChains::rLeg ? LinkChains::lLeg : LinkChains::rLeg;
  return true;
  if (ballPosition[0] > 0.180)
    return false;
  auto angle = targetAngle * 180.0 / M_PI;
  for (size_t i = 0; i < 14; ++i) {
    if (
        ballPosition[1] >= kickFootMap[i][1] &&
        ballPosition[1] < kickFootMap[i][2] &&
        angle >= kickFootMap[i][3] &&
        angle <= kickFootMap[i][4])
    {
      kickLeg = static_cast<LinkChains>(kickFootMap[i][0]);
      supportLeg = kickLeg == LinkChains::rLeg ? LinkChains::lLeg : LinkChains::rLeg;
      return true;
    }
  }
  return false;
}

template <typename Scalar>
bool KickModule<Scalar>::setTransformFrames(const JointStateType& type)
{
  try {
    if (kickLeg != supportLeg &&
        (kickLeg == LinkChains::lLeg ||
        kickLeg == LinkChains::rLeg) &&
        (supportLeg == LinkChains::lLeg ||
        supportLeg == LinkChains::rLeg))
    {
      torsoToSupport = this->kM->getForwardEffector(supportLeg, toUType(LegEEs::footCenter), type);
      supportToTorso =
        MathsUtils::getTInverse(torsoToSupport);
      supportToKick =
        supportToTorso *
        this->kM->getForwardEffector(kickLeg, toUType(LegEEs::ankle), type);
      return true;
    } else {
      throw BehaviorException(
        this,
        "Cannot set transformations for undefined kick and support legs.",
        false
      );
    }
  } catch (BehaviorException& e) {
    LOG_EXCEPTION(e.what());
    return false;
  }
}

template <typename Scalar>
bool KickModule<Scalar>::setEndEffectorXY(const Scalar& angle)
{
  try {
    if (kickLeg != LinkChains::lLeg && kickLeg != LinkChains::rLeg){
      throw BehaviorException(
          this,
          "No kick leg defined to find the end-effector.",
          false
        );
    }
  } catch (BehaviorException& e) {
    LOG_EXCEPTION(e.what());
    return false;
  }

  bool success;
  Matrix<Scalar, 3, 1> contourPoint;
  auto normal = Matrix<Scalar, 3, 1>(cos(angle), sin(angle), 0.f);
  Matrix<Scalar, 2, 1> tBounds;
  if (angle >= 0) {
    if (kickLeg == LinkChains::lLeg) {
      if (angle >= M_PI / 4)
        tBounds = Matrix<Scalar, 2, 1>(0.0, 0.3);
      else
        tBounds = Matrix<Scalar, 2, 1>(0.0, 0.625);
    } else {
      if (angle >= M_PI / 4)
        tBounds = Matrix<Scalar, 2, 1>(0.7, 1.0);
      else
        tBounds = Matrix<Scalar, 2, 1>(0.375, 1.0);
    }
  } else {
    if (kickLeg == LinkChains::lLeg) {
      if (angle <= M_PI / 4)
        tBounds = Matrix<Scalar, 2, 1>(0.7, 1.0);
      else
        tBounds = Matrix<Scalar, 2, 1>(0.375, 1.0);
    } else {
      if (angle <= M_PI / 4)
        tBounds = Matrix<Scalar, 2, 1>(0.0, 0.625);
      else
        tBounds = Matrix<Scalar, 2, 1>(0.0, 0.3);
    }
  }
  if (kickLeg == LinkChains::lLeg) {
    success = lFootContour->findNormalToVec(normal, contourPoint, tBounds);
    if (!success)
      return false;
    endEffector(0, 3) = contourPoint[0];
    endEffector(1, 3) = contourPoint[1];
    endEffector(2, 3) = contourPoint[2];
    this->kM->setEndEffector(
      kickLeg, toUType(LegEEs::kickEE), endEffector.block(0, 3, 4, 1));
    return true;
  } else if (kickLeg == LinkChains::rLeg) {
    success = rFootContour->findNormalToVec(normal, contourPoint, tBounds);
    if (!success)
      return false;
    endEffector(0, 3) = contourPoint[0];
    endEffector(1, 3) = contourPoint[1];
    endEffector(2, 3) = contourPoint[2];
    this->kM->setEndEffector(
      kickLeg, toUType(LegEEs::kickEE), endEffector.block(0, 3, 4, 1));
    return true;
  }
}

template <typename Scalar>
void KickModule<Scalar>::setEndEffectorZX(const Scalar& t)
{
  /*Matrix4f bezierMat;
  Matrix<Scalar, 4, 3> contourMat;
  bezierMat << 1, 0, 0, 0, -3, 3, 0, 0, 3, -6, 3, 0, -1, 3, -3, 1;
  Vector4f tVector;
  Matrix<Scalar, 3, 1> contourPoint;
  fstream footCurveLog;
  footCurveLog.open(
  (ConfigManager::getLogsDirPath() + string("KickModule/FootCurveZX.txt")).c_str(),
    std::ofstream::out | std::ofstream::trunc);
  footCurveLog << "# t     X     Y    Z" << endl;
  footCurveLog.close();
  contourMat << zxFootContour[0][0], zxFootContour[0][1], zxFootContour[0][2], zxFootContour[1][0], zxFootContour[1][1], zxFootContour[1][2], zxFootContour[2][0], zxFootContour[2][1], zxFootContour[2][2], zxFootContour[3][0], zxFootContour[3][1], zxFootContour[3][2];
  contourMat = bezierMat * contourMat;
  tVector << 1, t, pow(t, 2), pow(t, 3);
  contourPoint = (tVector.transpose() * contourMat).transpose();
  endEffector(0, 3) = contourPoint[0];
  endEffector(2, 3) = contourPoint[2];
  footCurveLog.open(
  (ConfigManager::getConfigDirPath() + string("KickModule/FootCurveZX.txt")).c_str(),
    std::ofstream::out | std::ofstream::trunc);
  for (Scalar ti = 0.0; ti <= 1.0; ti = ti + 0.01) {
    tVector(0, 0) = 1;
    tVector(1, 0) = ti;
    tVector(2, 0) = pow(ti, 2);
    tVector(3, 0) = pow(ti, 3);
    Matrix<Scalar, 3, 1> point = (tVector.transpose() * contourMat).transpose();
    footCurveLog << ti << "    " << point[0] << "    " << point[1] << "    " << point[2] << endl;
  }
  footCurveLog.close();*/
}

template <typename Scalar>
bool KickModule<Scalar>::checkFootCollision(
    const Matrix<Scalar, Dynamic, 1>& kickAngles)
{
  unsigned chainStart = this->kM->getLinkChain(kickLeg)->start;
  this->kM->setJointPositions(
    static_cast<Joints>(chainStart), kickAngles, JointStateType::sim); // impact pose joints
  Matrix<Scalar, 4, 4> supportToKickTemp = supportToTorso * this->kM->getForwardEffector(kickLeg, toUType(LegEEs::ankle), JointStateType::sim);
  vector<Matrix<Scalar, 3, 1>> footRectT;
  for (size_t i = 0; i < footRect.size(); ++i) {
    footRectT[i] = MathsUtils::transformVector(supportToKickTemp, footRect[i]);
  }
  if (kickLeg == LinkChains::rLeg) {
    Matrix<Scalar, 4, 1> kickFootRect;
    kickFootRect[0] = (footRectT[0][0] + footRectT[2][0])/2;
    kickFootRect[1] = (footRectT[0][1] + footRectT[1][1])/2;
    kickFootRect[2] = fabsf(footRectT[0][0] - footRectT[2][0]);
    kickFootRect[3] = fabsf(footRectT[0][1] - footRectT[1][1]);
    Matrix<Scalar, 4, 1> supportFootRect;
    supportFootRect[0] = footOriginShiftX;
    supportFootRect[1] = footOriginShiftY;
    supportFootRect[2] = footSizeX;
    supportFootRect[3] = footSizeY + 0.01; // 1 cm increase in size acts as collision tollerance
    return (abs(kickFootRect[0] - supportFootRect[0]) <= (kickFootRect[2] + supportFootRect[2]) / 2) &&
           (abs(kickFootRect[1] - supportFootRect[1]) <= (kickFootRect[3] + supportFootRect[3]) / 2);
  } else {
    Matrix<Scalar, 4, 1> kickFootRect;
    kickFootRect[0] = (footRectT[0][0] + footRectT[1][0])/2;
    kickFootRect[1] = (footRectT[0][1] + footRectT[2][1])/2;
    kickFootRect[2] = abs(footRectT[0][0] - footRectT[1][0]);
    kickFootRect[3] = abs(footRectT[0][1] - footRectT[2][1]);
    Matrix<Scalar, 4, 1> supportFootRect;
    supportFootRect[0] = footOriginShiftX;
    supportFootRect[1] = -footOriginShiftY;
    supportFootRect[2] = footSizeX;
    supportFootRect[3] = footSizeY + 0.01; // 1 cm increase in size acts as collision tollerance
    return (abs(kickFootRect[0] - supportFootRect[0]) <= (kickFootRect[2] + supportFootRect[2]) / 2) &&
           (abs(kickFootRect[1] - supportFootRect[1]) <= (kickFootRect[3] + supportFootRect[3]) / 2);
  }
  return false;
}


template <typename Scalar>
void KickModule<Scalar>::setupPosture()
{
  LOG_INFO("KickModule.setupPosture()")
  if (this->getBehaviorCast()->postureConfig) {
    this->setupChildRequest(this->getBehaviorCast()->postureConfig);
  } else {
    auto postureConfig =
      boost::shared_ptr<InterpToPostureConfig>(new InterpToPostureConfig());
    postureConfig->targetPosture = PostureState::stand;
    postureConfig->timeToReachP = 1.0;
    this->setupChildRequest(postureConfig);
  }
}

template <typename Scalar>
void KickModule<Scalar>::setupBalance()
{
  //LOG_INFO("KickModule.setupBalance()")
  if (this->getBehaviorCast()->balanceConfig) {
    this->getBehaviorCast()->balanceConfig->supportLeg = this->supportLeg;
    this->setupChildRequest(this->getBehaviorCast()->balanceConfig, true);
  } else {
    LOG_INFO("JointSpaceKiflBottomck.setupBalance(): No balance config found.")
  }
}

template <typename Scalar>
void KickModule<Scalar>::setDesBallVel()
{
  //desBallVel = sqrt(targetDistance / (0.026 / rf + 0.02496 / sf));
  //! Solving ball distance equation which is made up of two parts;
  //! Distance covered by the ball until the ball starts to roll
  //! And the distance covered after rolling under damped motion
  //! targetDistance = posI + velI / damping * (1 - exp(-damping * time));
  //! posI is set to zero as we are finding distance relative to ball
  //! The quadratic equation for the ball distance becomes;
  Scalar vRatio = 5.0 / 7.0;
  Scalar a = (vRatio * vRatio - 1.0) / (-2.0 * this->sf * Constants::gravity);
  Scalar b = vRatio * (1.0 / this->coeffDamping);
  Scalar c = -targetDistance;
  Scalar sol1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
  if (sol1 < 0) {
    Scalar sol2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
    desBallVel = sol2;
  } else {
    desBallVel = sol1;
  }
}

template <typename Scalar>
void KickModule<Scalar>::computeDesImpactVel(const Matrix<Scalar, Dynamic, 1>& impJoints)
{
  Scalar coeffRest = 1.f; // Full elastic collision
  this->setDesBallVel();
  Matrix<Scalar, 3, 1> direction =
  this->torsoToSupport.block(0, 0, 3, 3) * this->ballToTargetUnit;
  //cout << "desBallVel: " << desBallVel << endl;
  // impact pose joints
  this->kM->setChainPositions(this->kickLeg, impJoints, JointStateType::sim);
  Scalar vm;
  this->kM->computeVirtualMass(this->kickLeg, direction, this->endEffector, vm, JointStateType::sim);
  //cout << "Virtual mass in given direction and found pose: " << vm << endl;
  //cout << "coeffRest: " << coeffRest << endl;
  //cout << "this->ballMass: " << this->ballMass << endl;
  Scalar desEndEffVel =
  this->desBallVel * (vm + this->ballMass) / ((1 + coeffRest) * vm);

  desImpactVel[0] = desEndEffVel * this->ballToTargetUnit[0];
  desImpactVel[1] = desEndEffVel * this->ballToTargetUnit[1];
  desImpactVel[2] = 0.f;

  //cout << "desImpactVel:" << desImpactVel << endl;
  //cout << "this->ballToTargetUnit: " << this->ballToTargetUnit << endl;

  Matrix<Scalar, Dynamic, 1> balanceJoints(toUType(Joints::count));
  if (this->supportLeg == LinkChains::lLeg) {
    balanceJoints = Matrix<Scalar, Dynamic, 1>::Map(
      &balanceDefs[0][0],
      sizeof(balanceDefs[0]) / sizeof(balanceDefs[0][0]));
  } else {
    balanceJoints = Matrix<Scalar, Dynamic, 1>::Map(
      &balanceDefs[1][0],
      sizeof(balanceDefs[1]) / sizeof(balanceDefs[1][0]));
  }
  this->kM->setJointPositions(Joints::first, balanceJoints, JointStateType::sim);

  if (this->config->logData) {
    Json::Value jsonImpact;
    JSON_ASSIGN(jsonImpact, "desBallVel", this->desBallVel);
    JSON_ASSIGN(jsonImpact, "ballMass", this->ballMass);
    JSON_ASSIGN(jsonImpact, "coeffRest", coeffRest);
    JSON_ASSIGN(jsonImpact, "virtualMass", vm);
    JSON_ASSIGN(jsonImpact, "desEndEffVel", desEndEffVel);
    JSON_ASSIGN(jsonImpact, "desImpactVel", JsonUtils::matrixToJson(desImpactVel));
    JSON_ASSIGN(jsonImpact, "impJoints", JsonUtils::matrixToJson(impJoints * 180 / M_PI));
    JSON_ASSIGN(jsonImpact, "endEffector", JsonUtils::matrixToJson(this->endEffector));
    JSON_ASSIGN(this->dataLogger->getRoot(), "impact", jsonImpact);
  }
}

template <typename Scalar>
void KickModule<Scalar>::logFootContours()
{
  string pathToLogs = ConfigManager::getLogsDirPath() + string("KickModule/");
  string logLPath = pathToLogs + "FootCurveLeftXY.txt";
  string logRPath = pathToLogs + "FootCurveRightXY.txt";
  auto spline = lFootContour->getSpline(0);
  fstream logL, logR;
  logL.open(logLPath, std::fstream::out | std::fstream::trunc);
  for (int i = 0; i < spline.rows(); ++i) {
      logL << spline(i, 0) << " " << spline(i, 1) + 0.05 << " " << spline(i, 2) + footHeight << endl;
  }

  spline = rFootContour->getSpline(0);
  logR.open(logRPath, std::fstream::out | std::fstream::trunc);
  for (int i = 0; i < spline.rows(); ++i) {
      logR << spline(i, 0) << " " << spline(i, 1) - 0.05 << " " << spline(i, 2) + footHeight  << endl;
  }
  logL.close();
  logR.close();
}

template class KickModule<MType>;

/*
void
KickModule<Scalar>::plotFootSurfaces()
{
  string logsPath = ConfigManager::getLogsDirPath() + string("KickModule/");
  string lLogPath = logsPath + string("FootCurveLeftXY.txt");
  string rLogPath = logsPath + string("FootCurveRightXY.txt");
  string zxLLogPath = logsPath + string("FootCurveLeftZX.txt");
  string zxRLogPath = logsPath + string("FootCurveRightZX.txt");
  fstream lLog, rLog, zxLLog, zxRLog, surfacePoints;
  lLog.open(lLogPath, std::fstream::in);
  rLog.open(rLogPath, std::fstream::in);
  zxLLog.open(zxLLogPath, std::fstream::in);
  zxRLog.open(zxRLogPath, std::fstream::in);
  if (!rLog || !lLog || !zxLLog || !zxRLog) throw("Foot contour log files not found.\n");

  Mat lFoot, rF
template class HeadTargetTrack<MType>;oot;
  makeFootSurfaces3D(true, lLog, zxLLog, lFoot);
  makeFootSurfaces3D(false, rLog, zxRLog, rFoot);

  surfacePoints.open(
    logsPath + "surfacePointsLeft.txt",
    fstream::out | fstream::trunc);
  surfacePoints << "# X Y Z" << endl;
  for (size_t r = 0; r < lFoot.rows; ++r) {
    for (size_t c = 0; c < lFoot.cols; ++c) {
      surfacePoints << " " << lFoot.at < Vec3f > (r, c)[0] << " " << lFoot.at < Vec3f > (r, c)[1] << " " << lFoot.at < Vec3f > (r, c)[2] << endl;
    }
    surfacePoints << endl;
  }
  surfacePoints.close();

  surfacePoints.open(
    logsPath + "surfacePointsRight.txt",
    fstream::out | fstream::trunc);
  surfacePoints << "# X Y Z" << endl;

  for (size_t r = 0; r < rFoot.rows; ++r) {
    for (size_t c = 0; c < rFoot.cols; ++c) {
      surfacePoints << " " << rFoot.at < Vec3f > (r, c)[0] << " " << rFoot.at < Vec3f > (r, c)[1] << " " << rFoot.at < Vec3f > (r, c)[2] << endl;
    }
    surfacePoints << endl;
  }
  surfacePoints.close();
  gp << "set xlabel 'y-Axis'\n";
  gp << "set ylabel 'x-Axis'\n";
  gp << "set zlabel 'z-Axis'\n";
  Matrix<Scalar, 3, 1> framePos(0.f, 0.f, leftFootContour[0][2]);
  Matrix<Scalar, 3, 1> frameRot(0.f, 0.f, M_PI / 2);
  KickUtils::drawFrame3D(gp, framePos, frameRot); // taking height
  gp << "splot '" << logsPath + "surfacePointsLeft.txt" << "' using 1:2:3 with lines lw 1 lc 3, '" << logsPath + "surfacePointsRight.txt" << "' using 1:2:3 with lines lw 1 lc 3\n";

}
*//*
void
KickModule<Scalar>::makeFootSurfaces3D(const bool& leftFoot, fstream& log,
  fstream& zxLog, Mat& surfaceMat)
{
  unsigned rows = KickUtils::countLines(log) - 1;
  unsigned cols = KickUtils::countLines(zxLog) - 1;
  surfaceMat = Mat(Size(cols, rows), CV_32FC3);
  string line;
  unsigned r = 0;
  while (getline(log, line)) {
    if (line.find('#') != line.npos) continue;
    vector < string > parts;
    split(parts, line, boost::is_any_of(" "));
    if (parts.size() >= 7) {
      Scalar transX, transY;
      DataUtils::stringToVar(parts[1], transX);
      DataUtils::stringToVar(parts[2], transY);
      Scalar tangentX, tangentY;
      DataUtils::stringToVar(parts[4], tangentX);
      DataUtils::stringToVar(parts[5], tangentY);
      Scalar angle = atan2(-tangentX, tangentY);
      if (!leftFoot) angle += M_PI;
      Matrix4f trans = KickUtils::rotZ(angle);
      trans(0, 3) = transX;
      trans(1, 3) = transY;
      string lineZX;
      unsigned ln2 = 0;
      Scalar startX, startY;
      unsigned c = 0;
      while (getline(zxLog, lineZX)) {
        ++ln2;
        if (lineZX.find('#') != lineZX.npos) continue;
        vector < string > parts;
        split(parts, lineZX, boost::is_any_of(" "));
        if (parts.size() >= 4) {
          Vector4f zxVector;
          DataUtils::stringToVar(parts[1], zxVector[0]);
          DataUtils::stringToVar(parts[2], zxVector[1]);
          DataUtils::stringToVar(parts[3], zxVector[2]);
          if (ln2 == 2) {
            startX = zxVector[0];
            startY = zxVector[1];
          }
          zxVector[0] -= startX;
          zxVector[1] -= startY;
          zxVector[3] = 1;
          zxVector = trans * zxVector;
          surfaceMat.at < Vec3f > (r, c)[0] = -zxVector[1];
          surfaceMat.at < Vec3f > (r, c)[1] = zxVector[0];
          surfaceMat.at < Vec3f > (r, c)[2] = zxVector[2];
          c++;
        }
      }
      r++;
      zxLog.clear();
      zxLog.seekg(0, ios::beg);
    }
  }
  log.close();
  zxLog.close();
}
*/


