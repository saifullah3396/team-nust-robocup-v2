/**
 * @file Utils/src/PotentialField2D.cpp
 *
 * This file implements the class PotentialField2D
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/VelocityInput.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/PotentialField2D.h"

template <typename Scalar>
PotentialField2D<Scalar>::PotentialField2D(
  const Scalar& kAtt,
  const Scalar& kRep,
  const Scalar& kRepPerp,
  const Scalar& distThresholdAtt, // 1m range
  const Scalar& distThresholdRep) :
  kAtt(kAtt),
  kRep(kRep),
  kRepPerp(kRepPerp),
  distThresholdAtt(distThresholdAtt),
  distThresholdRep(distThresholdRep)
{
}

template <typename Scalar>
PotentialField2D<Scalar>::~PotentialField2D()
{
}

template <typename Scalar>
VelocityInput<Scalar> PotentialField2D<Scalar>::update(
  const RobotPose2D<Scalar>& robotPose,
  const RobotPose2D<Scalar>& goalPose,
  const vector<Obstacle<Scalar>>& obstacles)
{
  ///< Attractive potential
  Matrix<Scalar, 3, 1> goalDiff;
  goalDiff[0] = robotPose.getX() - goalPose.getX();
  goalDiff[1] = robotPose.getY() - goalPose.getY();
  goalDiff[2] = MathsUtils::diffAngle(robotPose.getTheta(), goalPose.getTheta());
  Scalar goalDist = goalDiff.norm();
  Matrix<Scalar, 3, 1> fAtt;
  if (goalDist < distThresholdAtt)
    fAtt = -kAtt * goalDiff;
  else
    fAtt = -kAtt * goalDiff / goalDist;

  Matrix<Scalar, 2, 1> totalFRep;
  totalFRep.setZero();
  for (const auto& obs : obstacles)
  {
    auto obsCenter = obs.centerT;
    Matrix<Scalar, 2, 1> obsDiff;
    obsDiff[0] = robotPose.getX() - obsCenter.x;
    obsDiff[1] = robotPose.getY() - obsCenter.y;
    Scalar obsDist = obsDiff.norm();
    Matrix<Scalar, 2, 1> fRepParallel, fRepPerp, minDistPosToGoal;
    if (obsDist < distThresholdRep) {
      Matrix<Scalar, 2, 1> unitDir = obsDiff / obsDist;
      //cout << "Within distance..." << endl;
      //cout << "obsDist: " << obsDist << endl;
      //cout << "distThresholdRep: " << distThresholdRep << endl;
      //cout << "unitDir: " << unitDir.transpose() << endl;
      fRepParallel = (1.0 / obsDist - 1.0 / distThresholdRep) * unitDir / (obsDist * obsDist);
      Matrix<Scalar, 2, 1> minDistPos;
      minDistPos[0] = obsCenter.x + unitDir[0] * distThresholdRep;
      minDistPos[1] = obsCenter.y + unitDir[1] * distThresholdRep;
      minDistPosToGoal[0] = goalPose.getX() - minDistPos[0];
      minDistPosToGoal[1] = goalPose.getY() - minDistPos[1];
      auto angleObsToMinDist = atan2(unitDir[1], unitDir[0]);
      auto angleMinDisToGoal = atan2(minDistPosToGoal[1], minDistPosToGoal[0]);
      auto angleDiff = angleObsToMinDist - angleMinDisToGoal;
      if (angleDiff >= 0 && angleDiff < M_PI) {
        fRepPerp[0] = fRepParallel[1]; // -90 degrees
        fRepPerp[1] = -fRepParallel[0]; // -90 degrees
      } else {
        fRepPerp[0] = -fRepParallel[1]; // 90 degrees
        fRepPerp[1] = fRepParallel[0]; // 90 degrees
      }
    } else {
      fRepParallel.setZero();
      fRepPerp.setZero();
    }
    //cout << "fRepPerp: " << fRepPerp.transpose() << endl;
    //cout << "fRepParallel: " << fRepParallel.transpose() << endl;
    totalFRep += kRep * fRepParallel;
    totalFRep += fRepPerp * kRepPerp;
  }
  //cout << "totalFRep: " << totalFRep.norm() << endl;
  fAtt[0] += totalFRep[0];
  fAtt[1] += totalFRep[1];
  ///< Transform back to robot coordinates
  VelocityInput<Scalar> cmd;
  const Scalar& cTheta = robotPose.getCTheta();
  const Scalar& sTheta = robotPose.getSTheta();
  cmd.x() = fAtt[0] * cTheta + fAtt[1] * sTheta;
  cmd.y() = -fAtt[0] * sTheta + fAtt[1] * cTheta;
  cmd.theta() = fAtt[2];
  return cmd;
}

template class PotentialField2D<float>;
template class PotentialField2D<double>;
