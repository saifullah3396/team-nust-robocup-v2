/**
 * @file Utils/src/PotentialField2D.cpp
 *
 * This file implements the class PotentialField2D
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include "Utils/include/PotentialField2D.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/VelocityInput.h"
#include "Utils/include/DataHolders/Obstacle.h"

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
  //! Attractive potential
  Matrix<Scalar, 3, 1> goalDiff = robotPose.get() - goalPose.get();
  Scalar goalDist = goalDiff.norm();
  Matrix<Scalar, 3, 1> fAtt;
  if (goalDist < distThresholdAtt)
    fAtt = -kAtt * goalDiff;
  else
    fAtt = -kAtt * goalDiff / goalDist;

  Matrix<Scalar, 2, 1> totalFRep;
  totalFRep.setZero();
  Scalar cTheta = cos(robotPose.getTheta());
  Scalar sTheta = sin(robotPose.getTheta());
  for (int i = 0; i < 2; ++i) {
    /*Obstacle obs = obstacles[i];
    Scalar worldLX = robotPose.getX() + obs.leftBound.x * cTheta - obs.leftBound.y * sTheta;
    Scalar worldLY = robotPose.getY() + obs.leftBound.x * sTheta + obs.leftBound.y * cTheta;
    Scalar worldRX = robotPose.getX() + obs.rightBound.x * cTheta - obs.rightBound.y * sTheta;
    Scalar worldRY = robotPose.getY() + obs.rightBound.x * sTheta + obs.rightBound.y * cTheta;
    Scalar midX = (worldLX + worldRX) / 2.0;
    Scalar midY = (worldLY + worldRY) / 2.0;
    Scalar obsDepth = obs.depth;
    Scalar obsSlope =
      (worldRY - worldLY) / (worldRX - worldLX);
    Scalar obsSlopeInv = -1 / obsSlope;
    Point2f tempPoint(
      worldLX + 0.1,
      0.1 * obsSlopeInv + worldLY);
    Point2f diff = tempPoint - obs.leftBound;
    Scalar mag = norm(diff);
    Point2f unit = Point2f(diff.x / mag, diff.y / mag);
    midX += unit.x * obsDepth;
    midY += unit.y * obsDepth;*/
    Matrix<Scalar, 2, 1> obsPose;
    obsPose[0] = 1.0;
    obsPose[1] = 0.1;
    if (i==1)
      obsPose[1] *= -1;
    Matrix<Scalar, 2, 1> obsDiff;
    obsDiff[0] = robotPose.getX() - obsPose[0];
    obsDiff[1] = robotPose.getY() - obsPose[1];
    Scalar obsDist = obsDiff.norm();
    Matrix<Scalar, 2, 1> fRepParallel, fRepPerp, minDistPosToGoal;
    if (obsDist < distThresholdRep) {
      Matrix<Scalar, 2, 1> unitDir = obsDiff / obsDist;
      //cout << "Within distance..." << endl;
      //cout << "obsDist: " << obsDist << endl;
      //cout << "distThresholdRep: " << distThresholdRep << endl;
      //cout << "unitDir: " << unitDir.transpose() << endl;
      fRepParallel = (1.0 / obsDist - 1.0 / distThresholdRep) * unitDir / (obsDist * obsDist);
      auto minDistPos = obsPose + unitDir * distThresholdRep;
      minDistPosToGoal[0] = goalPose.getX() - minDistPos[0];
      minDistPosToGoal[1] = goalPose.getY() - minDistPos[1];
      auto angleObsToMinDist = atan2(unitDir[1], unitDir[0]);
      auto angleMinDisToGoal = atan2(minDistPosToGoal[1], minDistPosToGoal[0]);
      auto angleDiff = angleObsToMinDist - angleMinDisToGoal;
     // if (angleDiff >= 0 && angleDiff < M_PI) {
     //   fRepPerp[0] = fRepParallel[1]; // -90 degrees
     //   fRepPerp[1] = -fRepParallel[0]; // -90 degrees
     // } else {
        fRepPerp[0] = -fRepParallel[1]; // 90 degrees
        fRepPerp[1] = fRepParallel[0]; // 90 degrees
     // }
    } else {
      fRepParallel.setZero();
      fRepPerp.setZero();
    }
    //cout << "fRepPerp: " << fRepPerp.transpose() << endl;
    //cout << "fRepParallel: " << fRepParallel.transpose() << endl;
    totalFRep += kRep * fRepParallel;
    totalFRep += fRepPerp * kRepPerp;
  }
  cout << "totalFRep: " << totalFRep.norm() << endl;
  fAtt[0] += totalFRep[0];
  fAtt[1] += totalFRep[1];
  //! Transform back to robot coordinates
  VelocityInput<Scalar> cmd;
  //cmd.mat[0] = fAtt[0] * cTheta + fAtt[1] * sTheta;
  //cmd.mat[1] = -fAtt[0] * sTheta + fAtt[1] * cTheta;
  //cmd.mat[2] = fAtt[2];
  cmd.set(fAtt);
  return cmd;
}

template class PotentialField2D<float>;
template class PotentialField2D<double>;
