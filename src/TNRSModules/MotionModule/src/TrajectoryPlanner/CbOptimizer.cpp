/**
 * @file MotionModule/TrajectoryPlanner/CbOptimizer.h
 *
 * This file implements the class to CbOptimizer
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */

#include "MotionModule/include/TrajectoryPlanner/CbOptimizer.h"
#include "MotionModule/include/TrajectoryPlanner/TrajectoryPlanner.h"
#include "Utils/include/PlotEnv.h"
#include <fstream>

using namespace GnuPlotEnv;

template<typename Scalar>
double CbOptimizer<Scalar>::costFunction(
  const vector<double>& knots, 
  vector<double>& grad,
  void *data)
{
  auto nKnots = cb->getNKnots();
  if (!grad.empty()) {
    for (int i = 0; i < nKnots; ++i)
      grad[i] = 1.0;
  }
  double f = 0;
  for (int i = 0; i < nKnots; ++i)
    f = f + knots[i];
  //cout << "f: " << f << endl;
  return f;
}

template<typename Scalar>
void CbOptimizer<Scalar>::ineqConstraints(unsigned nCons, double *result, unsigned nOptVars,
  const double* knots, double* grad, void* data)
{
  auto nKnots = cb->getNKnots();
  Matrix<Scalar, Dynamic, 1> times;
  times.resize(nKnots + 1);
  times.setZero();
  for (int i = 1; i < times.size(); ++i) {
    times[i] = times[i - 1] + knots[i - 1];
  }
  Matrix<Scalar, Dynamic, Dynamic> timesRep = times.replicate(1, this->chainSize);
  Matrix<Scalar, Dynamic, Dynamic> timesU = timesRep.block(1, 0, nKnots, this->chainSize);
  Matrix<Scalar, Dynamic, Dynamic> timesL = timesRep.block(0, 0, nKnots, this->chainSize);
  Matrix<Scalar, Dynamic, 1> knotsEigen;
  knotsEigen.resize(nKnots);
  for (int i = 0; i < nKnots; ++i)
    knotsEigen[i] = knots[i];
  cb->evaluateCoeffs(knotsEigen);

  auto coeffs = cb->getCoeffs();
  auto knotsRep = cb->getRepKnots();
  auto bAccelsL = cb->getBAccelsL();
  auto bAccelsU = cb->getBAccelsU();  
  
  Matrix<Scalar, Dynamic, Dynamic> epVels =
    (-3 * coeffs[0].cwiseProduct(knotsRep.cwiseProduct(knotsRep)) + coeffs[2] - coeffs[3]).cwiseAbs() - this->velLimits.replicate(
      nKnots,
      1);
  Map<Matrix<Scalar, 1, Dynamic> > epConstraints(epVels.data(), epVels.size());
  Matrix<Scalar, Dynamic, Dynamic> inflexionCheck =
    (bAccelsL.array().cwiseProduct(bAccelsU.array()) < 0).matrix().template cast<Scalar>();

  Matrix<Scalar, Dynamic, Dynamic> midTs = timesL + knotsRep.cwiseProduct(
    bAccelsL.cwiseQuotient(bAccelsL - bAccelsU));
  Matrix<Scalar, Dynamic, Dynamic> tDiffU = timesU - midTs;
  Matrix<Scalar, Dynamic, Dynamic> tDiffL = midTs - timesL;
  Matrix<Scalar, Dynamic, Dynamic> midVels =
    -3 * coeffs[0].cwiseProduct(tDiffU.cwiseProduct(tDiffU)) + 3 * coeffs[1].cwiseProduct(
      tDiffL.cwiseProduct(tDiffL)) + coeffs[2] - coeffs[3];
  midVels = inflexionCheck.cwiseProduct(
    midVels.cwiseAbs() - this->velLimits.replicate(nKnots, 1));
  Map<Matrix<Scalar, 1, Dynamic> > midpConstraints(midVels.data(), midVels.size());
  vector<Scalar> totalConstraints;
  for (size_t i = 0; i < epConstraints.size(); ++i)
    totalConstraints.push_back(epConstraints[i]);
  for (size_t i = 0; i < midpConstraints.size(); ++i) {
    if (MathsUtils::almostEqual(midpConstraints[i], (Scalar)0.0, (Scalar)1e-6)) {
      totalConstraints.push_back(-1);
    } else {
      totalConstraints.push_back(midpConstraints[i]);
    }
  }

  if (!constraints.empty()) {
    vector<Scalar> dynCons = computeDynCons(times, coeffs);
    totalConstraints.insert(totalConstraints.end(), dynCons.begin(), dynCons.end());
  }
  //cout << "cons size: " << totalConstraints.size() << endl;
  //cout << "cons: " << endl;
  for (size_t i = 0; i < nCons; ++i) {
    result[i] = totalConstraints[i];
    //if (result[i] >= 0)
    //  cout << result[i] << endl;
  }
}

template<typename Scalar>
vector<Scalar> CbOptimizer<Scalar>::computeDynCons(
  const Matrix<Scalar, Dynamic, 1>& times,
  const vector<Matrix<Scalar, Dynamic, Dynamic> >& coeffs)
{
  for (size_t i = 0; i < times.size()-1; ++i) {
    float diff = times[i+1] - times[i];
    unsigned fact = i*innerPointsPerKnot;
    for (size_t j = 0; j < innerPointsPerKnot; ++j) {
      float time = times[i] + diff * j / innerPointsPerKnot;
      unsigned idx = fact+j;
      timesSeqU[idx] = times[i+1] - time;
      timesSeqL[idx] = time - times[i];
    }
  }
  timesSeqU2 = timesSeqU.array().square().matrix();
  timesSeqU3 = (timesSeqU2.array() * timesSeqU.array()).matrix();
  timesSeqL2 = timesSeqL.array().square().matrix();
  timesSeqL3 = (timesSeqL2.array() * timesSeqL.array()).matrix();


  //cout << "knots : " << cb->getKnots().transpose() << endl;

  pos.setZero();
  vel.setZero();
  acc.setZero();
  vector<Scalar> cons;
  for (size_t i = 0; i < times.size()-1; ++i) {
    unsigned fact = i*innerPointsPerKnot;
    for (size_t j = 0; j < innerPointsPerKnot; ++j) {
      unsigned idx = fact+j;
      pos.block(idx, 0, 1, this->chainSize) =
      coeffs[0].block(i, 0, 1, this->chainSize) * timesSeqU3[idx] +
      coeffs[1].block(i, 0, 1, this->chainSize) * timesSeqL3[idx] +
      coeffs[2].block(i, 0, 1, this->chainSize) * timesSeqL[idx] +
      coeffs[3].block(i, 0, 1, this->chainSize) * timesSeqU[idx];
      vel.block(idx, 0, 1, this->chainSize) =
      3 * (-coeffs[0].block(i, 0, 1, this->chainSize) * timesSeqU2[idx] +
            coeffs[1].block(i, 0, 1, this->chainSize)  * timesSeqL2[idx]) +
      coeffs[2].block(i, 0, 1, this->chainSize) -
      coeffs[3].block(i, 0, 1, this->chainSize);
      acc.block(idx, 0, 1, this->chainSize) =
      6 * (coeffs[0].block(i, 0, 1, this->chainSize) * timesSeqU[idx] -
           coeffs[1].block(i, 0, 1, this->chainSize) * timesSeqL[idx]);
      this->kM->setChainState(
        this->chainIndex,
        pos.block(idx, 0, 1, this->chainSize).transpose(),
        vel.block(idx, 0, 1, this->chainSize).transpose(),
        acc.block(idx, 0, 1, this->chainSize).transpose(),
        JointStateType::sim
      );
      for (size_t k = 0; k < constraints.size(); ++k) {
        constraints[k]->update();
        vector<Scalar> c = constraints[k]->computeConstraintMatrix();
        cons.insert(cons.end(), c.begin(), c.end());
      }
    }
  }
  return cons;
}


template<typename Scalar>
void CbOptimizer<Scalar>::logConstraints(
  const unsigned& innerPointsPerKnot,
  const Scalar& startTime,
  const string& logsPath,
  const bool& reset)
{
  Matrix<Scalar, Dynamic, 1> knots = cb->getKnots();
  unsigned nInnerPoints = innerPointsPerKnot * knots.size();
  Matrix<Scalar, Dynamic, 1> timesSeqU, timesSeqU2, timesSeqU3, timesSeqL, timesSeqL2, timesSeqL3;
  Matrix<Scalar, Dynamic, Dynamic> pos, vel, acc;
  pos.resize(nInnerPoints, this->chainSize);
  vel.resize(nInnerPoints, this->chainSize);
  acc.resize(nInnerPoints, this->chainSize);
  timesSeqU.resize(nInnerPoints);
  timesSeqU2.resize(nInnerPoints);
  timesSeqU3.resize(nInnerPoints);
  timesSeqL.resize(nInnerPoints);
  timesSeqL2.resize(nInnerPoints);
  timesSeqL3.resize(nInnerPoints);
  Matrix<Scalar, Dynamic, 1> times;
  times.resize(knots.size() + 1);
  times.setZero();
  for (int i = 1; i < times.size(); ++i) {
    times[i] = times[i - 1] + knots[i - 1];
  }
  for (size_t i = 0; i < times.size()-1; ++i) {
    float diff = times[i+1] - times[i];
    unsigned fact = i*innerPointsPerKnot;
    for (size_t j = 0; j < innerPointsPerKnot; ++j) {
      float time = times[i] + diff * j / innerPointsPerKnot;
      unsigned idx = fact+j;
      timesSeqU[idx] = times[i+1] - time;
      timesSeqL[idx] = time - times[i];
    }
  }
  timesSeqU2 = timesSeqU.array().square().matrix();
  timesSeqU3 = (timesSeqU2.array() * timesSeqU.array()).matrix();
  timesSeqL2 = timesSeqL.array().square().matrix();
  timesSeqL3 = (timesSeqL2.array() * timesSeqL.array()).matrix();
  pos.setZero();
  vel.setZero();
  acc.setZero();

  fstream logVelocities, logTorques, logZmp;
  if (reset) {
    logVelocities.open(logsPath + "/joint-velocities.txt", fstream::out | fstream::trunc);
    logVelocities << "# Times Velocities" << endl;
    logVelocities.close();
    logTorques.open(logsPath + "/joint-torques.txt", fstream::out | fstream::trunc);
    logTorques << "# Times Torques" << endl;
    logTorques.close();
    logZmp.open(logsPath + "/zmps.txt", fstream::out | fstream::trunc);
    logZmp << "# Times Zmp" << endl;
    logZmp.close();
  }
  logVelocities.open(logsPath + "/joint-velocities.txt", fstream::out | fstream::app);
  logTorques.open(logsPath + "/joint-torques.txt", fstream::out | fstream::app);
  logZmp.open(logsPath + "/zmps.txt", fstream::out | fstream::app);
  auto coeffs = cb->getCoeffs();
  for (size_t i = 0; i < times.size()-1; ++i) {
    float diff = times[i+1] - times[i];
    unsigned fact = i*innerPointsPerKnot;
    for (size_t j = 0; j < innerPointsPerKnot; ++j) {
      float time = times[i] + diff * j / innerPointsPerKnot;
      unsigned idx = fact+j;
      pos.block(idx, 0, 1, this->chainSize) =
      coeffs[0].block(i, 0, 1, this->chainSize) * timesSeqU3[idx] +
      coeffs[1].block(i, 0, 1, this->chainSize) * timesSeqL3[idx] +
      coeffs[2].block(i, 0, 1, this->chainSize) * timesSeqL[idx] +
      coeffs[3].block(i, 0, 1, this->chainSize) * timesSeqU[idx];
      vel.block(idx, 0, 1, this->chainSize) =
      3 * (-coeffs[0].block(i, 0, 1, this->chainSize) * timesSeqU2[idx] +
            coeffs[1].block(i, 0, 1, this->chainSize)  * timesSeqL2[idx]) +
      coeffs[2].block(i, 0, 1, this->chainSize) -
      coeffs[3].block(i, 0, 1, this->chainSize);
      acc.block(idx, 0, 1, this->chainSize) =
      6 * (coeffs[0].block(i, 0, 1, this->chainSize) * timesSeqU[idx] -
           coeffs[1].block(i, 0, 1, this->chainSize) * timesSeqL[idx]);
      this->kM->setChainState(
        this->chainIndex,
        pos.block(idx, 0, 1, this->chainSize).transpose(),
        vel.block(idx, 0, 1, this->chainSize).transpose(),
        acc.block(idx, 0, 1, this->chainSize).transpose(),
        JointStateType::sim
      );
      logVelocities << startTime + time << " ";
      logTorques << startTime + time << " ";
      logZmp << startTime + time << " ";

      Matrix<Scalar, 1, Dynamic> jVels = vel.block(idx, 0, 1, this->chainSize);
      for (size_t k = 0; k < jVels.size(); ++k) {
        logVelocities << jVels[k] << " ";
      }
      logVelocities << endl;

      for (size_t k = 0; k < constraints.size(); ++k) {
        constraints[k]->update();
        Matrix<Scalar, Dynamic, 1> value = constraints[k]->getValue();
        auto name = constraints[k]->getName();
        if (name == "ZmpConstraint") {
          for (size_t m = 0; m < value.size(); ++m) {
            logZmp << value[m] << " ";
          }
          logZmp << endl;
        } else if(name == "TorqueConstraint") {
          for (size_t m = 0; m < this->chainSize; ++m) {
            logTorques << value[this->chainStart+m] << " ";
          }
          logTorques << endl;
        }
      }
    }
  }
  logVelocities.close();
  logTorques.close();
  logZmp.close();
}

template<typename Scalar>
void CbOptimizer<Scalar>::optDef()
{
  for (size_t i = 0; i < constraints.size(); ++i)
    constraints[i]->init();

  //!Objective function to minimize is sum_0^n{knots}
  //!Hessian for this objective function is zero matrix.
  //!Gradient for this function is matrix with each element equal to one.
  auto nKnots = cb->getNKnots();
  auto cpDiff = cb->getCpDiff();
  nlopt::opt opt(nlopt::LN_COBYLA, nKnots);
  Matrix<Scalar, Dynamic, 1> lbEigen = (cpDiff.cwiseQuotient(
    this->velLimits.replicate(nKnots, 1) * 0.25)).cwiseAbs().rowwise().maxCoeff();
  vector<double> lb, ub, knots0, constraintTols;
  for (int i = 0; i < nKnots; ++i) {
    lb.push_back(lbEigen[i]);
    ub.push_back(2.0);
    knots0.push_back(lb[i]);
  }

  // Vel upper and lower hence twice
  nInnerPoints = innerPointsPerKnot * nKnots;
  unsigned nCons = nKnots * this->velLimits.size() * 2;
  for (int i = 0; i < nCons; ++i) {
    constraintTols.push_back(1e-8);
  }
  for (size_t i = 0; i < constraints.size(); ++i) {
    nCons += nInnerPoints * constraints[i]->getNConstraints();
    auto tol = constraints[i]->getTolerance();
    for (size_t j = 0; i < nInnerPoints; ++i) {
      constraintTols.insert(constraintTols.end(), tol.begin(), tol.end());
    }
  }

  //! Used in finding trajectories fast
  pos.resize(nInnerPoints, this->chainSize);
  vel.resize(nInnerPoints, this->chainSize);
  acc.resize(nInnerPoints, this->chainSize);
  timesSeqU.resize(nInnerPoints);
  timesSeqU2.resize(nInnerPoints);
  timesSeqU3.resize(nInnerPoints);
  timesSeqL.resize(nInnerPoints);
  timesSeqL2.resize(nInnerPoints);
  timesSeqL3.resize(nInnerPoints);
  //opt.add_inequality_mconstraint(
//    CbOptimizer<Scalar>::ineqWrapper,
//    this,
//    constraintTols);
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_min_objective(CbOptimizer<Scalar>::objWrapper, this);
  opt.set_xtol_rel(1e-4);
  opt.set_maxeval(500);
  double minf;
  nlopt::result result = opt.optimize(knots0, minf);
  try {
    if (result < 0) {
      cout << "nlopt failed!" << endl;
    } else {
      for (int i = 0; i < nKnots; ++i) {
        knots0[i] = ceil(knots0[i] / this->stepSize) * this->stepSize;
      }
    }
  } catch (const exception& e) {
    LOG_EXCEPTION(e.what());
  }
  Matrix<Scalar, Dynamic, 1> knotsEigen;
  knotsEigen.resize(nKnots);
  for (size_t i = 0; i < nKnots; ++i)
    knotsEigen[i] = knots0[i];
  cb->evaluateCoeffs(knotsEigen);
  //cb->plotSpline(100, 0.0);
}

template class CbOptimizer<MType>;
