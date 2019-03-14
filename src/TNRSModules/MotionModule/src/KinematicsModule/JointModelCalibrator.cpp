/**
 * @file Utils/src/Solvers/JointModelCalibrator.cpp
 *
 * This file implements the class JointModelCalibrator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#include <boost/make_shared.hpp>
#include "MotionModule/include/KinematicsModule/JointEstimator.h"
#include "MotionModule/include/KinematicsModule/JointModelCalibrator.h"
#include "Utils/include/Constants.h"
#include "Utils/include/JsonUtils.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/PlotEnv.h"
#include "Utils/include/HardwareIds.h"

using namespace GnuPlotEnv;

JointModelCalibrator::JointModelCalibrator(const Joints& jointIndex, const string& measFilePath)
{
  using namespace std;
  Json::Value json;
  ifstream data(measFilePath, ifstream::binary);
  data >> json;
  auto state = json[Constants::jointNames[toUType(jointIndex)]]["state"];
  sensedPosition.resize(state.size());
  cmdPosition.resize(state.size());
  modelledPositions.resize(state.size());
  time.resize(state.size());
  for (size_t i = 0; i < state.size(); ++i) {
    cmdPosition[i] = state[static_cast<int>(i)]["cmd"].asFloat();
    sensedPosition[i] = state[static_cast<int>(i)]["sensed"].asFloat();
    modelledPositions[i] = state[static_cast<int>(i)]["position"].asFloat();
    time[i] = state[static_cast<int>(i)]["time"].asFloat();
  }
  Matrix<double, 2, 1> initState;
  initState << sensedPosition[0], 0.0;
  estimator = boost::make_shared<JointEstimator<double>>();
  estimator->init(jointIndex, initState, 0.01);
}

double JointModelCalibrator::costFunction(
  const vector<double>& vars, vector<double>& grad, void *data)
{
  double f = 0.0;
  pid[0] = vars[1];
  pid[1] = vars[2];
  pid[2] = vars[3];
  estimator->setPidGains(pid);
  Matrix<double, 2, 1> initState;
  initState << sensedPosition[vars[0] / 0.01], 0.0;
  estimator->reset(initState);
  for (size_t i = 0; i < cmdPosition.rows(); ++i) {
    auto shifted = vars[0] / 0.01 + i;
    if (shifted < cmdPosition.rows()) {
      estimator->setControl(cmdPosition[i]);
      estimator->updateModel();
      double position = estimator->getState()[0];
      f += fabsf(cmdPosition[i] - sensedPosition[shifted]) + fabsf(position - sensedPosition[shifted]);
    }
  }
  return f;
}


void JointModelCalibrator::optDef()
{
  //!Objective function to minimize the difference between the measured
  //!acceleration and target acceleration
  //!Hessian for this objective function is unknown.
  //!Gradient for this function is unknown.
  //!4 variables; 1 x time shift and pid gains x 3
  nlopt::opt opt(nlopt::LN_COBYLA, 4);
  vector<double> lb(4), ub(4), var0;
  var0.push_back(0.0);
  var0.push_back(0);
  var0.push_back(0);
  var0.push_back(0);
  lb[0] = 0.0;
  ub[0] = 0.1;
  lb[1] = 0.0;
  ub[1] = 500;
  lb[2] = 0.0;
  ub[2] = 100;
  lb[3] = 0.0;
  ub[3] = 20.0;
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_min_objective(JointModelCalibrator::objWrapper, this);
  opt.set_xtol_rel(1e-12);
  opt.set_maxeval(1e9);
  double minf;
  nlopt::result result = opt.optimize(var0, minf);
  if (result < 0) {
    cout << "nlopt failed!" << endl;
  } else {
    timeShift = var0[0];
    success = true;
  }
}


void JointModelCalibrator::printResults()
{
  Vector2d range;
  range[0] = -3.14;
  range[1] = 3.14;
  PlotEnv<double>::set_terminal_std("qt");
  PlotEnv<double> pe = PlotEnv<double>(
    "MyGraph",
    "x-Axis",
    "y-Axis",
    "z-Axis",
    range,
    range,
    range);
  double f = 0.0;
  cout << "Time shift:" << timeShift << endl;
  cout << "pid: " << pid.transpose() << endl;
  estimator->setPidGains(pid);
  Matrix<double, 2, 1> estState;
  Matrix<double, Dynamic, 1> estPositions;
  estState << sensedPosition[timeShift / 0.01], 0.0;
  estPositions.resize(cmdPosition.rows());
  estPositions[0] = estState[0];
  estimator->reset(estState);
  for (size_t i = 0; i < cmdPosition.rows(); ++i) {
    estimator->setControl(cmdPosition[i]);
    estimator->updateModel();
    double position = estimator->getState()[0];
    estPositions[i] = position;
    auto shifted = timeShift / 0.01 + i;
    if (shifted < cmdPosition.rows()) {
      f += fabsf(cmdPosition[i] - sensedPosition[shifted]) + fabsf(position - sensedPosition[shifted]);
    }
  }
  cout << "Average error: " << f / (cmdPosition.rows() - timeShift / 0.01) << endl;

  Matrix<double, Eigen::Dynamic, 1> sensedTime = time;
  for (int i = 0; i < time.rows(); ++i)
    sensedTime[i] -= timeShift;
  pe.plot2D("Commanded Position", time, cmdPosition);
  pe.plot2D("Sensed Position", sensedTime, sensedPosition);
  pe.plot2D("Estimated Position", time, estPositions);
  pe.plot2D("Modelled Position", time, modelledPositions);
  pe.showonscreen();
  while(true);
}
