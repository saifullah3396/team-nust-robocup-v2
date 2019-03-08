/**
 * @file Utils/src/Solvers/ImuCalibrator.cpp
 *
 * This file implements the class ImuCalibrator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#include "Utils/include/Constants.h"
#include "Utils/include/JsonUtils.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/Solvers/ImuCalibrator.h"

template <int StateSize>
ImuCalibrator<StateSize>::ImuCalibrator(const string& measFilePath)
{
  using namespace std;
  Json::Value json;
  ifstream data(measFilePath, ifstream::binary);
  data >> json;
  unsigned size = json["size"].asUInt();
  inputAcc.resize(size, 3);
  inputAngles.resize(size, 2);
  for (size_t i = 0; i < size; ++i) {
    inputAcc(i, 0) = (double)json["accelX"][(int)i].asFloat();
    inputAcc(i, 1) = (double)json["accelY"][(int)i].asFloat();
    inputAcc(i, 2) = (double)json["accelZ"][(int)i].asFloat();
    inputAngles(i, 0) = (double)json["angleX"][(int)i].asFloat();
    inputAngles(i, 1) = (double)json["angleY"][(int)i].asFloat();
  }
}

template <int StateSize>
double ImuCalibrator<StateSize>::costFunction(
  const vector<double>& vars, vector<double>& grad, void *data)
{
  static auto gravity = Matrix<double, 3, 1>(0.0, 0.0, -Constants::gravity);
  scale(0, 0) = vars[0];
  scale(1, 1) = vars[1];
  scale(2, 2) = vars[2];
  bias[0] = vars[3];
  bias[1] = vars[4];
  bias[2] = vars[5];
  Matrix<double, 3, 3> rot;
  double f = 0;
  for (size_t i = 0; i < inputAcc.rows(); ++i) {
    MathsUtils::makeRotationXYZ(
      rot,
      inputAngles(i, 0),
      inputAngles(i, 1),
      0.0);
    Matrix<double, 3, 1> accRot = rot * scale * (inputAcc.block(i, 0, 1, 3).transpose() - bias);
    f += (accRot - gravity).norm();
  }
  return f;
}

template <int StateSize>
void ImuCalibrator<StateSize>::optDef()
{
  //!Objective function to minimize the difference between the measured
  //!acceleration and target acceleration
  //!Hessian for this objective function is unknown.
  //!Gradient for this function is unknown.
  //!2 x StateSize  variables; bias x StateSize and scale x StateSize are to be found
  nlopt::opt opt(nlopt::LN_COBYLA, StateSize * 2);
  vector<double> lb(StateSize * 2), ub(StateSize * 2), var0;
  for (int i = 0; i < lb.size(); ++i) {
    lb[i]= -10.0;
    ub[i]= 10.0;
    var0.push_back(0.0);
  }
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_min_objective(ImuCalibrator::objWrapper, this);
  opt.set_xtol_rel(1e-6);
  opt.set_maxeval(1e9);
  double minf;
  nlopt::result result = opt.optimize(var0, minf);
  if (result < 0) {
    cout << "nlopt failed!" << endl;
  } else {
    scale(0, 0) = var0[0];
    scale(1, 1) = var0[1];
    scale(2, 2) = var0[2];
    bias[0] = var0[3];
    bias[1] = var0[4];
    bias[2] = var0[5];
    success = true;
  }
}

template <int StateSize>
void ImuCalibrator<StateSize>::printResults()
{
  cout << "Bias Matrix:\n" << bias << endl;
  cout << "Scale Matrix:\n" << scale << endl;
  double f = 0;
  static auto gravity = Matrix<double, 3, 1>(0.0, 0.0, -Constants::gravity);
  Matrix<double, 3, 3> rot;
  for (size_t i = 0; i < inputAcc.rows(); ++i) {
    MathsUtils::makeRotationXYZ(
      rot,
      inputAngles(i, 0),
      inputAngles(i, 1),
      0.0);
    Matrix<double, 3, 1> accRot = rot * scale * (inputAcc.block(i, 0, 1, 3).transpose() - bias);
    f += (accRot - gravity).norm();
  }
  cout << "Mean error: " << f / inputAcc.rows() << endl;
}

template class ImuCalibrator<3>;
