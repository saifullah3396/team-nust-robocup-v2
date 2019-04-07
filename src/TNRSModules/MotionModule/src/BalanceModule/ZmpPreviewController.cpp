/**
 * @file MotionModule/src/BalanceModule/ZmpPreviewController.cpp
 *
 * This file implements the class ZmpPreviewController
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include <boost/make_shared.hpp>
#include "MotionModule/include/BalanceModule/ZmpPreviewController.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/Constants.h"
#include "Utils/include/Filters/ProcessModel.h"

template<typename Scalar>
ZmpPreviewController<Scalar>::ZmpPreviewController(
  boost::shared_ptr<ProcessModel<Scalar, 3, 1, 1> > model) :
  model(model),
  intError(0.0),
  trueIntError(0.0),
  firstState(true)
{
}

template<typename Scalar>
void ZmpPreviewController<Scalar>::initController()
{
  aug = boost::shared_ptr<AugmentedStateMats<3> >(new AugmentedStateMats<3>(model));
  gGain = aug->solveDare();
  auto matBAugt = aug->matBAug.transpose();
  Scalar temp = 1 / (aug->R + (matBAugt * gGain * aug->matBAug)(0, 0));
  kGain = temp * matBAugt * gGain * aug->matAAug;
  Matrix<Scalar, Dynamic, Dynamic> matPAAug = aug->matAAug - aug->matBAug * kGain;
  matPrevGain.resize(nPreviews, 1);
  matPrevGain[0] = -kGain(0, 0);
  Matrix<Scalar, Dynamic, Dynamic> prevState = -matPAAug.transpose() * gGain * aug->matI;
  for (int n = 1; n < nPreviews; ++n) {
      matPrevGain[n] = temp * (matBAugt * prevState)(0, 0);
      prevState = matPAAug.transpose() * prevState;
  }
  trueState = model->getState();
  trueState[1] = 0.0;
  trueState[2] = 0.0;
  LOG_INFO("trueState at start:" << trueState.transpose());
  model->setTrueState(trueState);
  model->setInput(0.0);
  //model->setObserver(true);
  updateGains();
}

template<typename Scalar>
void ZmpPreviewController<Scalar>::updateGains()
{
  aug->updateCMatrix(model->getOutputMatrix());
  gGain = aug->solveDare();
  auto matBAugt = aug->matBAug.transpose();
  Scalar temp = 1 / (aug->R + (matBAugt * gGain * aug->matBAug)(0, 0));
  kGain = temp * matBAugt * gGain * aug->matAAug;
  Matrix<Scalar, Dynamic, Dynamic> matPAAug = aug->matAAug - aug->matBAug * kGain;
  matPrevGain.resize(nPreviews, 1);
  matPrevGain[0] = -kGain(0, 0);
  Matrix<Scalar, Dynamic, Dynamic> prevState = -matPAAug.transpose() * gGain * aug->matI;
  for (int n = 1; n < nPreviews; ++n) {
      matPrevGain[n] = temp * (matBAugt * prevState)(0, 0);
      prevState = matPAAug.transpose() * prevState;
  }

  /*Scalar R = 1.0;
  Matrix<Scalar, 3, 3> obsQ;
  obsQ.setIdentity();
  GET_CONFIG(
    "MotionBehaviors",
    (Scalar, ZmpControl.obsQ1, obsQ(0, 0)),
    (Scalar, ZmpControl.obsQ2, obsQ(1, 1)),
    (Scalar, ZmpControl.obsQ3, obsQ(2, 2)),
    (Scalar, ZmpControl.obsR, R),
  );
  model->computeObsGain(obsQ, R);*/

}

template<typename Scalar>
Matrix<Scalar, Dynamic, 1> ZmpPreviewController<Scalar>::step(const Matrix<Scalar, Dynamic, 1>& zmpRef)
{
  //updateGains();
  /*Scalar R = 1.0;
  Matrix<Scalar, 3, 3> obsQ;
  obsQ.setIdentity();
  GET_CONFIG(
    "MotionBehaviors",
    (Scalar, ZmpControl.obsQ1, obsQ(0, 0)),
    (Scalar, ZmpControl.obsQ2, obsQ(1, 1)),
    (Scalar, ZmpControl.obsQ3, obsQ(2, 2)),
    (Scalar, ZmpControl.obsR, R),
  );
  model->computeObsGain(obsQ, R);*/
  model->setTrueState(trueState);
  //Matrix<Scalar, 3, 1> estState = model->getState();
  //LOG_INFO("State Error: " << (trueState - estState).transpose());
  Scalar prevGain = matPrevGain.dot(zmpRef);
  //intError = intError + ((model->getOutputMatrix() * estState)(0, 0) - zmpRef[0]);
  trueIntError = trueIntError + ((model->getOutputMatrix() * trueState)(0, 0) - zmpRef[0]);
  //Scalar controlInput =
  //  -kGain(0, 0) * intError - (kGain.block(0, 1, 1, 3) * estState)(0, 0) - prevGain;
  Scalar controlInputTrue =
    -kGain(0, 0) * trueIntError - (kGain.block(0, 1, 1, 3) * trueState)(0, 0) - prevGain;
  trueState = model->getUpdatedState(trueState, controlInputTrue);
  //model->setControl(controlInputTrue);
  return trueState;
}

template<typename Scalar>
Matrix<Scalar, Dynamic, 1> ZmpPreviewController<Scalar>::stepActual(const Matrix<Scalar, Dynamic, 1>& zmpRef)
{
  updateGains();
  /*Scalar R = 1.0;
  Matrix<Scalar, 3, 3> obsQ;
  obsQ.setIdentity();
  GET_CONFIG(
    "MotionBehaviors",
    (Scalar, ZmpControl.obsQ1, obsQ(0, 0)),
    (Scalar, ZmpControl.obsQ2, obsQ(1, 1)),
    (Scalar, ZmpControl.obsQ3, obsQ(2, 2)),
    (Scalar, ZmpControl.obsR, R),
  );
  model->computeObsGain(obsQ, R);*/
  //model->setTrueState(trueState);
  Matrix<Scalar, 3, 1> estState = model->getState();
  //cout << "comState:"  << estState.transpose() << endl;
  //cout << "zmpRef: " << zmpRef.transpose() << endl;
  //cout << "currentZmp: " << (model->getOutputMatrix() * estState)(0, 0) << endl;
  //LOG_INFO("State Error: " << (trueState - estState).transpose());
  Scalar prevGain = matPrevGain.dot(zmpRef);
  intError = intError + ((model->getOutputMatrix() * estState)(0, 0) - zmpRef[0]);
  //trueIntError = trueIntError + ((model->getOutputMatrix() * trueState)(0, 0) - zmpRef[0]);
  Scalar controlInput =
    -kGain(0, 0) * intError - (kGain.block(0, 1, 1, 3) * estState)(0, 0) - prevGain;
  //Scalar controlInputTrue =
  //  -kGain(0, 0) * trueIntError - (kGain.block(0, 1, 1, 3) * trueState)(0, 0) - prevGain;
  //trueState = model->getUpdatedState(trueState, controlInputTrue);
  model->setInput(controlInput);
  return model->getUpdatedState(trueState, controlInput);
}

template class ZmpPreviewController<MType>;
