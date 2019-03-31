#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "Utils/include/Filters/KalmanFilter.h"

int main()
{
  boost::shared_ptr<ProcessModel<float, 3, 1, 1> > modelPtr =
    boost::shared_ptr<ProcessModel<float, 3, 1, 1> >(
      new ProcessModel<float, 3, 1, 1>());
  Matrix<float, 3, 3> A;
  A << 1.0, 0.01, 0.01*0.01/2, 0.0, 1.0, 0.01, 0.0, 0.0, 1.0;
  modelPtr->setStateMatrix(A);
  modelPtr->setInputMatrix(Matrix<float, 3, 1>::Identity());
  modelPtr->setOutputMatrix(Matrix<float, 1, 3>::Identity());
  Matrix<float, 3, 3> Q;
  Q = Q.setIdentity() * 0.001;
  modelPtr->setNoiseCovMatrix(Q);
  cout << "StateMatrix:\n" << modelPtr->getStateMatrix() << endl;
  cout << "InputMatrix:\n" << modelPtr->getInputMatrix() << endl;
  cout << "OutputMatrix:\n" << modelPtr->getOutputMatrix() << endl;
  cout << "State:\n" << modelPtr->getState().transpose() << endl;
  cout << "Input:\n" << modelPtr->getInput().transpose() << endl;
  cout << "Output:\n" << modelPtr->getOutput().transpose() << endl;

  KalmanFilter<float, 3, 3, 1, 1> kf;
  kf.setModel(modelPtr);
  Matrix<float, 3, 3> H;
  Matrix<float, 3, 3> R;
  H << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  R = R.setIdentity() * 0.001;
  kf.setMeasMatrix(H);
  kf.setMeasNoiseCov(R);
  Matrix<float, 3, 1> meas(10.0, 0.0, 0.0);
  for (size_t i = 0; i < 1000; ++i) {
    kf.predict();
    cout << "Predicted state:" << modelPtr->getState().transpose() << endl;
    kf.correct(meas);
    cout << "Corrected state:" << modelPtr->getState().transpose() << endl;
    cout << "Q:" << modelPtr->getNoiseCovMatrix() << endl;
    cout << "P:" << kf.getErrorNoiseCov() << endl;
  }
  return 0;
}

