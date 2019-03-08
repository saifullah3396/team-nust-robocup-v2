/**
 * @file Utils/inlclude/Filters/ProcessModel.h
 *
 * This file defines the class ProcessModel
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017
 */

#pragma once

#include "Utils/include/MathsUtils.h"
#include "Utils/include/PrintUtils.h"

/**
 * @class ProcessModel
 * @brief Defines the base class for defining a filter process update
 */
template <typename Scalar, size_t StateSize, size_t InputSize, size_t OutputSize>
class ProcessModel
{
public:
  ProcessModel() = default;
  ProcessModel(const ProcessModel&) = default;
  ProcessModel(ProcessModel&&) = default;
  ProcessModel& operator=(const ProcessModel&) & = default;
  ProcessModel& operator=(ProcessModel&&) & = default;

  /**
   * @brief ProcessModel Initializes the process model based
   *   on the given data
   * @param stateSize size of the state vector
   * @param inputSize size of the input input vector
   * @param A state transition matrix
   * @param B input input matrix
   */
  ProcessModel(
    const Matrix<Scalar, StateSize, StateSize>& A,
    const Matrix<Scalar, StateSize, 1>& B) :
    A(A), B(B)
  {
  }

  /**
   * @brief ProcessModel Initializes the process model based
   *   on the given data
   * @param stateSize size of the state vector
   * @param inputSize size of the input input vector
   * @param A state transition matrix
   * @param B input input matrix
   * @param C system output matrix
   */
  ProcessModel(
    const Matrix<Scalar, StateSize, StateSize>& A,
    const Matrix<Scalar, StateSize, 1>& B,
    const Matrix<Scalar, OutputSize, StateSize>& C) :
    A(A), B(B), C(C)
  {
  }

  /**
   * @brief ~ProcessModel Destructor
   */
  virtual ~ProcessModel() {}

  /**
   * @brief update The state-transition update
   */
  void update() {
    if (!updated) {
      if (useObserver)
        state = A * state + B * input + obsGain * (C * (trueState - state));
      else
        state = A * state + B * input;
      updated = true;
    }
  }

  /**
   * @brief getOutput returns the output defined by C * state
   */
  Matrix<Scalar, OutputSize, 1> getOutput() {
    return C * state;
  }

  /**
   * @brief computeErrorCov computes the error cov for the next state based
   *   on the given previous error cov
   * @param prevP prev error covariance matrix
   */
  Matrix<Scalar, StateSize, StateSize>
    computeErrorCov(const Matrix<Scalar, StateSize, StateSize>& prevP)
  {
    return A * prevP * At + Q;
  }

  /**
   * @brief setStateMatrix sets the state transition matrix
   * @param A state transition matrix
   */
  void setStateMatrix(const Matrix<Scalar, StateSize, StateSize>& A)
  {
    this->A = A;
    this->At = A.transpose();
  }

  /**
   * @brief setNoiseCovMatrix sets the process noise covariance matrix
   * @param Q rocess noise covariance matrix
   */
  void setNoiseCovMatrix(const Matrix<Scalar, StateSize, StateSize>& Q)
  {
    this->Q = Q;
  }

  /**
   * @brief setInputMatrix sets the input matrix
   * @param B input input matrix
   */
  void setInputMatrix(const Matrix<Scalar, StateSize, 1>& B)
  {
    this->B = B;
  }

  /**
   * @brief setOutputMatrix sets the system output matrix
   * @param C system output matrix
   */
  void setOutputMatrix(const Matrix<Scalar, OutputSize, StateSize>& C)
  {
    this->C = C;
  }

  /**
   * @brief setState sets the state vector
   * @param state state vector
   */
  void setState(const Matrix<Scalar, StateSize, 1>& state)
  {
    this->state = state;
  }

  /**
   * @brief setState Sets the state value at the given index
   * @param val State value
   * @param index State index
   */
  void setState(const Scalar& val, const unsigned& index)
  {
    this->state[index] = val;
  }

  /**
   * @brief setTrueState sets the true state vector
   * @param trueState true state vector
   */
  void setTrueState(const Matrix<Scalar, StateSize, 1>& trueState)
  {
    this->trueState = trueState;
  }

  /**
   * @brief setInput sets the input input vector
   * @param input input input vector
   */
  void setInput(const Matrix<Scalar, 1, InputSize>& input)
  {
    this->input = input;
  }

  /**
   * @brief setInput sets the input input
   * @param input input input
   */
  void setInput(const Scalar& input)
  {
    this->input[0] = input;
  }

  /**
   * @brief getUpdatedState computes the state for given input
   * @param input input input
   */
  Matrix<Scalar, StateSize, 1> getUpdatedState(
    const Matrix<Scalar, StateSize, 1> prevState, const Scalar& input, const bool& withObserver = false)
  {
    if (withObserver) {
      return A * prevState + B * input + obsGain * (C * (trueState - prevState));
    } else {
      return A * prevState + B * input;
    }
  }

  /**
   * @brief getStateSize returns the state size
   */
  unsigned getStateSize()
    { return StateSize; }

  /**
   * @brief getInputSize returns the input size
   */
  unsigned getInputSize()
    { return InputSize; }

  /**
   * @brief getStateMatrix returns the state matrix
   */
  Matrix<Scalar, StateSize, StateSize> getStateMatrix()
    { return A; }

  /**
   * @brief getInputMatrix returns the input matrix
   */
  Matrix<Scalar, StateSize, 1> getInputMatrix()
    { return B; }

  /**
   * @brief getInputMatrix returns the system output matrix
   */
  Matrix<Scalar, OutputSize, StateSize> getOutputMatrix()
    { return C; }

  /**
   * @brief getState returns the state
   */
  const Matrix<Scalar, StateSize, 1>& getState()
    { return state; }

  /**
   * @brief getInput returns the input input
   */
  const Matrix<Scalar, 1, InputSize>& getInput()
    { return input; }

  /**
   * @brief setUpdated Whether the model has been updated for this cycle.
   *   This is used by the filter to update only once for prediction step since
   *   the model can be updated externally as well
   * @param updated true or false
   */
  void setUpdated(const bool& updated) { this->updated = updated; }

  void setObserver(const bool& useObserver) { this->useObserver = useObserver; }

  void computeObsGain(
    Matrix<Scalar, 3, 3>& obsQ, Scalar& R) {
    // using duality system<=>observer
    Matrix<Scalar, 3, 3> obsA = A.transpose();
    Matrix<Scalar, 3, 1> obsB = C.transpose();
    Matrix<Scalar, 3, 3>
      P = MathsUtils::dare<Scalar, 3>(obsA, obsB, obsQ, R).transpose();
    obsGain = (1.0 / (R + (obsB.transpose() * P * obsB)(0, 0))) * obsB.transpose() * P * obsA;
    //matAl = model->getStateMatrix() - obsGain * model->getOutputMatrix();
    //EigenSolver<Matrix<Scalar, 3, 3> > eigensolver(matAl);
    LOG_INFO("ObsGain" << obsGain)
  }

private:
  //! State transition matrix
  Matrix<Scalar, StateSize, StateSize> A = {Matrix<Scalar, StateSize, StateSize>::Zero()};

  //! State transition matrix transpose
  Matrix<Scalar, StateSize, StateSize> At = {Matrix<Scalar, StateSize, StateSize>::Zero()};

  //! Input input matrix
  Matrix<Scalar, StateSize, 1> B = {Matrix<Scalar, StateSize, 1>::Zero()};

  //! System output matrix
  Matrix<Scalar, OutputSize, StateSize> C = {Matrix<Scalar, OutputSize, StateSize>::Zero()};

  //! State vector
  Matrix<Scalar, StateSize, 1> state = {Matrix<Scalar, StateSize, 1>::Zero()};

  //! True state vector if known
  Matrix<Scalar, StateSize, 1> trueState = {Matrix<Scalar, StateSize, 1>::Zero()};

  //! Input input vector
  Matrix<Scalar, 1, InputSize> input = {Matrix<Scalar, 1, InputSize>::Zero()};

  //! Process noise covariance matrix
  Matrix<Scalar, StateSize, StateSize> Q = {Matrix<Scalar, StateSize, StateSize>::Zero()};

  //! Observer gain
  Matrix<Scalar, StateSize, 1> obsGain;

  //! Whether the update has been called already
  bool updated = {false};

  bool useObserver = {false};
};
