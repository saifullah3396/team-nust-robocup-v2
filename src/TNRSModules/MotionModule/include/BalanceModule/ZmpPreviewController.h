/**
 * @file MotionModule/BalanceModule/ZmpPreviewController.h
 *
 * This file declares the class ZmpPreviewController
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include "MotionModule/include/MTypeHeader.h"
#include "Utils/include/Constants.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/PrintUtils.h"

template <typename Scalar, size_t StateSize, size_t InputSize, size_t OutputSize>
class ProcessModel;

/**
 * @class ZmpPreviewController
 * @brief The class for the implementation of zmp controller based
 * on Kajita's inverted-cart-table model.
 */
template <typename Scalar>
class ZmpPreviewController
{
public:
  /**
   * Default constructor for this class.
   */
  ZmpPreviewController(
    boost::shared_ptr<ProcessModel<Scalar, 3, 1, 1> > model);

  /**
   * Default destructor for this class.
   */
  ~ZmpPreviewController()
  {
  }

  /**
   * Initiates the controller.
   */
  void initController();

  /**
   * Updates the system model based on the new control input
   *
   * @param zmpRef: zmp references for the given dimension
   *
   * @return Matrix<Scalar, 3, 1> next state vector
   */
  Matrix<Scalar, Dynamic, 1> step(const Matrix<Scalar, Dynamic, 1>& zmpRef);

  /**
   * @brief stepActual Updates the controller based on actual com state
   * @param zmpRef Current zmp references
   * @return Center of mass state
   */
  Matrix<Scalar, Dynamic, 1> stepActual(const Matrix<Scalar, Dynamic, 1>& zmpRef);

  /**
   * Sets the number of previewed steps for the preview controller.
   *
   * @param nPreviews: number of previews
   */
  void setPreviewLength(const unsigned& nPreviews)
  {
    this->nPreviews = nPreviews;
  }

  void setTrueState(const Matrix<Scalar, 3, 1>& trueState) {
    this->trueState = trueState;
  }

  Matrix<Scalar, 3, 1> getTrueState() {
    return trueState;
  }

protected:
  /**
   * @brief updateGains Update gains for current com state
   */
  void updateGains();

  //! Number of previews for the preview controller.
  int nPreviews;

  //! Error matrices, preview gain matrix, controlInputs (com jerk),
  Scalar intError, trueIntError;

  //! Optimal gain matrices.
  Matrix<Scalar, Dynamic, Dynamic> kGain, gGain;

  Matrix<Scalar, Dynamic, 1> matPrevGain;

  //! Process model
  boost::shared_ptr<ProcessModel<Scalar, 3, 1, 1> > model;

  //! State
  Matrix<Scalar, 3, 1> trueState;

  //! First state check
  bool firstState;

  template <unsigned StateSize>
  struct AugmentedStateMats {
    AugmentedStateMats(
      const boost::shared_ptr<ProcessModel<Scalar, 3, 1, 1> >& model)
    {
      cout << "model: " << model << endl;
      matA = model->getStateMatrix();
      cout << "A: " << matA << endl;
      matB = model->getInputMatrix();
      cout << "B: " << matB << endl;
      matC = model->getOutputMatrix();
      cout << "C: " << matC << endl;
      matI << 1.0, Matrix<Scalar, StateSize, 1>::Zero();
      Matrix<Scalar, 1, StateSize> matCA = matC * matA;
      Matrix<Scalar, 1, StateSize + 1> mat1CA;
      mat1CA << 1, matCA;
      Matrix<Scalar, StateSize, StateSize + 1> mat0A;
      mat0A << Matrix<Scalar, StateSize, 1>::Zero(), matA;
      matAAug << mat1CA, mat0A;
      matBAug << (matC * matB)(0, 0), matB;
      matQ.setZero();
      matQ(0, 0) = 1.0;
      R = 1e-6;
    }
    AugmentedStateMats() = default;
    ~AugmentedStateMats() = default;

    void updateCMatrix(const Matrix<Scalar, 1, StateSize>& C) {
      matC = C;
      Matrix<Scalar, 1, StateSize + 1> mat1CA;
      mat1CA << 1, matC * matA;
      matAAug.block(0, 0, 1, StateSize + 1) = mat1CA;
      matBAug(0, 0) = (matC * matB)(0, 0);
    }

    Matrix<Scalar, StateSize, StateSize> matA;
    Matrix<Scalar, StateSize, 1> matB;
    Matrix<Scalar, 1, StateSize> matC;
    Matrix<Scalar, StateSize + 1, 1> matI;
    Matrix<Scalar, StateSize + 1, StateSize + 1> matAAug;
    Matrix<Scalar, StateSize + 1, 1> matBAug;
    Matrix<Scalar, StateSize + 1, StateSize + 1> matQ;
    Scalar R;
    unsigned stateSize;

    /**
     * Solves dare for current state
     *
     * @return Matrix<Scalar, StateSize + 1, StateSize + 1>
     */
    Matrix<Scalar, StateSize + 1, StateSize + 1> solveDare()
    {
      return MathsUtils::dare<Scalar, StateSize + 1>(matAAug, matBAug, matQ, R);
    }

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  boost::shared_ptr<AugmentedStateMats<3> > aug;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
