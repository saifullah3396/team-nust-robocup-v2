/**
 * @file MotionModule/BalanceModule/Types/PIDComControl.cpp
 *
 * This file implements the class PIDComControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include "MotionModule/include/BalanceModule/Types/PIDComControl.h"

template<typename Scalar>
PIDComControl<Scalar>::PIDComControl(
  MotionModule* motionModule,
  const boost::shared_ptr<PIDComControlConfig>& config) :
  BalanceModule<Scalar>(motionModule, config, "PIDComControl")
{
}

template<typename Scalar>
bool PIDComControl<Scalar>::initiate()
{
  //! Behavior not yet defined
  LOG_ERROR("Behavior PIDComControl is undefined")
  return false;
}

template<typename Scalar>
void PIDComControl<Scalar>::update() {}

template<typename Scalar>
void PIDComControl<Scalar>::finish()
{
  this->inBehavior = false;
}

template<typename Scalar>
PIDComControlConfigPtr PIDComControl<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <PIDComControlConfig> (this->config);
}

template class PIDComControl<MType>;
