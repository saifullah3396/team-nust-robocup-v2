#ifndef STATEMACHINEMACROS_H
#define STATEMACHINEMACROS_H

#include "StateMachine.h"

#define DECLARE_FUNC_(FunctionName) \
  virtual void FunctionName();

#define DECLARE_FSM_STATE_(className, stateName, varName, ...) \
  struct stateName : public FSMState<className> \
  { \
    stateName(className* bPtr) : FSMState<className>(bPtr, string(#stateName)) {} \
    FOR_EACH(DECLARE_FUNC_, __VA_ARGS__) \
  }; \
  unique_ptr<stateName> varName; \

#define DECLARE_FSM_STATE(...) \
  DECLARE_FSM_STATE_(__VA_ARGS__)

#define DEFINE_FSM_STATE(className, stateName, varName) \
  varName = unique_ptr<stateName>(new stateName(this)); \
  this->fsmStates.insert(pair<string, FSMState<className>*>(string(#stateName), varName.get()));

#define DECLARE_FSM(varName, className) \
  string getFsmState() override { \
    return "FsmState: " + this->varName->state->name; \
  } \
  unique_ptr<StateMachine<className> > varName; \
  map<string, FSMState<className>*> fsmStates;

#define DEFINE_FSM(varName, className, startState) \
  varName = unique_ptr<StateMachine<className>>( \
    new StateMachine<className>(this, startState.get()));

#endif // STATEMACHINEMACROS_H
