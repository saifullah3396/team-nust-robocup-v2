/**
 * @file BehaviorManager/include/StateMachine.h
 *
 * This file declares the classes FSMState and FSMState
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#pragma once

#include <memory>
#include "Utils/include/VariadicMacros.h"

template <typename Behavior>
struct FSMState
{
  FSMState(Behavior* bPtr, std::string name) :
    bPtr(bPtr), name(name), nextState(this) {}
  virtual void onStart() {}
  virtual void onRun() = 0;
  virtual void onStop() {}
  void reset() { nextState = this; }
  FSMState<Behavior>* getNextState() { return nextState; }
  FSMState<Behavior>* nextState;
  Behavior* bPtr;
  std::string name;
};

template <typename Behavior>
struct StateMachine
{
  StateMachine(Behavior* bPtr, FSMState<Behavior>* startState) :
    bPtr(bPtr), state(startState), lastState(nullptr) {}
  bool update() {
    state = state->getNextState();
    if (!state) {
      return true;
    }
    if (state != lastState) {
      if (lastState) {
        lastState->onStop();
        lastState->reset();
      }
      state->onStart();
      lastState = state;
      return false;
    }
    state->onRun();
    lastState = state;
    return false;
  }
  FSMState<Behavior>* state;
  FSMState<Behavior>* lastState;
  Behavior* bPtr;
};
