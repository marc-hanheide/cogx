/**
 * @author Marko Mahnič
 * @created April 2012
 */
#include "statemachine.hpp"

#include <sstream>

namespace testing
{

std::map<CState::EPhase, std::string> CState::mPhaseName;
CState::CStaticInit::CStaticInit()
{
#define __PHASE(st) mPhaseName[st] = #st
  __PHASE(phInit);
  __PHASE(phEnter);
  __PHASE(phWork);
  __PHASE(phExit);
  __PHASE(phDone);
#undef __PHASE
}

TStateFunctionResult CState::noAction(CState* pState)
{
  assert(pState != nullptr);
  return Continue;
}

TStateFunctionResult CState::enter()
{
  if (_enter)
    return _enter(this);
  return NotImplemented;
}

TStateFunctionResult CState::work()
{
  if (_work)
    return _work(this);
  return NotImplemented;
}

TStateFunctionResult CState::exit()
{
  if (_exit)
    return _exit(this);
  return NotImplemented;
}


void CState::advance()
{
  switch(mPhase) {
    case phInit:
      mPhase = phEnter;
      break;
    case phEnter:
      mPhase = phWork;
      break;
    case phWork:
      mPhase = phExit;
      break;
    case phExit:
      mPhase = phDone;
      break;
    default: break; // assert(can't advance)
  }
}

void CState::execute()
{
  TStateFunctionResult rv = Continue;
  mActivity = acActive;

  switch(mPhase) {
    case phInit:
      mPhase = phEnter;
      break;

    case phEnter:
      this->enter();
      if (mFirstSleepMs > 0) {
        mSleepTimer.setTimeout(mFirstSleepMs, true);
        mActivity = acSleeping;
      }
      else {
        mFirstSleepMs = 0;
      }
      if (mTimeoutMs > 0) {
        mTimeoutTimer.setTimeout(mTimeoutMs + mFirstSleepMs, true);
      }
      else {
        mTimeoutMs = 0;
      }
      if (rv == WaitChange) {
        mActivity = acWaiting;
      }
      advance();
      break;

    case phWork:
      mbSwitchStateCalled = false;
      rv = this->work();
      if (rv == NotImplemented) {
        advance();
      }
      else {
        if (mbSwitchStateCalled) {
          rv = Continue;
        }
        if (rv == Continue) {
          advance();
        }
        else if (rv == Sleep) {
          mSleepTimer.setTimeout(mSleepMs, true);
          mActivity = acSleeping;
        }
        else if (rv == WaitChange) {
          mActivity = acWaiting;
        }
      }
      break;

    case phExit:
      this->exit();
      advance();
      break;

    default: break; // Terminated, assert(false)
  }
}

CLinkedStatePtr CState::linkedState(const std::string& stateName, const std::string& reasons)
{
  for (auto st : mLinkedStates) {
    if (st->mId == stateName)
      return st;
  }
  auto st = CLinkedStatePtr(new CLinkedState(mpMachine, this, stateName, reasons));
  mLinkedStates.push_back(st);
  return st;
}

std::string CLinkedState::description()
{
  std::ostringstream ss;
  ss << mpFromState->id() << " -> " << mId << " (" << mReasons << ")";
  return ss.str();
}


void CMachine::switchToState(CState* pState)
{
  assert(pState);
  if (! mpCurrentState.get()) {
    // Initial state
    mpCurrentState.reset(pState);
    mpNextState.reset();
    return;
  }
  assert (mpCurrentState->mPhase == CState::phWork);
  mpNextState.reset(pState);
  mpCurrentState->mbSwitchStateCalled = true;
}

void CMachine::runOneStep()
{
  if (! mpCurrentState.get()) return;

  if (mpCurrentState->mActivity == CState::acSleeping) {
    if (mpCurrentState->mSleepTimer.timeToWait() > 0)
      return;
    mpCurrentState->mActivity = CState::acActive;
  }

  if (mpCurrentState->mActivity == CState::acWaiting) {
    if (mpCurrentState->mEvents.size() > 0) {
      // The caller will call checkEvents() which may cause the activity to
      // become other than acWaiting.
      return;
    }
    // acWaiting, but no events were registered => switch to acActive
    mpCurrentState->mActivity = CState::acActive;
  }

  if (mpCurrentState->mPhase == CState::phDone) {
    if (! mpNextState.get()) {
      // Terminated
      return;
    }
    onTransition(mpCurrentState, mpNextState);

    mpPrevState = mpCurrentState;
    mpCurrentState = mpNextState;
    mpCurrentState->mPhase = CState::phInit;
    mpCurrentState->mActivity = CState::acActive;
    mpNextState.reset();
  }
  mpCurrentState->execute();
}

}// namespace
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim
