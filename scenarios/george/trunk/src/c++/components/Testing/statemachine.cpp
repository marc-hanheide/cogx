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

void CState::restart()
{
  mInfo = mInitInfo;
  mPhase = CState::phInit;
  mActivity = CState::acActive;
  mbSwitchStateCalled = false;
  msExitReason = "";
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

std::string CState::getProgressText()
{
  return "";
}

void CState::execute()
{
  TStateFunctionResult rv = Continue;
  mActivity = acActive;

  switch(mPhase) {
    case phInit:
      restart();
      mPhase = phEnter;
      // fall through

    case phEnter:
      rv = this->enter();
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

void CMachine::writeStateDescription(std::ostringstream& ss)
{
  if (!mpCurrentState.get()) {
    ss << "NULL";
    return;
  }
  ss << mpCurrentState->id();
  ss << "</td><td>";
  if (mpCurrentState->mActivity == CState::acSleeping) {
    long tts = timeToSleep();
    //if (tts > 0) {
    char cbuf[20];
    std::sprintf(cbuf, " Sleep(%.3fs)", tts / 1000.0);
    ss << cbuf;
    //}
  }
  if (isWaitingForEvent()) {
    ss << " WaitEvent";
  }
  if (mpCurrentState->msExitReason.size()) {
    ss << " " << mpCurrentState->msExitReason;
  }
  ss << "</td><td>";
  ss << mpCurrentState->getProgressText();
}

void CMachine::writeMachineDescription(std::ostringstream& ss)
{
}

void CMachine::onTransition(CStatePtr fromState, CStatePtr toState)
{
}

void CMachine::switchToState(CStatePtr pState, const std::string& reason)
{
  assert(pState.get());
  if (! mpCurrentState.get()) {
    // Initial state
    mpCurrentState = pState;
    mpNextState.reset();
    mStepNumber = 1;
    return;
  }
  assert (mpCurrentState->mPhase == CState::phWork);
  mpNextState = pState;
  mpCurrentState->mbSwitchStateCalled = true;
  mpCurrentState->msExitReason = "Done(" + reason + ")";
}

void CMachine::checkEvents()
{
  if (isWaitingForEvent() && mReceivedEvents.size()) {
    std::map<std::string, bool> tmp;
    {
      std::lock_guard<std::mutex> lock(mReceivedEventsMutex);
      tmp = mReceivedEvents;
      mReceivedEvents.clear();
    }
    for (auto s : tmp) {
      if (mpCurrentState->isWaitingForEvent(s.first)) {
        mpCurrentState->mActivity = CState::acActive;
        break;
      }
    }
  }
}


void CMachine::runOneStep()
{
  if (! mpCurrentState.get()) return;

  if (mpCurrentState->mActivity == CState::acSleeping) {
    if (mpCurrentState->mSleepTimer.timeToWait() > 0)
      return;
    mpCurrentState->mActivity = CState::acActive;
  }

  if (mpCurrentState->mActivity == CState::acWaiting && !mpCurrentState->hasTimedOut()) {
    if (mpCurrentState->mEvents.size() > 0) {
      // Periodically wake-up a waiting state
      if (mWaitTimer.elapsed() > 2000) {
        mpCurrentState->execute();
        mWaitTimer.restart();
      }

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

    ++mStepNumber;
    mpPrevState = mpCurrentState;
    mpCurrentState = mpNextState;
    mpCurrentState->restart();
    mpNextState.reset();
    mWaitTimer.restart();
  }
  mpCurrentState->execute();
}

}// namespace
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim
