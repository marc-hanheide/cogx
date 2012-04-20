/**
 * @author Marko Mahnič
 * @created April 2012
 */
#ifndef _TESTING_STATEMACHINE_HPP_4F7967EF_
#define _TESTING_STATEMACHINE_HPP_4F7967EF_

#include <castutils/Timers.hpp>

#include <cassert>
#include <memory>
#include <string>
#include <map>
#include <functional>

namespace testing
{

class CMachine;
typedef std::shared_ptr<CMachine> CMachinePtr;

class CState;
typedef std::shared_ptr<CState> CStatePtr;

class CLinkedState;
typedef std::shared_ptr<CLinkedState> CLinkedStatePtr;

enum TStateFunctionResult {
  Continue,    // go to next stage/state
  Sleep,       // wait in this stage (uses sleep timer)
  WaitChange,  // wait for (any) registered WM change
  NotImplemented
};
typedef std::function<TStateFunctionResult(CState*)> TStateFunction;

class CState
{
private:
  enum EActivity {
    acActive, acSleeping, acWaiting, acDone
  };
  enum EPhase {
    phInit, phEnter, phWork, phExit, phDone
  };
  static std::map<EPhase, std::string> mPhaseName;
  class CStaticInit { CStaticInit(); };

private:
  friend class CMachine;
  CMachine* mpMachine;
  std::string mId;
  EPhase mPhase;
  EActivity mActivity;
  long mFirstSleepMs;
  long mSleepMs;
  long mTimeoutMs;
  bool mbSwitchStateCalled;
  castutils::CMilliTimer mSleepTimer;
  castutils::CMilliTimer mTimeoutTimer;
  std::vector<std::string> mEvents;

  // List of registered exits from this state. It enables link verification
  // before the machine is started. Entries are added in linkedState().
  std::vector<CLinkedStatePtr> mLinkedStates;

  TStateFunction _enter;
  TStateFunction _work;
  TStateFunction _exit;

public:
  CState(CMachine* pmachine, std::string id);

  const std::string& id();
  // set during configuration
  void setEnter(TStateFunction pEnter);
  void setWork(TStateFunction pWork);
  void setExit(TStateFunction pExit);

  // The default implementation of enter() calls the function set by setEnter()
  // (same for work and exit). The programmer can either call setEnter or/and
  // reimplement the enter() method in a derived class.
  virtual TStateFunctionResult enter();
  virtual TStateFunctionResult work();
  virtual TStateFunctionResult exit();

  // set in enter or during configuration
  void setSleepTime(long working, long first=0);
  void setTimeout(long milliseconds);
  void setWatchEvents(const std::vector<std::string>& events);

  CLinkedStatePtr linkedState(const std::string& stateName, const std::string& reasons="");
  // TODO: std::vector<CLinkedStatePtr> getBadLinks();

  bool isWaitingForEvent(const std::string& evnet);
  bool hasTimedOut();

private:
  void execute();
  void advance();
  static TStateFunctionResult noAction(CState* pState); // default TStateFunction
};

class CLinkedState
{
private:
  friend class CState;
  CMachine* mpMachine;
  std::string mId;
  std::string mReasons;
  CState* mpFromState;
  CState* mpToState;
  CLinkedState(CMachine* pMachine, CState* pFromState, const std::string& toStateName, const std::string& reasons);
public:
  CState* getState();
  std::string description();
};

class CMachine
{
private:
  std::map<std::string, CStatePtr> mStates;
  CStatePtr mpPrevState;
  CStatePtr mpCurrentState;
  CStatePtr mpNextState;
public:
  CStatePtr addState(std::string id);
  CStatePtr addState(CState* pState);
  CStatePtr findState(std::string id);
  void switchToState(CState* pNextState);
  void switchToState(CStatePtr pNextState);
  void switchToState(CLinkedStatePtr pNextState);
  void runOneStep();
  virtual void onTransition(CStatePtr fromState, CStatePtr toState);
  bool isWaitingForEvent();
  bool isFinished();
  void checkEvents(const std::vector<std::string>& events);
  long timeToSleep();
};

template<class MachineT>
class CMachineStateMixin
{
  MachineT* __mpMachine;
public:
  CMachineStateMixin(MachineT* pMachine);
  MachineT* machine();
};


inline
CState::CState(CMachine* pmachine, std::string id)
  : mpMachine(pmachine), mId(id), mPhase(phInit), mSleepMs(0), mTimeoutMs(0),
  _enter(noAction), _work(noAction), _exit(noAction)
{
}

inline
const std::string& CState::id()
{
  return mId;
}

inline
void CState::setWatchEvents(const std::vector<std::string>& events)
{
  mEvents = events;
}

inline
bool CState::isWaitingForEvent(const std::string& event)
{
  for (auto s : mEvents) {
    if (event.find(s) != std::string::npos) {
      return true;
    }
  }
  return false;
}

inline
bool CState::hasTimedOut()
{
  if (mTimeoutMs > 0) {
    return mTimeoutTimer.isTimeoutReached();
  }
  return false;
}

inline
void CState::setEnter(TStateFunction pEnter)
{
  this->_enter = pEnter;
}

inline
void CState::setWork(TStateFunction pWork)
{
  this->_work = pWork;
}

inline
void CState::setExit(TStateFunction pExit)
{
  this->_exit = pExit;
}

inline
CLinkedState::CLinkedState(CMachine* pMachine, CState* pFromState,
    const std::string& toStateName, const std::string& reasons)
:mpMachine(pMachine), mId(toStateName), mReasons(reasons),
  mpFromState(pFromState), mpToState(nullptr)
{
}

inline
CState* CLinkedState::getState()
{
  if (!mpToState) {
    mpToState = mpMachine->findState(mId).get();
  }
  return mpToState;
}

inline
CStatePtr CMachine::addState(std::string id)
{
  CStatePtr pState(new CState(this, id));
  mStates[id] = pState;
  return pState;
}

inline
CStatePtr CMachine::addState(CState* pState)
{
  assert(pState);
  CStatePtr pst(pState);
  mStates[pst->mId] = pst;
  return pst;
}

inline
CStatePtr CMachine::findState(std::string id)
{
  auto it = mStates.find(id);
  if (it == mStates.end()) {
    return CStatePtr();
  }
  return mStates[id];
}

inline
void CMachine::switchToState(CStatePtr pNextState)
{
  switchToState(pNextState.get());
}

inline
void CMachine::switchToState(CLinkedStatePtr pNextState)
{
  switchToState(pNextState->getState());
}

inline
bool CMachine::isWaitingForEvent()
{
  if (! mpCurrentState.get()) return false;
  return mpCurrentState->mActivity == CState::acWaiting;
}

inline
bool CMachine::isFinished()
{
  if (! mpCurrentState.get()) return true;
  return (mpCurrentState->mPhase == CState::phDone && !mpNextState.get());
}

inline
void CMachine::checkEvents(const std::vector<std::string>& events)
{
  if (isWaitingForEvent()) {
    for (auto s : events) {
      if (mpCurrentState->isWaitingForEvent(s)) {
        mpCurrentState->mActivity = CState::acActive;
        break;
      }
    }
  }
}

inline
long CMachine::timeToSleep()
{
  if (!mpCurrentState.get()) {
    return 0;
  }
  if (mpCurrentState->mActivity == CState::acSleeping) {
    return mpCurrentState->mSleepTimer.timeToWait();
  }
  return 0;
}

template<class MachineT>
CMachineStateMixin<MachineT>::CMachineStateMixin(MachineT* pMachine)
{
  __mpMachine = pMachine;
}

template<class MachineT>
MachineT* CMachineStateMixin<MachineT>::machine()
{
  return __mpMachine;
}

}// namespace
#endif /* _TESTING_STATEMACHINE_HPP_4F7967EF_ */
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim
