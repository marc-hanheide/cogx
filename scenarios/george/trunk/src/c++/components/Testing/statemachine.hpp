/**
 * @author Marko Mahnič
 * @created April 2012
 */
#ifndef _TESTING_STATEMACHINE_HPP_4F7967EF_
#define _TESTING_STATEMACHINE_HPP_4F7967EF_

#include <castutils/Timers.hpp>

#include <cassert>
#include <stdexcept>
#include <memory>
#include <string>
#include <sstream>
#include <map>
#include <functional>
#include <mutex>

namespace testing
{

class CMachine;
typedef std::shared_ptr<CMachine> CMachinePtr;

class CState;
typedef std::shared_ptr<CState> CStatePtr;

class CLinkedState;
typedef std::shared_ptr<CLinkedState> CLinkedStatePtr;

// The state can contain varios fields with arbitrary infromation that can be
// used for processing.
typedef std::map<std::string, std::string> TStateInfoMap;

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
    acActive, acSleeping, acWaiting, acWaitInt, acDone
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
  std::string msExitReason;
  TStateInfoMap mInitInfo; // this is copied to mStateInfo in phInit/restart().

protected:
  TStateInfoMap mInfo;

private:
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
  void setSleepTime(long workingMs, long firstTimeMs=0);
  void setTimeout(long milliseconds);
  void setWatchEvents(const std::vector<std::string>& events);

  // Can be used to transfer some info between states
  void setInfo(const TStateInfoMap& info);

  CLinkedStatePtr linkedState(const std::string& stateName, const std::string& reasons="");
  // TODO: std::vector<CLinkedStatePtr> getBadLinks();

  bool isWaitingForEvent(const std::string& evnet);
  bool hasTimedOut();

  virtual std::string getProgressText();
private:
  void restart();
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
  CStatePtr mpToState;
  CLinkedState(CMachine* pMachine, CState* pFromState, const std::string& toStateName, const std::string& reasons);
public:
  CStatePtr getState();
  std::string description();
};

class CMachine
{
private:
  std::map<std::string, CStatePtr> mStates;
  CStatePtr mpPrevState;
  CStatePtr mpCurrentState;
  CStatePtr mpNextState;
  castutils::CMilliTimer mWaitTimer; // periodically activate even if there are no events
  long mStepNumber;

protected:
  std::mutex mReceivedEventsMutex;
  std::map<std::string, bool> mReceivedEvents;
  void checkReceivedEvent(const std::string& event);

public:
  CStatePtr addState(std::string id);
  CStatePtr addState(CState* pState);
  CStatePtr findState(std::string id);
  void switchToState(CStatePtr pNextState, const std::string& reason = "");
  void switchToState(CLinkedStatePtr pNextState, const std::string& reason = "");
  void runOneStep();
  virtual void onTransition(CStatePtr fromState, CStatePtr toState);
  bool isWaitingForEvent();
  bool isFinished();
  bool isSwitching();
  void checkEvents();
  long timeToSleep();
  long getStepNumber();
  virtual void writeStateDescription(std::ostringstream& ss);
  virtual void writeMachineDescription(std::ostringstream& ss);
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
void CState::setSleepTime(long workingMs, long firstTimeMs)
{
  mSleepMs = workingMs;
  mFirstSleepMs = firstTimeMs;
}

inline
void CState::setTimeout(long milliseconds)
{
  mTimeoutMs = milliseconds;
}

inline
void CState::setWatchEvents(const std::vector<std::string>& events)
{
  mEvents = events;
}

inline
void CState::setInfo(const TStateInfoMap& info)
{
  mInitInfo = info;
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
CStatePtr CLinkedState::getState()
{
  if (!mpToState) {
    mpToState = mpMachine->findState(mId);
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
    throw std::runtime_error((std::string("State not found: ") + id));
    return CStatePtr();
  }
  return mStates[id];
}

inline
void CMachine::switchToState(CLinkedStatePtr pNextState, const std::string& reason)
{
  switchToState(pNextState->getState(), reason);
}

inline
long CMachine::getStepNumber()
{
  return mStepNumber;
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
bool CMachine::isSwitching()
{
  if (! mpCurrentState.get()) return true;
  return (mpCurrentState->mbSwitchStateCalled);
}

inline
void CMachine::checkReceivedEvent(const std::string& event)
{
  std::lock_guard<std::mutex> lock(mReceivedEventsMutex);
  mReceivedEvents[event] = true;
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
