/**
 * @author Marko Mahnič
 * @created April 2012
 */
#include "objectcount.hpp"

namespace testing { 

CStatePtr CObjectCountTest::stStart(CCastMachine* pMachine)
{
  auto pStart = pMachine->addState("Start");
  pStart->setSleepTime(20, 30 * 1000);
  pStart->setWatchEvents({ "::Video::VisualObject", "::Video::ProtoObject" });
  auto stEmpty = pStart->linkedState("TableEmpty");

  auto fnWork = [=](CState* pState) {
    pMachine->switchToState(stEmpty);
    return Continue;
  };
  pStart->setWork(fnWork);
  return pStart;
}

CStatePtr CObjectCountTest::stTableEmpty(CCastMachine* pMachine)
{
  auto pEmpty = pMachine->addState("TableEmpty");
  pEmpty->setSleepTime(20);
  pEmpty->setWatchEvents({ "::Video::VisualObject", "::Video::ProtoObject" });
  auto stFinished = pEmpty->linkedState("Finished");
  auto stWait = pEmpty->linkedState("WaitToAppear");

  auto fnEnter = [=](CState* pState) {
    pMachine->verifyCount(0);
    if (pMachine->mObjectCount > 0) {
      return WaitChange;
    }
    return Continue;
  };
  pEmpty->setEnter(fnEnter);

  auto fnWork = [=](CState* pState) {
    if (pMachine->mObjectCount < 1) {
      if (! pMachine->getCurrentTest()) {
        pMachine->switchToState(stFinished);
      }
      else {
        pMachine->loadScene();
        pMachine->switchToState(stWait);
      }
      return Continue;
    }
    return WaitChange;
  };
  pEmpty->setWork(fnWork);

  return pEmpty;
}

CStatePtr CObjectCountTest::stWaitToAppear(CCastMachine* pMachine)
{
  auto pWaitAppear = pMachine->addState("WaitToAppear");
  pWaitAppear->setSleepTime(20);
  pWaitAppear->setWatchEvents({ "::Video::VisualObject", "::Video::ProtoObject" });

  auto fnWork = [=] (CState* pState) {
    if (pMachine->mObjectCount < 1) {
      return WaitChange;
    }
    return Continue;
  };
  pWaitAppear->setWork(fnWork);

  return pWaitAppear;
}

CMachinePtr CObjectCountTest::init()
{
  TStateFunction fnEnter, fnWork, fnExit;
  CCastMachine* pMachine = new CCastMachine();

  auto pFinish = pMachine->addState("Finished");
  auto pStart = stStart(pMachine);
  stTableEmpty(pMachine);
  stWaitToAppear(pMachine);

  pMachine->switchToState(pStart);
  return CMachinePtr(pMachine);
}

} //namespace
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim
