/**
 * @author Marko Mahnič
 * @created April 2012
 */
#include "y4learning.hpp"

namespace testing { 

class CstStart: public CState, public CMachineStateMixin<CCastMachine>
{
private:
  CLinkedStatePtr mTableEmpty;
public:
  CstStart(CCastMachine* pMachine)
    : CState(pMachine, "Start"),
    CMachineStateMixin(pMachine),
    mTableEmpty(linkedState("TableEmpty"))
  {
    setSleepTime(20, 30 * 1000);
    setWatchEvents({ "::Video::VisualObject", "::Video::ProtoObject" });
  }
  TStateFunctionResult work() {
    machine()->switchToState(mTableEmpty);
    return Continue;
  }
};

#if 0
CStatePtr CY4Learning::stStart(CCastMachine* pMachine)
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
#endif

class CstTableEmpty: public CState, public CMachineStateMixin<CCastMachine>
{
private:
  CLinkedStatePtr mFinished;
  CLinkedStatePtr mWaitToAppear;
public:
  CstTableEmpty(CCastMachine* pMachine)
    : CState(pMachine, "TableEmpty"),
    CMachineStateMixin(pMachine),
    mFinished(linkedState("Finished")),
    mWaitToAppear(linkedState("WaitToAppear"))
  {
    setSleepTime(20);
    // TODO pEmpty->setTimeout()
    setWatchEvents({ "::Video::VisualObject", "::Video::ProtoObject" });
  }

  TStateFunctionResult enter() {
    machine()->verifyCount("VisualObject", 0);
    if (machine()->getCount("VisualObject") > 0) {
      return WaitChange;
    }
    return Continue;
  }

  TStateFunctionResult work() {
    if (machine()->getCount("VisualObject") < 1) {
      if (machine()->nextScene()) {
        machine()->loadScene();
        machine()->switchToState(mWaitToAppear);
      }
      else {
        machine()->switchToState(mFinished);
      }
      return Continue;
    }
    return WaitChange;
  }
};

#if 0
CStatePtr CY4Learning::stTableEmpty(CCastMachine* pMachine)
{
  auto pEmpty = pMachine->addState("TableEmpty");
  pEmpty->setSleepTime(20);
  // TODO pEmpty->setTimeout()
  pEmpty->setWatchEvents({ "::Video::VisualObject", "::Video::ProtoObject" });
  auto stFinished = pEmpty->linkedState("Finished");
  auto stWait = pEmpty->linkedState("WaitToAppear");

  auto fnEnter = [=](CState* pState) {
    pMachine->verifyCount("VisualObject", 0);
    if (pMachine->getCount("VisualObject") > 0) {
      return WaitChange;
    }
    return Continue;
  };
  pEmpty->setEnter(fnEnter);

  auto fnWork = [=, &mSceneId](CState* pState) {
    if (pMachine->getCount("VisualObject") < 1) {
      ++mSceneId;
      if (pMachine->loadScene(mSceneId)) {
        pMachine->switchToState(stWait);
      }
      else {
        pMachine->switchToState(stFinished);
      }
      return Continue;
    }
    return WaitChange;
  };
  pEmpty->setWork(fnWork);

  return pEmpty;
}
#endif

class CstWaitToAppear: public CState, public CMachineStateMixin<CCastMachine>
{
private:
  CLinkedStatePtr mStartTeach;
public:
  CstWaitToAppear(CCastMachine* pMachine)
    : CState(pMachine, "WaitToAppear"),
    CMachineStateMixin(pMachine),
    mStartTeach(linkedState("StartTeaching"))
  {
    setSleepTime(20);
    // TODO pWaitAppear->setTimeout()
    setWatchEvents({ "::Video::VisualObject", "::Video::ProtoObject" });
  }
  TStateFunctionResult work() {
    if (machine()->getCount("VisualObject") < 1) {
      return WaitChange;
    }
    machine()->switchToState(mStartTeach);
    return Continue;
  }
};

#if 0
CStatePtr CY4Learning::stWaitToAppear(CCastMachine* pMachine)
{
  auto pWaitAppear = pMachine->addState("WaitToAppear");
  pWaitAppear->setSleepTime(20);
  // TODO pWaitAppear->setTimeout()
  pWaitAppear->setWatchEvents({ "::Video::VisualObject", "::Video::ProtoObject" });
  auto stStartTeach = pWaitAppear->linkedState("StartTeaching");

  auto fnWork = [=] (CState* pState) {
    if (pMachine->getCount("VisualObject") < 1) {
      return WaitChange;
    }
    pMachine->switchToState(stStartTeach);
    return Continue;
  };
  pWaitAppear->setWork(fnWork);

  return pWaitAppear;
}
#endif

class CstStartTeach: public CState, public CMachineStateMixin<CCastMachine>
{
private:
  CLinkedStatePtr mTeachStep;
public:
  CstStartTeach(CCastMachine* pMachine)
    : CState(pMachine, "StartTeaching"),
    CMachineStateMixin(pMachine),
    mTeachStep(linkedState("TeachOneStep"))
  {
  }
  TStateFunctionResult work() {
    machine()->mTeachingStep = 0;
    machine()->mStepsTaught = 0;
    machine()->switchToState(mTeachStep);
    return Continue;
  }
};

class CstTeachOneStep: public CState, public CMachineStateMixin<CCastMachine>
{
private:
  CLinkedStatePtr mWaitResponse;
  CLinkedStatePtr mEndTeach;
public:
  CstTeachOneStep(CCastMachine* pMachine)
    : CState(pMachine, "TeachOneStep"),
    CMachineStateMixin(pMachine),
    mWaitResponse(linkedState("WaitResponse", "LessonSpoken")),
    mEndTeach(linkedState("EndTeaching", "NoMoreLessons,Timeout"))
  {
    setWatchEvents({ "::Video::VisualObject", "::Video::ProtoObject" });
    setTimeout(30 * 1000);
  }
  TStateFunctionResult work() {
    machine()->verifyCount("VisualObject", 1);
    if (machine()->getCount("VisualObject") != 1) {
      if (hasTimedOut()) {
        machine()->reportTimeout("Waiting for VisualObject count==1");
        machine()->switchToState(mEndTeach);
        return Continue;
      }
      return WaitChange;
    }
    // TODO: pMachine->clearSpokenItems()
    if (machine()->sayLesson(machine()->mTeachingStep)) {
      machine()->switchToState(mWaitResponse);
    }
    else {
      machine()->switchToState(mEndTeach);
    }
    return Continue;
  }
};

class CstWaitResponse: public CState, public CMachineStateMixin<CCastMachine>
{
private:
  CLinkedStatePtr mTeachStep;
  CLinkedStatePtr mEndTeach;
public:
  CstWaitResponse(CCastMachine* pMachine)
    : CState(pMachine, "WaitResponse"),
    CMachineStateMixin(pMachine),
    mTeachStep(linkedState("TeachOneStep", "Confirmed")),
    mEndTeach(linkedState("EndTeaching", "Timeout"))
  {
    setWatchEvents({ "synthesize::SpokenOutputItem" });
    setTimeout(60 * 1000);
    setSleepTime(20);
  }
  TStateFunctionResult enter() {
    return WaitChange;
  }
  TStateFunctionResult work() {
    if (hasTimedOut()) {
      machine()->reportTimeout("Waiting for Robot Response");
      machine()->switchToState(mEndTeach);
      return Continue;
    }
    // TODO WaitResponse:  if (pMachine->robotSaidOk()) {
    //    ++mStepsTaught;
    //    pMachine->switchToState(mTeachStep);
    // else {
    //    return WaitChange;
    // }
    return Continue;
  }
};

class CstEndTeach: public CState, public CMachineStateMixin<CCastMachine>
{
private:
  CLinkedStatePtr mTableEmpty;
public:
  CstEndTeach(CCastMachine* pMachine)
    : CState(pMachine, "EndTeaching"),
    CMachineStateMixin(pMachine),
    mTableEmpty(linkedState("TableEmpty"))
  {
  }
  TStateFunctionResult work() {
    // TODO EndTeaching: pMachine->clearScene();
    // TODO if (mStepsTaught > 0) pMachine->saveKnowledge();
    machine()->switchToState(mTableEmpty);
    return Continue;
  }
};

// The sub-automaton for teaching the robot the properties of an object.
CStatePtr CY4Learning::stTeach(CCastMachine* pMachine)
{
#if 0
  auto pStartTeach = pMachine->addState("StartTeaching");
  auto stTeachStep = pStartTeach->linkedState("TeachOneStep");
  auto fnWork = [=, &mTeachingStep, &mStepsTaught] (CState* pState) {
    mTeachingStep = 0;
    mStepsTaught = 0;
    pMachine->switchToState(stTeachStep);
    return Continue;
  };
  pStartTeach->setWork(fnWork);
#endif


#if 0
  auto pTeach = pMachine->addState("TeachOneStep");
  pTeach->setWatchEvents({ "::Video::VisualObject", "::Video::ProtoObject" });
  pTeach->setTimeout(30 * 1000);
  auto stEndTeach = pTeach->linkedState("EndTeaching", "NoMoreSteps,Timeout");
  auto stWaitResponse = pTeach->linkedState("WaitResponse", "Verbalized");
  auto fnTeachWork = [=, &mTeachingStep] (CState* pState) {
    pMachine->verifyCount("VisualObject", 1);
    if (pMachine->getCount("VisualObject") != 1) {
      if (pState->hasTimedOut()) {
        pMachine->reportTimeout("Waiting for VisualObject count==1");
        pMachine->switchToState(stEndTeach);
        return Continue;
      }
      return WaitChange;
    }
    // TODO: pMachine->clearSpokenItems()
    if (pMachine->sayLesson(mTeachingStep)) {
      pMachine->switchToState(stWaitResponse);
    }
    else {
      pMachine->switchToState(stEndTeach);
    }
    return Continue;
  };
  pTeach->setWork(fnTeachWork);
#endif


#if 0
  auto pWaitResponse = pMachine->addState("WaitResponse");
  pWaitResponse->linkedState("TeachOneStep", "Confirmed,Timeout");
  pWaitResponse->setWatchEvents({ "synthesize::SpokenOutputItem" });
  pWaitResponse->setTimeout(60 * 1000);
  auto fnWaitRespEnter = [=](CState* pState) {
    return WaitChange;
  };
  pWaitResponse->setEnter(fnWaitRespEnter);

  auto fnWaitRespWork = [=, &mStepsTaught](CState* pState) {
    if (pState->hasTimedOut()) {
      pMachine->reportTimeout("Waiting for Robot Response");
      pMachine->switchToState(stEndTeach);
      return Continue;
    }
    // TODO WaitResponse:  if (pMachine->robotSaidOk()) {
    //    ++mStepsTaught;
    //    pMachine->switchToState(stTeachStep);
    // else {
    //    return WaitChange;
    // }
    return Continue;
  };
  pWaitResponse->setWork(fnWaitRespWork);
#endif


#if 0
  auto pEndTeach = pMachine->addState("EndTeaching");
  auto stTableEmpty = pEndTeach->linkedState("TableEmpty");
  auto fnEndTeachWork = [=, &mStepsTaught](CState* pState) {
    // TODO EndTeaching: pMachine->clearScene();
    // TODO if (mStepsTaught > 0) pMachine->saveKnowledge();
    pMachine->switchToState(stTableEmpty);
    return Continue;
  };

  return pStartTeach;
#endif
}

CMachinePtr CY4Learning::init()
{
  TStateFunction fnEnter, fnWork, fnExit;
  CCastMachine* pMachine = new CCastMachine();

  auto pFinish = pMachine->addState("Finished");
  auto pStart = stStart(pMachine);
  //stTableEmpty(pMachine);
  //stWaitToAppear(pMachine);
  //stTeach(pMachine);

  pMachine->switchToState(pStart);
  return CMachinePtr(pMachine);
}

} //namespace
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim
