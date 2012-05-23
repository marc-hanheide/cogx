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
    setWatchEvents({ "::VisionData::VisualObject", "::VisionData::ProtoObject" });
  }
  TStateFunctionResult work() {
    machine()->switchToState(mTableEmpty);
    return Continue;
  }
};

class CstTableEmpty: public CState, public CMachineStateMixin<CCastMachine>
{
private:
  CLinkedStatePtr mFinished;
  CLinkedStatePtr mWaitToAppear;
  bool mSceneFound;
public:
  CstTableEmpty(CCastMachine* pMachine)
    : CState(pMachine, "TableEmpty"),
    CMachineStateMixin(pMachine),
    mFinished(linkedState("Finished")),
    mWaitToAppear(linkedState("WaitToAppear"))
  {
    setSleepTime(20);
    // TODO pEmpty->setTimeout()
    setWatchEvents({ "::VisionData::VisualObject", "::VisionData::ProtoObject" });
  }

  TStateFunctionResult enter() {
    machine()->clearScene();
    mSceneFound = machine()->nextScene();
    machine()->verifyCount("VisualObject", 0);
    if (machine()->getCount("VisualObject") != 0) {
      return WaitChange;
    }
    return Continue;
  }

  TStateFunctionResult work() {
    if (machine()->getCount("VisualObject") < 1) {
      if (mSceneFound) {
        machine()->loadScene();
        machine()->switchToState(mWaitToAppear, "scene-loaded");
      }
      else {
        machine()->switchToState(mFinished, "no-next-scene");
      }
      return Continue;
    }
    return WaitChange;
  }
};

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
    setWatchEvents({ "::VisionData::VisualObject", "::VisionData::ProtoObject" });
  }
  TStateFunctionResult work() {
    if (machine()->getCount("VisualObject") < 1) {
      return WaitChange;
    }
    machine()->switchToState(mStartTeach, "I-see-VO");
    return Continue;
  }
};

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
    //setSleepTime(20, 30 * 1000); // TODO: initial timeout could be configurable with params!
    setSleepTime(20, 5 * 1000);
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
  CLinkedStatePtr mSelf; // this will probably cause a memory leak
public:
  CstTeachOneStep(CCastMachine* pMachine)
    : CState(pMachine, "TeachOneStep"),
    CMachineStateMixin(pMachine),
    mWaitResponse(linkedState("WaitResponse", "LessonSpoken")),
    mEndTeach(linkedState("EndTeaching", "NoMoreLessons,Timeout")),
    mSelf(linkedState("TeachOneStep", "SkippedLesson"))
  {
    setWatchEvents({ "::VisionData::VisualObject", "::VisionData::ProtoObject" });
    setTimeout(30 * 1000);
  }

  TStateFunctionResult enter() {
    machine()->nextLesson();
    return Continue;
  }

  TStateFunctionResult work() {
    machine()->verifyCount("VisualObject", 1);
    if (machine()->getCount("VisualObject") != 1) {
      if (hasTimedOut()) {
        machine()->reportTimeout("Waiting for VisualObject count==1");
        machine()->switchToState(mEndTeach, "timeout");
        return Continue;
      }
      return WaitChange;
    }

    if (machine()->sayLesson()) {
      machine()->switchToState(mWaitResponse, "lesson-spoken");
    }
    else if (machine()->hasMoreLessons()) {
      machine()->switchToState(mSelf, "skipped-lesson");
    }
    else {
      machine()->switchToState(mEndTeach, "no-more-lessons");
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
    machine()->log("WaitResponse work");
    if (hasTimedOut()) {
      machine()->reportTimeout("Waiting for Robot Response");
      machine()->switchToState(mTeachStep, "timeout");
      return Continue;
    }

    long resp = machine()->getRobotAnswerClass();
    if (resp == 1) { // TODO: constant YES
       ++machine()->mStepsTaught;
       machine()->switchToState(mTeachStep);
    }
    else if (resp == 2) { // TODO: constant NO
       machine()->switchToState(mTeachStep);
    }
    else { // something else or no response
       return WaitChange;
    }
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
    machine()->clearScene();
    if (machine()->mStepsTaught > 0) {
      // TODO machine()->saveKnowledge();
    }
    machine()->switchToState(mTableEmpty);
    return Continue;
  }
};

CMachinePtr CY4Learning::createMachine(CTester* pOwner)
{
  TStateFunction fnEnter, fnWork, fnExit;
  CCastMachine* pMachine = new CCastMachine(pOwner);
  //pMachine->castInit(pOwner);

  auto pFinish = pMachine->addState("Finished");
  auto pStart = pMachine->addState(new CstStart(pMachine));
  pMachine->addState(new CstWaitToAppear(pMachine));
  pMachine->addState(new CstTableEmpty(pMachine));
  pMachine->addState(new CstStartTeach(pMachine));
  pMachine->addState(new CstTeachOneStep(pMachine));
  pMachine->addState(new CstWaitResponse(pMachine));
  pMachine->addState(new CstEndTeach(pMachine));

  pMachine->switchToState(pStart);
  return CMachinePtr(pMachine);
}

} //namespace
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim
