/**
 * @author Marko Mahnič
 * @created April 2012
 */
#include "y4learning.hpp"
#include <cstdlib>

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
  CLinkedStatePtr mSelf;
  CLinkedStatePtr mFinished;
  CLinkedStatePtr mLoadScene;
  bool mSceneFound;
public:
  CstTableEmpty(CCastMachine* pMachine)
    : CState(pMachine, "TableEmpty"),
    CMachineStateMixin(pMachine),
    mSelf(linkedState("TableEmpty")),
    mFinished(linkedState("Finished")),
    mLoadScene(linkedState("LoadScene"))
  {
    setSleepTime(20);
    setTimeout(90 * 1000);
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
    if (machine()->getCount("VisualObject") < 1 && machine()->getCount("ProtoObject") < 1) {
      if (mSceneFound) {
        machine()->switchToState(mLoadScene, "table-empty");
      }
      else {
        machine()->switchToState(mFinished, "no-next-scene");
      }
      return Continue;
    }
    if (hasTimedOut()) {
      // TODO: should we try to turn the head?
      machine()->switchToState(mSelf, "timeout");
    }
    return WaitChange;
  }
};

class CstLoadScene: public CState, public CMachineStateMixin<CCastMachine>
{
  CLinkedStatePtr mWaitToAppear;
public:
  CstLoadScene(CCastMachine* pMachine)
    : CState(pMachine, "LoadScene"),
    CMachineStateMixin(pMachine),
    mWaitToAppear(linkedState("WaitToAppear"))
  {
    setSleepTime(20, 1 * 1000);
  }
  TStateFunctionResult work() {
    machine()->loadScene();
    machine()->switchToState(mWaitToAppear, "scene-loaded");
    return Continue;
  }
};

class CstWaitToAppear: public CState, public CMachineStateMixin<CCastMachine>
{
private:
  CLinkedStatePtr mStartTeach;
  CLinkedStatePtr mTableEmpty;
  CLinkedStatePtr mWaitRecogTask;
public:
  CstWaitToAppear(CCastMachine* pMachine)
    : CState(pMachine, "WaitToAppear"),
    CMachineStateMixin(pMachine),
    mWaitRecogTask(linkedState("WaitRecognitionTask")),
    mStartTeach(linkedState("StartTeaching")),
    mTableEmpty(linkedState("TableEmpty"))
  {
    setSleepTime(20);
    setTimeout(30 * 1000);
    setWatchEvents({ "::VisionData::VisualObject", "::VisionData::ProtoObject" });
  }
  TStateFunctionResult work() {
    if (hasTimedOut()) {
      machine()->clearScene();
      machine()->switchToState(mTableEmpty, "timeout");
    }
    if (machine()->getCount("VisualObject") < 1) {
      return WaitChange;
    }
    //machine()->switchToState(mStartTeach, "I-see-VO");
    machine()->switchToState(mWaitRecogTask, "I-see-VO");
    return Continue;
  }
};

class CstWaitRecognitionTask: public CState, public CMachineStateMixin<CCastMachine>
{
  CLinkedStatePtr mStartTeach;
public:
  CstWaitRecognitionTask(CCastMachine* pMachine)
    : CState(pMachine, "WaitRecognitionTask"),
    CMachineStateMixin(pMachine),
    mStartTeach(linkedState("StartTeaching"))
  {
    setSleepTime(20, 3 * 1000);
  }
  TStateFunctionResult work() {
    // TODO: should actually wait for RecognitionTask; now we just sleep a while
    machine()->switchToState(mStartTeach);
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
    setSleepTime(20, 2 * 1000);
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
  CLinkedStatePtr mWaitLearningTask;
  CLinkedStatePtr mEndTeach;
  CLinkedStatePtr mSelf; // this will probably cause a memory leak
public:
  CstTeachOneStep(CCastMachine* pMachine)
    : CState(pMachine, "TeachOneStep"),
    CMachineStateMixin(pMachine),
    //mWaitResponse(linkedState("WaitResponse", "LessonSpoken")),
    mWaitLearningTask(linkedState("WaitLearningTask", "LessonSpoken")),
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

    TStateInfoMap info;
    info["LearningTask-add"] = std::to_string(machine()->getCount("LearningTask-add"));
    info["LearningTask-done"] = std::to_string(machine()->getCount("LearningTask-done"));
    mWaitLearningTask->getState()->setInfo(info);

    if (machine()->sayLesson()) {
      //machine()->switchToState(mWaitResponse, "lesson-spoken");
      machine()->switchToState(mWaitLearningTask, "lesson-spoken");
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

#if 0
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
#endif

class CstWaitLearningTask: public CState, public CMachineStateMixin<CCastMachine>
{
protected:
  CLinkedStatePtr mTeachStep;
  CLinkedStatePtr mEndTeach;
  CLinkedStatePtr mSelf;
  CLinkedStatePtr mWLearnTaskComplete;
  long mPrevTaskAddCount, mPrevTaskDoneCount;
  std::string mDebug;

  void initLinkedStates()
  {
    mTeachStep = linkedState("TeachOneStep", "Confirmed");
    mWLearnTaskComplete = linkedState("WaitLearningTaskComplete", "TaskCreated");
    mEndTeach = linkedState("EndTeaching", "Timeout");
  }

  virtual std::string getProgressText()
  {
    return mDebug;
  }

public:
  CstWaitLearningTask(CCastMachine* pMachine, const std::string name="WaitLearningTask")
    : CState(pMachine, name),
    CMachineStateMixin(pMachine)
  {
    initLinkedStates();
    setWatchEvents({ "::VisionData::VisualLearningTask" });
    setTimeout(30 * 1000);
    setSleepTime(20);
  }

  TStateFunctionResult enter() {
    mDebug = "";
    mPrevTaskAddCount = std::strtol(mInfo["LearningTask-add"].c_str(), nullptr, 10);
    mPrevTaskDoneCount = std::strtol(mInfo["LearningTask-done"].c_str(), nullptr, 10);
    return Continue;
  }

  void exitLesson(const std::string& reason)
  {
    if (machine()->hasMoreLessons()) {
      machine()->switchToState(mTeachStep, 2000, reason);
    }
    else {
      machine()->switchToState(mEndTeach, reason + ",no-more-lessons");
    }
  }

  TStateFunctionResult work() {
    long cntAdd = machine()->getCount("LearningTask-add");

    if (cntAdd != mPrevTaskAddCount) {
      mWLearnTaskComplete->getState()->setInfo(mInfo);
      machine()->switchToState(mWLearnTaskComplete, "task-created");
      return Continue;
    }

    if (hasTimedOut()) {
      machine()->reportTimeout("Waiting for Learning Task");
      exitLesson("timeout");
      return Continue;
    }

    return WaitChange;
  }
};

class CstWaitLearningTaskComplete: public CstWaitLearningTask
{
public:
  CstWaitLearningTaskComplete(CCastMachine* pMachine)
    : CstWaitLearningTask(pMachine, "WaitLearningTaskComplete")
  {
  }

  TStateFunctionResult work() {
    long cntDone = machine()->getCount("LearningTask-done");

    if (cntDone != mPrevTaskDoneCount) {
      exitLesson("task-completed");
      return Continue;
    }

    if (hasTimedOut()) {
      machine()->reportTimeout("Waiting for Learning Task Completion");
      exitLesson("timeout");
      return Continue;
    }

    return WaitChange;
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
  pMachine->addState(new CstTableEmpty(pMachine));
  pMachine->addState(new CstLoadScene(pMachine));
  pMachine->addState(new CstWaitToAppear(pMachine));
  pMachine->addState(new CstWaitRecognitionTask(pMachine));
  pMachine->addState(new CstStartTeach(pMachine));
  pMachine->addState(new CstTeachOneStep(pMachine));
  //pMachine->addState(new CstWaitResponse(pMachine));
  pMachine->addState(new CstWaitLearningTask(pMachine));
  pMachine->addState(new CstWaitLearningTaskComplete(pMachine));
  pMachine->addState(new CstEndTeach(pMachine));

  pMachine->switchToState(pStart);
  return CMachinePtr(pMachine);
}

} //namespace
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim
