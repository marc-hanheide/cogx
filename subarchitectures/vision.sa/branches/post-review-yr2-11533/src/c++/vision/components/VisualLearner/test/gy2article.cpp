#include "gy2article.h"

#include <VisionData.hpp>
#include <dialogue.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

using namespace std;
using namespace cast;
using namespace VisionData;
using namespace de::dfki::lt::tr::dialogue::slice;

CGeorgeY2Article::CGeorgeY2Article(string name, CTestRecognizer *pOwner)
   : CTestCase(name, pOwner)
{
   m_State = stTableEmpty;
   m_ObjectCount = 0;
   m_RobotResponse = "";
}

void CGeorgeY2Article::onStart()
{
   m_pOwner->addChangeFilter(
      createLocalTypeFilter<VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<CGeorgeY2Article>(
         this, &CGeorgeY2Article::onAdd_VisualObject)
      );
   m_pOwner->addChangeFilter(
      createLocalTypeFilter<VisualObject>(cdl::DELETE),
      new MemberFunctionChangeReceiver<CGeorgeY2Article>(
         this, &CGeorgeY2Article::onDel_VisualObject)
      );
   m_pOwner->addChangeFilter(
      createGlobalTypeFilter<synthesize::SpokenOutputItem>(cdl::ADD),
      new MemberFunctionChangeReceiver<CGeorgeY2Article>(
         this, &CGeorgeY2Article::onAdd_SpokenItem)
      );
}

void CGeorgeY2Article::onAdd_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_ObjectCount++;
   }
   m_EventMonitor.notify();
}

void CGeorgeY2Article::onDel_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_ObjectCount--;
   }
   m_EventMonitor.notify();
}

void CGeorgeY2Article::onAdd_SpokenItem(const cast::cdl::WorkingMemoryChange & _wmc)
{
   // TODO: read the response, verify that it's the robots response, notify
   if (false)
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
      m_RobotResponse = "I see";
   }
   m_EventMonitor.notify();
}

//void CGeorgeY2Article::issueRequest()
//{
//   // TODO: Create a new request for learning
//   //    - load an empty sequence into the SequenceServer
//   //    - wait till VO disappears
//   //    - load next sequence
//   //    - wait till VO appears
//   //    - write utterance to WM
//   //    -   wait until the robot responds
//   //    -   repeat with next utterance
//   //    - end-of-learning

//   //ProtoObjectPtr pproto = m_pOwner->loadFakeProtoObject();
//   //cdl::WorkingMemoryAddress addr;
//   //addr.subarchitecture = string(m_pOwner->getSubarchitectureID());
//   //addr.id = m_pOwner->newDataID();
//   //m_pOwner->addToWorkingMemory(addr, pproto);

//   //m_pOwner->println("Adding new VisualLearnerRecognitionTask");
//   //VisualLearnerRecognitionTaskPtr ptask = new VisualLearnerRecognitionTask();
//   //ptask->protoObjectAddr = addr;

//   //string reqId(m_pOwner->newDataID());
//   //m_pOwner->addToWorkingMemory(reqId, ptask);
//}

void CGeorgeY2Article::switchState(int newState)
{
   if (newState == m_State) return;

   if (newState == stTeaching) {
      if (m_State == stObjectOn) {
         m_TeachingStep = 0;
      }
   }

   if (newState == stWaitForResponse) {
      m_RobotResponse = "";
   }

   m_State = newState;
}

void CGeorgeY2Article::verifyCount(int count)
{
   if (count == 0) {
      if (m_ObjectCount != 0) {
         m_pOwner->println("The scene should be empty, but I see %d objects.", m_ObjectCount);
      }
      return;
   }
   if (count == 1) {
      if (m_ObjectCount != 1) {
         m_pOwner->println("There should be one object on the scene, but I see %d.", m_ObjectCount);
      }
      return;
   }
}

void CGeorgeY2Article::loadNextObject()
{
}

void CGeorgeY2Article::loadEmptyScene()
{
}

bool CGeorgeY2Article::performNextTeachingStep()
{
   m_TeachingStep++;
   if (m_TeachingStep > 1) return false;

   // TODO: retreive the learning step from script.

   cdl::WorkingMemoryAddress addr;
   addr.subarchitecture = string("dialogue"); // XXX: Hardcoded SA name, has to match SA in .cast file
   addr.id = m_pOwner->newDataID();

   asr::PhonStringPtr sayWhat = new asr::PhonString();
   sayWhat->id = addr.id;
   sayWhat->wordSequence = "hello george";

   m_pOwner->addToWorkingMemory(addr, sayWhat);

   return true;
}

void CGeorgeY2Article::runOneStep()
{
   // SYNC: Lock the monitor
   IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_EventMonitor);
   bool mustWait = false;
   mustWait = mustWait
      || (m_State == stTableEmpty && m_ObjectCount > 0) // ghosts
      || (m_State == stWaitToAppear && m_ObjectCount < 1)
      || (m_State == stTeaching && m_ObjectCount < 1)   // object unstable
      || (m_State == stWaitToDisappear && m_ObjectCount > 0)
      || (m_State == stWaitForResponse && m_RobotResponse.length() < 1);

   // SYNC: Unlock the monitor and wait for notify() or timeout
   if (mustWait)
      m_EventMonitor.timedWait(IceUtil::Time::seconds(2));
   else 
      m_EventMonitor.timedWait(IceUtil::Time::milliSeconds(2)); // give a chance to other thread, anyway
   // SYNC: Continue with a locked monitor

   switch (m_State) {
      case stTableEmpty:
         verifyCount(0);
         if (m_ObjectCount < 1) {
            loadNextObject();
            switchState(stWaitToAppear);
         }
         break;

      case stWaitToAppear:
         if (m_ObjectCount > 0) {
            switchState(stObjectOn);
         }
         break;

      case stObjectOn:
         verifyCount(1);
         switchState(stTeaching);
         break;

      // While there are teaching steps to perform, change between stTeaching and stWaitForResponse.
      // After last step, (remove the object video sequence and) change to stEndOfTeaching
      case stTeaching:
         verifyCount(1);
         if (performNextTeachingStep())
            switchState(stWaitForResponse); // TODO: subscribe to robot utterance (SpokenOutputItem WM entry)
         else
            switchState(stEndOfTeaching);
         break;

      case stWaitForResponse: // "I see" or "Thank you" or ...
         verifyCount(1);
         if (m_RobotResponse.length() > 0) {
            switchState(stTeaching);
         }
         // TODO: what about some timeout?
         break;

      case stEndOfTeaching:
         loadEmptyScene();
         switchState(stWaitToDisappear);
         break;

      case stWaitToDisappear:
         if (m_ObjectCount <= 0) {
            switchState(stTableEmpty);
         }
         break;
   }
}
// vim: set sw=3 ts=8 et :vim
