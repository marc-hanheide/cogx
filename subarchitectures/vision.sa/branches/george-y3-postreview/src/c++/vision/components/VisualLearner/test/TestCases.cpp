#include <opencv/cv.h>
#include <opencv/highgui.h>

// cast
#include <cast/architecture/ChangeFilterFactory.hpp>

#include "TestCases.h"

using namespace std;
using namespace cast;
using namespace VisionData;

static string descAddr(const cdl::WorkingMemoryAddress &addr)
{
   string res("[" + addr.subarchitecture + "/" + addr.id + "]");
   return res;
}

//################################################################# 
// STANDALONE
//################################################################# 
CTestCase_Standalone::CTestCase_Standalone(string name, CTestRecognizer *pOwner)
   : CTestCase(name, pOwner)
{
   m_nCallsLeft = 3;
}

void CTestCase_Standalone::onStart()
{
   m_pOwner->dumpChanged_RecogTask = true;
   m_pOwner->addChangeFilter(
      createLocalTypeFilter<VisualLearnerRecognitionTask>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CTestCase_Standalone>(
         this, &CTestCase_Standalone::onChange_RecognitionTask)
      );
}

void CTestCase_Standalone::onChange_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   m_pOwner->sleepComponent(100);
   m_pOwner->deleteFromWorkingMemory(_wmc.address);
}

void CTestCase_Standalone::issueRequest()
{
   ProtoObjectPtr pproto = m_pOwner->loadFakeProtoObject();
   cdl::WorkingMemoryAddress addr;
   addr.subarchitecture = string(m_pOwner->getSubarchitectureID());
   addr.id = m_pOwner->newDataID();
   m_pOwner->addToWorkingMemory(addr, pproto);

   m_pOwner->println("Adding new VisualLearnerRecognitionTask");
   VisualLearnerRecognitionTaskPtr ptask = new VisualLearnerRecognitionTask();
   ptask->status = VCREQUESTED;
   ptask->protoObjectAddr = new cdl::WorkingMemoryPointer();
   ptask->protoObjectAddr->type = cast::typeName<ProtoObject>();
   ptask->protoObjectAddr->address = addr;

   string reqId(m_pOwner->newDataID());
   m_pOwner->addToWorkingMemory(reqId, ptask);
}

void CTestCase_Standalone::runOneStep()
{
   if (m_pOwner->m_ProtoObjects < 1) {
      if (m_nCallsLeft > 0) {
         issueRequest();
         m_nCallsLeft--;
      }
      m_pOwner->sleepComponent(500);
   }
}

//################################################################# 
// FAKE PROTO
//################################################################# 
CTestCase_FakeProto::CTestCase_FakeProto(string name, CTestRecognizer *pOwner)
   : CTestCase(name, pOwner)
{
   m_nCallsLeft = 3;
}

void CTestCase_FakeProto::onStart()
{
   m_pOwner->dumpChanged_RecogTask = true;
}

void CTestCase_FakeProto::issueRequest()
{
   ProtoObjectPtr pproto = m_pOwner->loadFakeProtoObject();
   string protoId = m_pOwner->newDataID();
   m_pOwner->addToWorkingMemory(protoId, pproto);
   m_protoIDs.push_back(protoId);
}

void CTestCase_FakeProto::runOneStep()
{
   if (m_pOwner->m_ProtoObjects < 1) {
      if (m_nCallsLeft > 0) {
         issueRequest();
         m_nCallsLeft--;
      }
      m_pOwner->sleepComponent(500);
   }

   // FIXME: the protoobjects are never deleted, but they shoudl be
   //    m_nCallsLeft is never 0, we only ad 1 protoobject and stop adding (conditions above)
   if(m_nCallsLeft <= 0 && m_protoIDs.size() > 1) {
      m_pOwner->sleepComponent(2000);
      string id = m_protoIDs.front();
      m_protoIDs.erase(m_protoIDs.begin());
      m_pOwner->deleteFromWorkingMemory(id);
   }
}

//################################################################# 
// LEARNING
//################################################################# 
CTestCase_Learning::CTestCase_Learning(string name, CTestRecognizer *pOwner)
   : CTestCase(name, pOwner)
{
   m_stepsComplete = 0;
   m_issued = 0;
}

void CTestCase_Learning::onStart()
{
   m_pOwner->addChangeFilter(
      createLocalTypeFilter<VisualLearnerRecognitionTask>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CTestCase_Learning>(
         this, &CTestCase_Learning::onChange_RecognitionTask)
      );

   m_pOwner->addChangeFilter(
     createLocalTypeFilter<VisualLearningTask>(cdl::OVERWRITE),
     new MemberFunctionChangeReceiver<CTestCase_Learning>(
        this, &CTestCase_Learning::onChange_LearningTask)
     );
}

void CTestCase_Learning::onChange_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   m_pOwner->sleepComponent(100);
   m_pOwner->deleteFromWorkingMemory(_wmc.address);
   m_stepsComplete++;
}

static
bool intLower (int i, int j)
{
   return (i<j);
}

static
bool cmpStrLower (string i, string j)
{
   return (i<j);
}

void CTestCase_Learning::onChange_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   if (m_stepsComplete == 0) {
      VisualLearnerRecognitionTaskPtr pTask = m_pOwner->getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
      std::vector<string>::iterator plabel;
      std::vector<double>::iterator pdbl;
      labels.clear();
      for( plabel = pTask->labels.begin(); plabel != pTask->labels.end(); plabel++) {
         labels.push_back(*plabel);
      }
      std::sort(labels.begin(), labels.end(), cmpStrLower);
   }

   m_pOwner->sleepComponent(100);
   m_pOwner->deleteFromWorkingMemory(_wmc.address);
   m_stepsComplete++;
}

void CTestCase_Learning::performLearningStep(int issued, string protoId)
{
   // 1. Recognize
   // 2. Update: learn
   // 3. Recognize
   // 4. Update: unlearn
   // 5. Recognize
   std::vector<int>::iterator plabel;
   int n;
   if (issued == 1 || issued == 3 || issued == 5) {
      m_pOwner->log("STEP %d: RECOGNITION", issued);
      VisualLearnerRecognitionTaskPtr ptask = new VisualLearnerRecognitionTask();
      ptask->status = VCREQUESTED;
      cdl::WorkingMemoryAddress addr;
      addr.subarchitecture = string(m_pOwner->getSubarchitectureID());
      addr.id = protoId;
      ptask->protoObjectAddr = new cdl::WorkingMemoryPointer();
      ptask->protoObjectAddr->type = cast::typeName<ProtoObject>();
      ptask->protoObjectAddr->address = addr;

      string reqId(m_pOwner->newDataID());
      m_pOwner->addToWorkingMemory(reqId, ptask);
   }
   // TODO: TestCases for learning task, replace ProtoObject with VisualObject
   //else if (issued == 2) {
   //   m_pOwner->log("STEP %d: LEARNING", issued);
   //   VisualLearningTaskPtr ptask = new VisualLearningTask();
   //   ptask->protoObjectId = protoId;
   //   std::vector<double> &distrib = ptask->distribution;

   //   double sum = 0;
   //   for( plabel = labels.begin(); plabel != labels.end(); plabel++) {
   //      ptask->labels.push_back(*plabel);
   //      n = rand() % 100;
   //      distrib.push_back(n);
   //      sum += n;
   //   }
   //   for (n = 0; n < distrib.size(); n++) distrib[n] = distrib[n] / sum;

   //   string reqId(m_pOwner->newDataID());
   //   m_pOwner->addToWorkingMemory(reqId, ptask);
   //}
   //else if (issued == 4) {
   //   m_pOwner->log("STEP %d: UNLEARNING", issued);
   //   VisualLearningTaskPtr ptask = new VisualLearningTask();
   //   ptask->protoObjectId = protoId;
   //   std::vector<double> &distrib = ptask->distribution;

   //   double sum = 0;
   //   for( plabel = labels.begin(); plabel != labels.end(); plabel++) {
   //      ptask->labels.push_back(*plabel);
   //      n = rand() % 100;
   //      distrib.push_back(n);
   //      sum += n;
   //   }
   //   for (n = 0; n < distrib.size(); n++) {
   //      distrib[n] = distrib[n] / sum;
   //      if (n % 2 == 0) distrib[n] = -distrib[n];
   //   }

   //   string reqId(m_pOwner->newDataID());
   //   m_pOwner->addToWorkingMemory(reqId, ptask);
   //}
}

void CTestCase_Learning::runOneStep()
{
   if (m_issued <= m_stepsComplete && m_issued < 5) {
      m_issued++;
      m_pOwner->log("Steps issued: %d, complete: %d", m_issued, m_stepsComplete);
      ProtoObjectPtr pproto = m_pOwner->loadFakeProtoObject();
      string protoId = m_pOwner->newDataID();
      m_pOwner->addToWorkingMemory(protoId, pproto);
      performLearningStep(m_issued, protoId);
   }
   m_pOwner->sleepComponent(500);
}

//################################################################# 
// LOAD EXAMPLES FROM DISK
//################################################################# 
CTestCase_SavedExamples::CTestCase_SavedExamples(string name, CTestRecognizer *pOwner)
   : CTestCase(name, pOwner)
{
}

void CTestCase_SavedExamples::onStart()
{
   m_pOwner->dumpChanged_RecogTask = true;
}

//void CTestCase_SavedExamples::onChange_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
//{
//   m_pOwner->sleepComponent(100);
//   m_pOwner->deleteFromWorkingMemory(_wmc.address);
//}

void CTestCase_SavedExamples::issueRequest()
{
   // TODO> Load protoobject data from disk
   //ProtoObjectPtr pproto = m_pOwner->loadFakeProtoObject();
   //string protoId = m_pOwner->newDataID();
   //m_pOwner->addToWorkingMemory(protoId, pproto);
   //m_protoIDs.push_back(protoId);
}

void CTestCase_SavedExamples::runOneStep()
{
   //if (m_nRequests < 1) {
   //   if (m_nCallsLeft > 0) {
   //      issueRequest();
   //      m_nCallsLeft--;
   //   }
   //   m_pOwner->sleepComponent(500);
   //}

   //if(m_nCallsLeft <= 0 && m_protoIDs.size() > 1) {
   //   m_pOwner->sleepComponent(2000);
   //   string id = m_protoIDs.front();
   //   m_protoIDs.erase(m_protoIDs.begin());
   //   m_pOwner->deleteFromWorkingMemory(id);
   //}
}

