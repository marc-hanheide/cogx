// vim:set fileencoding=utf-8 sw=3 ts=8 et:vim
/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#include "TestCases.h"

#include <string>
#include <VisionData.hpp>

using namespace std;
using namespace VisionData;

//------------------------------------------------------- 
// VERIFY IF the Recognizer Server works
//------------------------------------------------------- 
CTestCase_Server::CTestCase_Server(string name, CTestRecognizer *pOwner)
      : CTestCase(name, pOwner)
{
}

void CTestCase_Server::onStart()
{
   // Example:
   //m_pOwner->addChangeFilter(
   //   createLocalTypeFilter<VisualLearnerRecognitionTask>(cdl::OVERWRITE),
   //   new MemberFunctionChangeReceiver<CTestCase_Standalone>(
   //      this, &CTestCase_Standalone::onChange_RecognitionTask)
   //   );
}

void CTestCase_Server::runOneStep()
{
   // Example:
   //if (m_pOwner->m_ProtoObjects < 1) {
   //   if (m_nCallsLeft > 0) {
   //      issueRequest();
   //      m_nCallsLeft--;
   //   }
   //   m_pOwner->sleepComponent(500);
   //}
}



//------------------------------------------------------- 
// VERIFY IF the Recognizer responds to WM event
//------------------------------------------------------- 
CTestCase_WmResponder::CTestCase_WmResponder(string name, CTestRecognizer *pOwner)
      : CTestCase(name, pOwner)
{
}

void CTestCase_WmResponder::onStart()
{
   // Example:
   //m_pOwner->addChangeFilter(
   //   createLocalTypeFilter<VisualLearnerRecognitionTask>(cdl::OVERWRITE),
   //   new MemberFunctionChangeReceiver<CTestCase_Standalone>(
   //      this, &CTestCase_Standalone::onChange_RecognitionTask)
   //   );

   // Global change filter expecting message from vision.sa
   // (ID set in CAST file, subarchitecture entry)
   // TODO: Events moved from CTestRecognizer to CTestCase_WmResponder
   //addChangeFilter(
   //   createLocalTypeFilter<ObjectRecognitionTask>(cdl::ADD),
   //   new MemberFunctionChangeReceiver<CTestRecognizer>(
   //      this, &CTestRecognizer::onAdd_RecognitionTask)
   //   );

   //addChangeFilter(
   //   createLocalTypeFilter<ObjectRecognitionTask>(cdl::DELETE),
   //   new MemberFunctionChangeReceiver<CTestRecognizer>(
   //      this, &CTestRecognizer::onDelete_RecognitionTask)
   //   );

   //addChangeFilter(
   //   createLocalTypeFilter<ObjectRecognitionTask>(cdl::OVERWRITE),
   //   new MemberFunctionChangeReceiver<CTestRecognizer>(
   //      this, &CTestRecognizer::onChange_RecognitionTask)
   //   );

   //addChangeFilter(
   //   createLocalTypeFilter<ProtoObject>(cdl::ADD),
   //   new MemberFunctionChangeReceiver<CTestRecognizer>(
   //      this, &CTestRecognizer::onAdd_ProtoObject)
   //   );
}


// TODO: Event moved from CTestRecognizer to CTestCase_WmResponder
//void CTestRecognizer::onAdd_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
//{
//   nTasks++;
//   log("Recognition task added. %ld in WM.", nTasks);
//}

// TODO: Event moved from CTestRecognizer to CTestCase_WmResponder
//void CTestRecognizer::onDelete_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
//{
//   nTasks--;
//   log("Recognition task removed. %ld in WM.", nTasks);
//}

// TODO: Event moved from CTestRecognizer to CTestCase_WmResponder
//void CTestRecognizer::onChange_RecognitionTask(const cdl::WorkingMemoryChange & _wmc)
//{
//   log("Recognition task modified.");
//   ObjectRecognitionTaskPtr pcmd = getMemoryEntry<ObjectRecognitionTask>(_wmc.address);
//   std::vector<string>::iterator itstr;
//   std::vector<double>::iterator itdbl;

//   ostringstream msg;
//   std::vector<VisionData::ObjectRecognitionMatchPtr>::iterator pmatch;
//   for (pmatch = pcmd->matches.begin(); pmatch < pcmd->matches.end(); pmatch++) {
//      println("Match for '%s' ID: [%s]", (*pmatch)->sourceType.c_str(), (*pmatch)->sourceId.id.c_str());
//      msg.str("");
//      for (itstr = (*pmatch)->objectId.begin(); itstr != (*pmatch)->objectId.end(); itstr++) {
//         msg << *itstr << ", ";
//      }
//      println("Labels: %s", msg.str().c_str());

//      msg.str("");
//      for (itdbl = (*pmatch)->probability.begin(); itdbl != (*pmatch)->probability.end(); itdbl++) {
//         msg << *itdbl << ", ";
//      }
//      println("Probabilities: %s", msg.str().c_str());
//   }

//   // TODO: dump the request!
//   if (testmode) {
//      sleepComponent(500);
//      deleteFromWorkingMemory(_wmc.address);
//   }
//}

// TODO: Event moved from CTestRecognizer to CTestCase_WmResponder
//void CTestRecognizer::onAdd_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
//{
//   log("New proto object");
//   // TODO: add request to the queue
//}

void CTestCase_WmResponder::runOneStep()
{
   // Example:
   //if (m_pOwner->m_ProtoObjects < 1) {
   //   if (m_nCallsLeft > 0) {
   //      issueRequest();
   //      m_nCallsLeft--;
   //   }
   //   m_pOwner->sleepComponent(500);
   //}

   string id(m_pOwner->newDataID());
   ObjectRecognitionTaskPtr task = new ObjectRecognitionTask();
   m_pOwner->println("Adding new task");
   m_pOwner->addToWorkingMemory(id, m_pOwner->getSubarchitectureID(), task);
   m_pOwner->sleepComponent(5000);
}

