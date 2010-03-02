// vim:set fileencoding=utf-8 sw=3 ts=8 et:vim
/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#include "TestCases.h"

#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <Ice/LocalException.h>

#include <VideoUtils.h>
#include <Video.hpp>
#include <VisionData.hpp>
#include <ObjectRecognizerSrv.hpp>

using namespace std;
using namespace VisionData;
using namespace cogx::vision;
namespace orice = ObjectRecognizerIce;

//------------------------------------------------------- 
// VERIFY if the Recognizer Server works
//------------------------------------------------------- 
CTestCase_Server::CTestCase_Server(string name, CTestRecognizer *pOwner)
      : CTestCase(name, pOwner)
{
}

void CTestCase_Server::configure(const map<string,string> & _config)
{
   m_OrClient.configureRecognizer(_config);
}

void CTestCase_Server::onStart()
{
   m_OrClient.connectIceClient(*m_pOwner);

   // Example:
   //m_pOwner->addChangeFilter(
   //   createLocalTypeFilter<ObjectRecognitionTask>(cdl::OVERWRITE),
   //   new MemberFunctionChangeReceiver<CTestCase_Server>(
   //      this, &CTestCase_Server::onChange_RecognitionTask)
   //   );
}

//void CTestCase_Server::onExitComponent()
//{
//   m_pOwner->removeChangeFilter(...);
//}

// The test will run 5 times, after that it will only log.
// The tests can be repeated only after the servers are restarted.
int ccount = 5;
void CTestCase_Server::runOneStep()
{
   m_pOwner->log("FindMatchingObjects, const image; attempts left: %d", ccount);

   IplImage *pimg = cvLoadImage("subarchitectures/vision.sa/config/test-vislearner/image.png");
   Video::Image image;
   Video::convertImageFromIpl(pimg, image);
   cvReleaseImage(&pimg);
   orice::RecognitionResultSeq results;

   try {
      if (ccount > 0) m_OrClient.FindMatchingObjects(image, results);
   }
   catch (const Ice::ObjectNotExistException &e) {
      m_pOwner->log("Server does not exist. %s", e.what());
      m_pOwner->sleepComponent(2000);
   }
   if (ccount > 0) ccount--;

   m_pOwner->sleepComponent(500);
}



//------------------------------------------------------- 
// VERIFY if the Recognizer responds to WM event
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

