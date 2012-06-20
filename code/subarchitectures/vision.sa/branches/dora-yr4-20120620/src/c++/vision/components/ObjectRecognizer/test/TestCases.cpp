// vim:set fileencoding=utf-8 sw=3 ts=8 et:vim
/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#include "TestCases.h"

#include <string>

#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <Ice/LocalException.h>
#include <cast/architecture/ChangeFilterFactory.hpp>

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
   m_pOwner->log("going to: configureRecognizer");
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

// The test will run ccount times, after that it will only log.
// The tests can be repeated only after the servers are restarted.
int ccount = 5555;
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


void CTestCase_ServerCamera::runOneStep()
{
   m_pOwner->log("FindMatchingObjects, camera %d", m_pOwner->m_camId);

   Video::Image image;
   if (! m_pOwner->getOneImage(image)) {
      m_pOwner->log("   Could not get an image");
   }
   else {
      orice::RecognitionResultSeq results;
      try {
         m_OrClient.FindMatchingObjects(image, results);
      }
      catch (const Ice::ObjectNotExistException &e) {
         m_pOwner->log("Server does not exist. %s", e.what());
         m_pOwner->sleepComponent(2000);
      }
   }

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
   orice::ObjectRecognitionTaskPtr task = new orice::ObjectRecognitionTask();
   m_pOwner->println("Adding new task");
   m_pOwner->addToWorkingMemory(id, m_pOwner->getSubarchitectureID(), task);
   m_pOwner->sleepComponent(5000);
}


//------------------------------------------------------- 
// VERIFY if the Recognizer Serverworks with the stereo
// pipeline (process new ProtoObjects).
//------------------------------------------------------- 

void CTestCase_StereoPipeline::onStart()
{
   CTestCase_Server::onStart();

   m_pOwner->addChangeFilter(
     cast::createLocalTypeFilter<ProtoObject>(cast::cdl::ADD),
     new cast::MemberFunctionChangeReceiver<CTestCase_StereoPipeline>(
        this, &CTestCase_StereoPipeline::onAdd_ProtoObject)
     );
}

void CTestCase_StereoPipeline::runOneStep()
{
   std::vector<CProcessItem*> queue;

   {
      // id=StereoPipeline_timedWait
      // SYNC: Lock the monitor
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_queueMonitor);
      if (m_queue.size() < 1) {
         // SYNC: Unlock the monitor and wait for notify() or timeout
         m_queueMonitor.timedWait(IceUtil::Time::seconds(2));
         // SYNC: Continue with a locked monitor
      }

      // move new items (if any) to another vector
      queue = m_queue;
      m_queue.clear();
      // SYNC: unlock the monitor when going out of scope
   }

   if (queue.size() > 0) {
      m_pOwner->log("CTestCase_StereoPipeline: Got items in queue.");
      std::vector<CProcessItem*>::iterator it;
      CProcessItem *pItem;
      for (it = queue.begin(); it != queue.end(); it++) {
         pItem = *it;
         if (! pItem) continue;
         if (pItem->m_type == "protoobject") {
            processProtoObject(pItem->m_address);
         }
         delete pItem;
      }
   }
}

// Receiver thread
void CTestCase_StereoPipeline::onAdd_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   CProcessItem* pItem = new CProcessItem("protoobject", _wmc.address);
   IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_queueMonitor);
   m_queue.push_back(pItem);
   m_queueMonitor.notify(); // wakes up the runComponent thread (<url:#r=StereoPipeline_timedWait>)
}

void CTestCase_StereoPipeline::processProtoObject(const cast::cdl::WorkingMemoryAddress& address)
{
   ProtoObjectPtr pObj;
   try {
     pObj = m_pOwner->getMemoryEntry<VisionData::ProtoObject>(address);
   }
   catch(cast::DoesNotExistOnWMException e) {
     m_pOwner->println("WARNING: Proto-object ID %s removed before it could be processed", address.id.c_str());
     return;
   }

   Video::Image image;
   image = pObj->image;
   
   orice::RecognitionResultSeq results;
   try {
      m_pOwner->log("Calling m_OrClient for Proto %s.",  address.id.c_str());
      m_OrClient.FindMatchingObjects(image, results);
   }
   catch (const Ice::ObjectNotExistException &e) {
      m_pOwner->println("Server does not exist. %s", e.what());
      m_pOwner->sleepComponent(2000);
   }
}
