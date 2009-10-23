/*
 * @author:  Marko Mahnič
 * @created: October 2009 
 */

#include <algorithm>   
#include <opencv/cv.h>
#include <opencv/highgui.h>

// cast
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <VideoUtils.h>
#include <Video.hpp>

#include "TestLearner.h"
#include "../../../VisionUtils.h"

using namespace std;
using namespace cast;
using namespace VisionData;

extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new CTestRecognizer();
   }
}

static string descAddr(const cdl::WorkingMemoryAddress &addr)
{
   string res("[" + addr.subarchitecture + "/" + addr.id + "]");
   return res;
}

CTestRecognizer::CTestRecognizer()
{
   testmode = "standalone";
   nRequests = 0;
   learningStepsComplete = 999;
}

void CTestRecognizer::start()
{
   log("Recognizer TEST starting");

   // Global change filter expecting message from vision.sa
   // (ID set in CAST file, subarchitecture entry)
   addChangeFilter(
      createLocalTypeFilter<VisualLearnerRecognitionTask>(cdl::ADD),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onAdd_RecognitionTask)
      );

   addChangeFilter(
      createLocalTypeFilter<VisualLearnerRecognitionTask>(cdl::DELETE),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onDelete_RecognitionTask)
      );

   addChangeFilter(
      createLocalTypeFilter<VisualLearnerRecognitionTask>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onChange_RecognitionTask)
      );

   addChangeFilter(
      createLocalTypeFilter<VisualLearnerLearningTask>(cdl::ADD),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onAdd_LearningTask)
      );

   addChangeFilter(
      createLocalTypeFilter<VisualLearnerLearningTask>(cdl::DELETE),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onDelete_LearningTask)
      );

   addChangeFilter(
      createLocalTypeFilter<VisualLearnerLearningTask>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onChange_LearningTask)
      );

   addChangeFilter(
      createLocalTypeFilter<ProtoObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onAddProtoObject)
      );

   addChangeFilter(
      createLocalTypeFilter<AttrObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onAddAttrObject)
      );
}

void CTestRecognizer::configure(const std::map<std::string,std::string> & _config)
{
   map<string,string>::const_iterator it;

   if((it = _config.find("--testmode")) != _config.end())
   {
      string mode;
      istringstream istr(it->second);
      istr >> mode;
      if (mode == "standalone") testmode = mode;
      else if (mode == "fake-proto") testmode = mode;
      else testmode = "***unknown***";
      log("TEST MODE: %s", testmode.c_str());
   }
   else if((it = _config.find("--delay")) != _config.end()) {
      string delay;
      istringstream istr(it->second);
      istr >> delay;
      // TODO: convert to number
   }
}

void CTestRecognizer::onAdd_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   nRequests++;
   log("Recognition task %s added. %d requests in WM.", descAddr(_wmc.address).c_str(), nRequests);
}

void CTestRecognizer::onDelete_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   nRequests--;
   log("Recognition task %s removed. %d requests in WM.", descAddr(_wmc.address).c_str(), nRequests);
}

static
bool intLower (int i, int j)
{
   return (i<j);
}

void CTestRecognizer::onChange_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   log("Recognition task %s modified. %d requests in WM.", descAddr(_wmc.address).c_str(), nRequests);
   VisualLearnerRecognitionTaskPtr pTask = getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
   std::vector<int>::iterator plabel;
   std::vector<double>::iterator pdbl;

   pdbl = pTask->distribution.begin();
   for( plabel = pTask->labels.begin(); plabel != pTask->labels.end(); plabel++) {
      log("Label '%d' Distr '%f'", *plabel, pdbl != pTask->distribution.end() ? *pdbl : -1);
      if (pdbl != pTask->distribution.end()) pdbl++;
   }

   sleepComponent(100);
   deleteFromWorkingMemory(_wmc.address);
   
   if (testmode == "learning") {
      learningStepsComplete++;
      if (learningStepsComplete == 1) {
         labels.clear();
         for( plabel = pTask->labels.begin(); plabel != pTask->labels.end(); plabel++) {
            labels.push_back(*plabel);
         std::sort(labels.begin(), labels.end(), intLower);
      }
   }
}

void CTestRecognizer::onAdd_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   nRequests++;
   log("Learning task %s added. %d requests in WM.", descAddr(_wmc.address).c_str(), nRequests);
   VisualLearnerRecognitionTaskPtr pTask = getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);

   std::vector<int>::iterator plabel;
   std::vector<double>::iterator pdbl;
   ostringstream ostr;
   ostr << "Labels: "
   for( plabel = pTask->labels.begin(); plabel != pTask->labels.end(); plabel++) {
      ostr << "'" << *plabel << "' ";
   }
   log(ostr.str());

   ostringstream ostr2;
   ostr2 << "Distr: "
   for( pdbl = pTask->distribution.begin(); pdbl != pTask->distribution.end(); pdbl++) {
      ostr2 << *pdbl << " ";
   }
   log(ostr2.str());
}

void CTestRecognizer::onDelete_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   nRequests--;
   log("Learning task %s removed. %d requests in WM.", descAddr(_wmc.address).c_str(), nRequests);
}

void CTestRecognizer::onChange_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   log("Learning task %s modified. %d requests in WM.", descAddr(_wmc.address).c_str(), nRequests);
   VisualLearnerRecognitionTaskPtr pTask = getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
   std::vector<int>::iterator plabel;
   std::vector<double>::iterator pdbl;

   sleepComponent(100);
   deleteFromWorkingMemory(_wmc.address);
   
   if (testmode == "learning") learningStepsComplete++;
}

void CTestRecognizer::onAddProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   log("Detected: new proto object: %s", descAddr(_wmc.address).c_str());
}

ProtoObjectPtr CTestRecognizer::loadFakeProtoObject()
{
   ProtoObjectPtr pproto = new ProtoObject();
   pproto->time = getCASTTime();

   IplImage *pimg = cvLoadImage("subarchitectures/vision.sa/config/test-vislearner/image.png");
   Video::convertImageFromIpl(pimg, pproto->image);
   cvReleaseImage(&pimg);

   // imgMask bytes to pproto->mask bytes
   IplImage *pmsk = cvLoadImage("subarchitectures/vision.sa/config/test-vislearner/mask.png");
   Video::Image imgMask;
   Video::convertImageFromIpl(pmsk, imgMask);
   cvReleaseImage(&pmsk);
   int count = imgMask.width * imgMask.height;
   pproto->mask.width = imgMask.width;
   pproto->mask.height = imgMask.height;
   pproto->mask.data.resize(count);
   for (int ipix = 0; ipix < count; ipix++)
      pproto->mask.data[ipix] = imgMask.data[ipix*3]; // copy one channel

   return pproto;
}

// Add a new recognition task with an artificial protoobject.
// The resutls are parsed in onChangeRecognitionTask
void CTestRecognizer::_test_addRecognitionTask()
{
   println("Adding new ProtoObject");

   if (testmode == "standalone") {
      ProtoObjectPtr pproto = loadFakeProtoObject();
      string protoId = newDataID();
      addToWorkingMemory(protoId, pproto);

      println("Adding new VisualLearnerRecognitionTask");
      VisualLearnerRecognitionTaskPtr ptask = new VisualLearnerRecognitionTask();
      ptask->protoObjectId = protoId;

      string reqId(newDataID());
      addToWorkingMemory(reqId, ptask);
   }
   else if (testmode == "fake-proto") {
      ProtoObjectPtr pproto = loadFakeProtoObject();
      string protoId = newDataID();
      addToWorkingMemory(protoId, pproto);
   }
}

void CTestRecognizer::_test_performLearningStep(int issued, string protoId)
{
   // 1. Recognize
   // 2. Update: learn
   // 3. Recognize
   // 4. Update: unlearn
   // 5. Recognize
   std::vector<int>::iterator plabel;
   int n;
   if (issued == 1 || issued == 3 || issued == 5) {
      log("STEP %d: RECOGNITION", issued)
      VisualLearnerRecognitionTaskPtr ptask = new VisualLearnerRecognitionTask();
      ptask->protoObjectId = protoId;

      string reqId(newDataID());
      addToWorkingMemory(reqId, ptask);
   }
   else if (issued == 2) {
      log("STEP %d: LEARNING", issued)
      VisualLearnerLearningTaskPtr ptask = new VisualLearnerLearningTask();
      ptask->protoObjectId = protoId;
      std::vector<double> &distrib = ptask->distribution;

      double sum = 0;
      for( plabel = labels.begin(); plabel != labels.end(); plabel++) {
         ptask->labels.push_back(*plabel);
         n = rand(100);
         distrib.push_back(n);
         sum += n;
      }
      for (n = 0; n < distrib.size(); n++) distrib[n] = distrib[n] / sum;

      string reqId(newDataID());
      addToWorkingMemory(reqId, ptask);
   }
   else if (issued == 4) {
      log("STEP %d: UNLEARNING", issued)
      VisualLearnerLearningTaskPtr ptask = new VisualLearnerLearningTask();
      ptask->protoObjectId = protoId;
      std::vector<double> &distrib = ptask->distribution;

      string reqId(newDataID());
      double sum = 0;
      for( plabel = labels.begin(); plabel != labels.end(); plabel++) {
         ptask->labels.push_back(*plabel);
         n = rand(100);
         distrib.push_back(n);
         sum += n;
      }
      for (n = 0; n < dist.size(); n++) {
         distrib[n] = distrib[n] / sum;
         if (n % 2 == 0) distrib[n] = -distrib[n];
      }

      string reqId(newDataID());
      addToWorkingMemory(reqId, ptask);
   }
}

void CTestRecognizer::runComponent()
{
   sleepComponent(2000);

   if (testmode == "learning") {
      ProtoObjectPtr pproto = loadFakeProtoObject();
      string protoId = newDataID();
      addToWorkingMemory(protoId, pproto);
      learningStepComplete = 0;
      int issued = 0;
      while(isRunning()) {
         if (issued <= learningStepsComplete && issued < 6) {
            issued++;
            _test_performLearningStep(issued, protoId);
         }
         sleepComponent(500);
      }
   }
   else {
      int nCalls = 3;
      while(isRunning()) {
         // TODO: if request queue not empty and not processing
         // If deleteFromWorkingMemory is disabled in onChangeRecognitionTask
         // then stop after 3 requests, otherwise create new tasks indefinitely.
         if (nRequests < 1) {
            if (nCalls) {
               _test_addRecognitionTask();
               nCalls --;
            }
            sleepComponent(500);
         }

         sleepComponent(300);
      }
   }
   log("runComponent Done.");
}

/* vim:set fileencoding=utf-8 sw=3 ts=8 et:vim */
