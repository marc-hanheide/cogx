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
#include "../../../VisionUtils.h"

#include "TestLearner.h"
#include "TestCases.h"

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
   m_pTestCase = NULL;
   testmode = "*** unknown-test ***";
   m_RecogTasks = 0;
   m_LearnTasks = 0;
   dumpChanged_RecogTask = false;
   dumpNew_LearningTask = false;
}

void CTestRecognizer::start()
{
   log("Recognizer TEST starting");

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
      createLocalTypeFilter<VisualLearningTask>(cdl::ADD),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onAdd_LearningTask)
      );

   addChangeFilter(
      createLocalTypeFilter<VisualLearningTask>(cdl::DELETE),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onDelete_LearningTask)
      );

   addChangeFilter(
     createLocalTypeFilter<VisualLearningTask>(cdl::OVERWRITE),
     new MemberFunctionChangeReceiver<CTestRecognizer>(
        this, &CTestRecognizer::onChange_LearningTask)
     );

   addChangeFilter(
      createLocalTypeFilter<ProtoObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onAdd_ProtoObject)
      );

   addChangeFilter(
      createLocalTypeFilter<ProtoObject>(cdl::DELETE),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onDelete_ProtoObject)
      );

   if (m_pTestCase) {
      m_pTestCase->onStart();
   }
}

void CTestRecognizer::configure(const std::map<std::string,std::string> & _config)
{
   map<string,string>::const_iterator it;

   if((it = _config.find("--testmode")) != _config.end())
   {
      string mode;
      istringstream istr(it->second);
      istr >> mode;
      if (mode == "standalone") {
         testmode = mode;
         m_pTestCase = new CTestCase_Standalone(mode, this);
      }
      else if (mode == "fake-proto") {
         testmode = mode;
         m_pTestCase = new CTestCase_FakeProto(mode, this);
      }
      else if (mode == "learning") {
         testmode = mode;
         m_pTestCase = new CTestCase_Learning(mode, this);
      }
      else if (mode == "saved-example") {
         testmode = mode;
         m_pTestCase = new CTestCase_SavedExamples(mode, this);
      }
      log("TEST MODE: %s", testmode.c_str());
   }

   if((it = _config.find("--delay")) != _config.end()) {
      string delay;
      istringstream istr(it->second);
      istr >> delay;
      // TODO: convert to number
   }
}

void CTestRecognizer::onAdd_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   m_RecogTasks++;
   log("Recognition task %s added. %d requests in WM.", descAddr(_wmc.address).c_str(), m_RecogTasks);
}

void CTestRecognizer::onDelete_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   m_RecogTasks--;
   log("Recognition task %s removed. %d requests in WM.", descAddr(_wmc.address).c_str(), m_RecogTasks);
}

static
bool intLower (int i, int j)
{
   return (i<j);
}

void CTestRecognizer::onChange_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   log("Recognition task %s modified. %d requests in WM.", descAddr(_wmc.address).c_str(), m_RecogTasks);
   if (dumpChanged_RecogTask) {
      VisualLearnerRecognitionTaskPtr pTask = getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
      std::vector<string>::iterator plabel;
      std::vector<double>::iterator pdbl;

      pdbl = pTask->distribution.begin();
      for( plabel = pTask->labels.begin(); plabel != pTask->labels.end(); plabel++) {
         log("Label '%s' Distr '%f'", plabel->c_str(), pdbl != pTask->distribution.end() ? *pdbl : -1);
         if (pdbl != pTask->distribution.end()) pdbl++;
      }
   }
}

void CTestRecognizer::onAdd_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   m_LearnTasks++;
   log("Learning task %s added. %d requests in WM.", descAddr(_wmc.address).c_str(), m_LearnTasks);
   if (dumpNew_LearningTask) {
      VisualLearningTaskPtr pTask = getMemoryEntry<VisualLearningTask>(_wmc.address);

      std::vector<string>::iterator plabel;
      std::vector<double>::iterator pdbl;
      ostringstream ostr;
      ostr << "Labels: ";
      for( plabel = pTask->labels.begin(); plabel != pTask->labels.end(); plabel++) {
         ostr << "'" << *plabel << "' ";
      }
      log(ostr.str());

      ostringstream ostr2;
      ostr2 << "Weights: ";
      for( pdbl = pTask->weights.begin(); pdbl != pTask->weights.end(); pdbl++) {
         ostr2 << *pdbl << " ";
      }
      log(ostr2.str());
   }
}

void CTestRecognizer::onDelete_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   m_LearnTasks--;
   log("Learning task %s removed. %d requests in WM.", descAddr(_wmc.address).c_str(), m_LearnTasks);
}

void CTestRecognizer::onChange_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   log("Learning task %s modified. %d requests in WM.", descAddr(_wmc.address).c_str(), m_LearnTasks);
}

void CTestRecognizer::onAdd_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   m_ProtoObjects++;
   log("New proto object: %s. %d in WM", descAddr(_wmc.address).c_str(), m_ProtoObjects);
}

void CTestRecognizer::onDelete_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   m_ProtoObjects--;
   log("Deleted proto object: %s. %d in WM", descAddr(_wmc.address).c_str(), m_ProtoObjects);
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

void CTestRecognizer::runComponent()
{
   sleepComponent(2000);

   if (m_pTestCase != NULL) {
      while(isRunning()) {
         m_pTestCase->runOneStep();
         sleepComponent(10);
      }
   }
   if (m_pTestCase) {
      log("runComponent: Test Case Cleanup.");
      m_pTestCase->onExitComponent();
      delete m_pTestCase;
      m_pTestCase = NULL;
   }
   log("runComponent Done.");
}

/* vim:set fileencoding=utf-8 sw=3 ts=8 et:vim */
