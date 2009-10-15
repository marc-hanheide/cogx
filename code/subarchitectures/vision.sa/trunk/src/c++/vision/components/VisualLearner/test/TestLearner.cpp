/*
 * @author:  Marko Mahnič
 * @created: October 2009 
 */
   
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

CTestRecognizer::CTestRecognizer()
{
   testmode = 0;
   nRequests = 0;
}

void CTestRecognizer::start()
{
   log("Recognizer TEST starting");

   // Global change filter expecting message from vision.sa
   // (ID set in CAST file, subarchitecture entry)
   addChangeFilter(
      createLocalTypeFilter<VisualLearnerRecognitionTask>(cdl::ADD),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onAddRecognitionTask)
      );

   addChangeFilter(
      createLocalTypeFilter<VisualLearnerRecognitionTask>(cdl::DELETE),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onDeleteRecognitionTask)
      );

   addChangeFilter(
      createLocalTypeFilter<VisualLearnerRecognitionTask>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onChangeRecognitionTask)
      );

   addChangeFilter(
      createLocalTypeFilter<ProtoObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onAddProtoObject)
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
      if (mode == "1") testmode = 1;
      else if (mode == "2") testmode = 2;
      else if (mode == "3") testmode = 3;
      else testmode = 0;
      log("TEST MODE: %ld", testmode);
   }
}

void CTestRecognizer::onAddRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   log("Recognition task added. %d requests in WM.", nRequests);
}

void CTestRecognizer::onDeleteRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   nRequests--;
   log("Recognition task removed. %d requests in WM.", nRequests);
}

void CTestRecognizer::onChangeRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   log("Recognition task modified. %d requests in WM.", nRequests);
   VisualLearnerRecognitionTaskPtr pTask = getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
   std::vector<string>::iterator pstr;
   std::vector<double>::iterator pdbl;

   pdbl = pTask->colorDistr.begin();
   for( pstr = pTask->colorLabel.begin(); pstr != pTask->colorLabel.end(); pstr++) {
      log("Label '%s' Distr '%f'", pstr->c_str(), pdbl != pTask->colorDistr.end() ? *pdbl : -1);
      if (pdbl != pTask->colorDistr.end()) pdbl++;
   }

   sleepComponent(100);
   // deleteFromWorkingMemory(_wmc.address);
}

void CTestRecognizer::onAddProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   // log("New proto object");
}

void CTestRecognizer::_test_addRecognitionTask()
{
   println("Adding new ProtoObject");
   ProtoObjectPtr pproto = new ProtoObject();
   pproto->time = getCASTTime();

   IplImage *pimg = cvLoadImage("xdata/TestVisualLearner/image.png");
   IplImage *pmsk = cvLoadImage("xdata/TestVisualLearner/mask.png");
   Video::Image imgMask;
   Video::convertImageFromIpl(pimg, pproto->image);
   Video::convertImageFromIpl(pmsk, imgMask);
   cvReleaseImage(&pimg);
   cvReleaseImage(&pmsk);
   // imgMask bytes to pproto->mask bytes
   int count = imgMask.width * imgMask.height;
   pproto->mask.width = imgMask.width;
   pproto->mask.height = imgMask.height;
   pproto->mask.data.resize(count);
   for (int ipix = 0; ipix < count; ipix++)
      pproto->mask.data[ipix] = imgMask.data[ipix*3]; // copy one channel

   string protoId = newDataID();
   addToWorkingMemory(protoId, pproto);

   println("Adding new VisualLearnerRecognitionTask");
   VisualLearnerRecognitionTaskPtr ptask = new VisualLearnerRecognitionTask();
   ptask->protoObjectId = protoId;

   string reqId(newDataID());
   nRequests++;
   addToWorkingMemory(reqId, ptask);
}

void CTestRecognizer::runComponent()
{
   sleepComponent(2000);

   while(isRunning()) {
      // TODO: if request queue not empty and not processing
      if (nRequests < 3) _test_addRecognitionTask();

      sleepComponent(300);
   }
   log("runComponent Done.");
}

/* vim:set fileencoding=utf-8 sw=3 ts=8 et:vim */
