// vim:set fileencoding=utf-8 sw=1 ts=1 et:vim
/*
 * @author:  Marko Mahnič
 * @created: jun 2009 
 */
   
// cast
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <VideoUtils.h>
#include <Video.hpp>

#include "TestRecognizer.h"
#include "../../VisionUtils.h"

using namespace std;
using namespace cast;
using namespace VisionData;

extern "C" {
   cast::CASTComponentPtr newComponent()
   {
      return new CTestRecognizer();
   }
}

CTestRecognizer::CTestRecognizer()
{
   testmode = 0;
   nTasks = 0;
}

void CTestRecognizer::start()
{
   log("Recognizer TEST starting");

   // Global change filter expecting message from vision.sa
   // (ID set in CAST file, subarchitecture entry)
   addChangeFilter(
      createLocalTypeFilter<ObjectRecognitionTask>(cdl::ADD),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onAdd_RecognitionTask)
      );

   addChangeFilter(
      createLocalTypeFilter<ObjectRecognitionTask>(cdl::DELETE),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onDelete_RecognitionTask)
      );

   addChangeFilter(
      createLocalTypeFilter<ObjectRecognitionTask>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onChange_RecognitionTask)
      );

   addChangeFilter(
      createLocalTypeFilter<ProtoObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<CTestRecognizer>(
         this, &CTestRecognizer::onAdd_ProtoObject)
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

void CTestRecognizer::onAdd_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   nTasks++;
   log("Recognition task added. %ld in WM.", nTasks);
}

void CTestRecognizer::onDelete_RecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
   nTasks--;
   log("Recognition task removed. %ld in WM.", nTasks);
}

void CTestRecognizer::onChange_RecognitionTask(const cdl::WorkingMemoryChange & _wmc)
{
   log("Recognition task modified.");
   ObjectRecognitionTaskPtr pcmd = getMemoryEntry<ObjectRecognitionTask>(_wmc.address);
   std::vector<string>::iterator itstr;
   std::vector<double>::iterator itdbl;

   ostringstream msg;
   std::vector<VisionData::ObjectRecognitionMatchPtr>::iterator pmatch;
   for (pmatch = pcmd->matches.begin(); pmatch < pcmd->matches.end(); pmatch++) {
      println("Match for '%s' ID: [%s]", (*pmatch)->sourceType.c_str(), (*pmatch)->sourceId.id.c_str());
      msg.str("");
      for (itstr = (*pmatch)->objectId.begin(); itstr != (*pmatch)->objectId.end(); itstr++) {
         msg << *itstr << ", ";
      }
      println("Labels: %s", msg.str().c_str());

      msg.str("");
      for (itdbl = (*pmatch)->probability.begin(); itdbl != (*pmatch)->probability.end(); itdbl++) {
         msg << *itdbl << ", ";
      }
      println("Probabilities: %s", msg.str().c_str());
   }

   // TODO: dump the request!
   if (testmode) {
      sleepComponent(500);
      deleteFromWorkingMemory(_wmc.address);
   }
}

void CTestRecognizer::onAdd_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
   log("New proto object");
   // TODO: add request to the queue
}

void CTestRecognizer::_test_addRecognitionTask()
{
   string id(newDataID());
   ObjectRecognitionTaskPtr task = new ObjectRecognitionTask();
   println("Adding new task");
   addToWorkingMemory(id, getSubarchitectureID(), task);
   sleepComponent(5000);
}

void CTestRecognizer::runComponent()
{
   sleepComponent(2000);

   while(isRunning()) {
      // TODO: if request queue not empty and not processing
      if (nTasks < 3) {
         _test_addRecognitionTask();
      }

      sleepComponent(100);
   }
   log("runComponent Done.");
}

