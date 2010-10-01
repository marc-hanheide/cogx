#include "VisualLearner.h"
#include "VisualLearnerProxy.h"

#include <cast/architecture/ChangeFilterFactory.hpp>

extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::VisualLearner();
   }
}

//default useful namespaces, fix to reflect your own code
using namespace std;
using namespace VisionData;
using namespace cast;
using namespace boost;
using namespace Matlab; // Matrix

namespace cogx {
static string descAddr(const cdl::WorkingMemoryAddress &addr)
{
   string res("[" + addr.subarchitecture + "/" + addr.id + "]");
   return res;
}

VisualLearner::VisualLearner()
{
   m_ClfConfigFile = "";
}

VisualLearner::~VisualLearner()
{
}

void VisualLearner::configure(const map<string, string>& _config)
{
   // first let the base class configure itself
   ManagedComponent::configure(_config);

   log("configure");

   map<string,string>::const_iterator it;
   if((it = _config.find("--clfconfig")) != _config.end()) {
      string s = it->second;
      // trim
      s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
      m_ClfConfigFile = s;
      log("--clfconfig='%s'", m_ClfConfigFile.c_str());
   }
}

void VisualLearner::start()
{
   debug("::start");
   ManagedComponent::start();

   addChangeFilter(createLocalTypeFilter<VisualLearnerRecognitionTask>(cdl::ADD),
         new MemberFunctionChangeReceiver<VisualLearner>(this, &VisualLearner::onAdd_RecognitionTask));

   addChangeFilter(createLocalTypeFilter<VisualLearningTask>(cdl::ADD),
         new MemberFunctionChangeReceiver<VisualLearner>(this, &VisualLearner::onAdd_LearningTask));
}

void VisualLearner::onAdd_RecognitionTask(const cdl::WorkingMemoryChange & _wmc)
{
   debug("::onAdd_RecognitionTask");
   string type(_wmc.type);
   string id(_wmc.address.id);

   log("Added: " + type + ", id: " + id);
   VisualLearnerRecognitionTaskPtr pTask;
   try {
      // get the data from working memory
      // pTask = getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
      {
         IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_RrqMonitor);
         m_RecogTaskId_Queue.push_back(_wmc.address);
         m_RrqMonitor.notify();
      }
      log("Request addr pushed: %s", descAddr(_wmc.address).c_str());
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner.OnAdd: VL_RecognitionTask deleted while working...");
      return;
   };
}

void VisualLearner::onAdd_LearningTask(const cdl::WorkingMemoryChange & _wmc)
{
   debug("::onAdd_LearningTask");
   string type(_wmc.type);
   string id(_wmc.address.id);

   log("Added: " + type + ", id: " + id);
   VisualLearningTaskPtr pTask;
   try {
      // get the data from working memory
      // pTask = getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
      {
         IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_RrqMonitor);
         m_LearnTaskId_Queue.push_back(_wmc.address);
         m_RrqMonitor.notify();
      }
      log("Request addr pushed: %s", descAddr(_wmc.address).c_str());
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner.OnAdd: VL_LearningTask deleted while working...");
      return;
   };
}

void VisualLearner::runComponent()
{
   debug("::runComponent");
   sleepComponent(3011);

   if (m_ClfConfigFile.size() > 0)
      matlab::VL_setClfStartConfig(m_ClfConfigFile.c_str());

   TWmAddressVector::iterator pwma;
   while (isRunning()) {

      TWmAddressVector newRecogRqs;
      TWmAddressVector newLearnRqs;
      {
         // SYNC: Lock the monitor
         IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_RrqMonitor);
         // SYNC: if queue empty, unlock the monitor and wait for notify() or timeout
         if (m_RecogTaskId_Queue.size() < 1 && m_LearnTaskId_Queue.size() < 1) 
            m_RrqMonitor.timedWait(IceUtil::Time::seconds(2));
         // SYNC: Continue with a locked monitor

         newRecogRqs = m_RecogTaskId_Queue;
         m_RecogTaskId_Queue.clear();
         newLearnRqs = m_LearnTaskId_Queue;
         m_LearnTaskId_Queue.clear();
         // SYNC: unlock the monitor on scope exit
      }

      if (newRecogRqs.size() > 0) {
         pwma = newRecogRqs.begin();
         cdl::WorkingMemoryAddress addr = *pwma;
         newRecogRqs.erase(pwma);

         log("VL_Recognition Request addr popped: %s", descAddr(addr).c_str());
         VisualLearnerRecognitionTaskPtr pTaskData;
         try{
            // get the data from working memory
            pTaskData = getMemoryEntry<VisualLearnerRecognitionTask>(addr);

            // TODO: check if pTaskData is valid
            // TODO: lock pTaskData in WM
            if (recogniseAttributes(pTaskData)) {
               debug("Will overwrite task: %s", descAddr(addr).c_str());
               // sleepComponent(100);
               overwriteWorkingMemory(addr, pTaskData);
            }
            // TODO: unlock pTaskData in WM
         }
         catch(cast::DoesNotExistOnWMException){
            log("run: VisualLearnerRecognitionTask deleted while working...");
         };
         // TODO: catch other stuff from Matlab Proxy

         log("VL_Recognition Task processed");
      } // while

      if (newLearnRqs.size() > 0) {
         pwma = newLearnRqs.begin();
         cdl::WorkingMemoryAddress addr = *pwma;
         newLearnRqs.erase(pwma);

         log("Learning Request addr popped: %s", descAddr(addr).c_str());
         VisualLearningTaskPtr pTaskData;
         try{
            // get the data from working memory
            pTaskData = getMemoryEntry<VisualLearningTask>(addr);

            // TODO: check if pTaskData is valid
            // TODO: lock pTaskData
            if (updateModel(pTaskData)) {
               debug("Will overwrite task: %s", descAddr(addr).c_str());
               // sleepComponent(100);
               overwriteWorkingMemory(addr, pTaskData); // Not changed, but write to notify owner!
            }
            // TODO: unlock pTaskData
         }
         catch(cast::DoesNotExistOnWMException){
            log("VisualLearner.run: VisualLearnerLearningTask deleted while working...");
         };
         // TODO: catch other stuff from Matlab Proxy

         log("VL_Learning Task processed");
      }

   } // while isRunning
}

bool VisualLearner::recogniseAttributes(VisualLearnerRecognitionTaskPtr _pTask)
{
   debug("::recogniseAttributes");
   //try{
   //   lockEntry(_pProtoObj.address, cdl::LOCKEDODR);
   //}
   //catch(cast::DoesNotExistOnWMException){
   //   printf("VisualLearner: ProtoObject deleted while working...\n");
   //   return;
   //};

   // get the SOI data nah: this is naughty, but as we're overwriting
   // the soi in wm anyway it's not too bad

   cdl::WorkingMemoryAddress addr;
   addr.subarchitecture = string(getSubarchitectureID());
   addr.id = _pTask->protoObjectId;
   ProtoObjectPtr pProtoObj;
   try{
      pProtoObj = getMemoryEntry<ProtoObject>(addr);
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner.recogAttr: ProtoObject %s deleted while working...", descAddr(addr).c_str());
      return false;
   };
   // TODO: lock pProtoObj

   matlab::VL_recognise_attributes(*pProtoObj, _pTask->labels, _pTask->labelConcepts,
         _pTask->distribution, _pTask->gains);

   // TODO: unlock pProtoObj
   return true;

   // Now write this back into working memory, this will manage the memory for us.
   //debug("::recogniseAttributes writing");
   //overwriteWorkingMemory<SOI>(_pProtoObj->getAddress(), _pProtoObj);
   //log("Added a new AttrObject.");
   //unlockEntry(_pProtoObj.address);
}

bool VisualLearner::updateModel(VisualLearningTaskPtr _pTask)
{
   debug("::updateModel");

   cdl::WorkingMemoryAddress addr;
   addr.subarchitecture = string(getSubarchitectureID());
   addr.id = _pTask->visualObjectId;

   VisualObjectPtr pVisObj;
   try{
      pVisObj = getMemoryEntry<VisualObject>(addr);
      addr.id = pVisObj->protoObjectID;
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner.updateAttr: VisualObject %s deleted while working...", descAddr(addr).c_str());
      return false;
   };

   ProtoObjectPtr pProtoObj;
   try{
      pProtoObj = getMemoryEntry<ProtoObject>(addr);
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner.updateAttr: ProtoObject %s deleted while working...", descAddr(addr).c_str());
      return false;
   };

   matlab::VL_update_model(*pProtoObj, _pTask->labels, _pTask->weights);

   return true;
}

} // namespace
