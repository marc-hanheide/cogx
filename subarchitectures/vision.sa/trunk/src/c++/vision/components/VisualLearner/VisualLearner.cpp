#include "VisualLearner.h"
#include "VisualLearnerProxy.h"
//#include <vision/VisionGoals.h>
#include <VisionData.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

namespace cogx {
extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new VisualLearner();
   }
}

static string descAddr(const cdl::WorkingMemoryAddress &addr)
{
   string res("[" + addr.subarchitecture + "/" + addr.id + "]");
   return res;
}

VisualLearner::VisualLearner()
   // : WorkingMemoryAttachedComponent(), ManagedComponent()
{
} // VisualLearner::VisualLearner

VisualLearner::~VisualLearner()
{
} // VisualLearner::~VisualLearner

void VisualLearner::configure(map<string, string>& _config)
{
   // first let the base class configure itself
   ManagedComponent::configure(_config);
} // VisualLearner::configure

void VisualLearner::start()
{
   debug("::start");
   ManagedComponent::start();

   addChangeFilter(createLocalTypeFilter<VisualLearnerRecognitionTask>(cdl::ADD),
         new MemberFunctionChangeReceiver<VisualLearner>(this, &VisualLearner::onAdd_RecognitionTask));

   addChangeFilter(createLocalTypeFilter<VisualLearnerLearningTask>(cdl::ADD),
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
      m_RecogTaskId_Queue.push_back(_wmc.address);
      log("Request addr pushed: %s", descAddr(_wmc.address).c_str());
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner.OnAdd: VL_RecognitionTask deleted while working...\n");
      return;
   };
}

void VisualLearner::onAdd_LearningTask(const cdl::WorkingMemoryChange & _wmc)
{
   debug("::onAdd_LearningTask");
   string type(_wmc.type);
   string id(_wmc.address.id);

   log("Added: " + type + ", id: " + id);
   VisualLearnerLearningTaskPtr pTask;
   try {
      // get the data from working memory
      // pTask = getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
      m_LearnTaskId_Queue.push_back(_wmc.address);
      log("Request addr pushed: %s", descAddr(_wmc.address).c_str());
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner.OnAdd: VL_LearningTask deleted while working...\n");
      return;
   };
}

void VisualLearner::runComponent()
{
   debug("::runComponent");
   sleep(3011);

   WmAddressVector::iterator pwma;
   bool foundSomething;
   while (isRunning()) {
      // lockComponent();

      foundSomething = false;
      pwma = m_RecogTaskId_Queue.begin();
      if (pwma != m_RecogTaskId_Queue.end()) {
         cdl::WorkingMemoryAddress addr = *pwma;
         log("VL_Recognition Request addr popped: %s", descAddr(addr).c_str());
         VisualLearnerRecognitionTaskPtr pTaskData;
         try{
            // get the data from working memory
            pTaskData = getMemoryEntry<VisualLearnerRecognitionTask>(addr);
            foundSomething = true;

            // TODO: check if pTaskData is valid
            // TODO: lock pTaskData
            if (recogniseAttributes(pTaskData)) {
               debug("Will overwrite task: %s", descAddr(addr).c_str());
               // sleepComponent(100);
               overwriteWorkingMemory(addr, pTaskData);
            }
            // TODO: unlock pTaskData
         }
         catch(cast::DoesNotExistOnWMException){
            log("run: VisualLearnerRecognitionTask deleted while working...\n");
         };
         // TODO: catch other stuff from Matlab Proxy

         // Erase and move to the next point in the list.
         pwma = m_RecogTaskId_Queue.erase(pwma);
         log("VL_Recognition Task processed");
      } // while

      pwma = m_LearnTaskId_Queue.begin();
      if (pwma != m_LearnTaskId_Queue.end()) {
         cdl::WorkingMemoryAddress addr = *pwma;
         log("Learning Request addr popped: %s", descAddr(addr).c_str());
         VisualLearnerLearningTaskPtr pTaskData;
         try{
            // get the data from working memory
            pTaskData = getMemoryEntry<VisualLearnerLearningTask>(addr);
            foundSomething = true;

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
            log("VisualLearner.run: VisualLearnerLearningTask deleted while working...\n");
         };
         // TODO: catch other stuff from Matlab Proxy

         // Erase and move to the next point in the list.
         pwma = m_LearnTaskId_Queue.erase(pwma);
         log("VL_Learning Task processed");
      }

      if (! foundSomething) sleepComponent(100);

      // unlockComponent();
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
      log("VisualLearner.recogAttr: ProtoObject %s deleted while working...\n", descAddr(addr).c_str());
      return false;
   };
   // TODO: lock pProtoObj

   VL_recognise_attributes(*pProtoObj, _pTask->labels, _pTask->distribution);

   // TODO: unlock pProtoObj
   return true;

   // Now write this back into working memory, this will manage the memory for us.
   //debug("::recogniseAttributes writing");
   //overwriteWorkingMemory<SOI>(_pProtoObj->getAddress(), _pProtoObj);
   //log("Added a new AttrObject.");
   //unlockEntry(_pProtoObj.address);
}

bool VisualLearner::updateModel(VisualLearnerLearningTaskPtr _pTask)
{
   debug("::updateModel");

   cdl::WorkingMemoryAddress addr;
   addr.subarchitecture = string(getSubarchitectureID());
   addr.id = _pTask->protoObjectId;
   ProtoObjectPtr pProtoObj;
   try{
      pProtoObj = getMemoryEntry<ProtoObject>(addr);
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner.updateAttr: ProtoObject %s deleted while working...\n", descAddr(addr).c_str());
      return false;
   };

   VL_update_model(*pProtoObj, _pTask->labels, _pTask->distribution);

   return true;
}

} // namespace
