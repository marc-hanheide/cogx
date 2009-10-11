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
   ManagedComponent::start();
   debug("::start");

   addChangeFilter(createLocalTypeFilter<VisualLearnerRecognitionTask>(cdl::ADD),
         new MemberFunctionChangeReceiver<VisualLearner>(this, &VisualLearner::onNewRecognitionTask));
}

void VisualLearner::stop()
{
   ManagedComponent::stop();
   log("!!!STOP CALL!!!");
   TermVisualLearnerLib();
}

void VisualLearner::onNewRecognitionTask(const cdl::WorkingMemoryChange & _wmc)
{
   debug("::onNewRecognitionTask");
   string type(_wmc.type);

   log(type + " added");

   // get the id of the changed entry
   string id(_wmc.address.id);

   log("Task id: " + id);
   VisualLearnerRecognitionTaskPtr pTaskData;
   try{
      // get the data from working memory
      pTaskData = getWorkingMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address)->getData();
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner: VisualLearnerRecognitionTask deleted while working...\n");
      return;
   };

   if (pTaskData != NULL) {
      m_RequestIdQueue.push_back(_wmc.address);
      // proposeTask(component, taskID, "s");
      log("done");
   } // if

} // VisualLearner::WorkingMemoryChange


void VisualLearner::runComponent()
{
   debug("::runComponent");
   while (isRunning()) {
      // do nothing for a while
      sleepComponent(20);

      // must check that we're still running after sleep
      if (isRunning()) {
         //prevent external access
         lockComponent();

         // check (synchronised) joke queue
         WmAddressVector::iterator pwma = m_RequestIdQueue.begin();

         // see what's in there
         while (pwma != m_RequestIdQueue.end()) {
            log("Update another ProtoObject.");
            VisualLearnerRecognitionTaskPtr pTaskData;
            try{
               // get the data from working memory
               pTaskData = getWorkingMemoryEntry<VisualLearnerRecognitionTask>(*pwma)->getData();

               // TODO: check if pTaskData is valid
               recogniseAttributes(pTaskData);
            }
            catch(cast::DoesNotExistOnWMException){
               log("VisualLearner: VisualLearnerRecognitionTask deleted while working...\n");
            };

            // Erase and move to the next point in the list.
            pwma = m_RequestIdQueue.erase(pwma);
            log("ProtoObject processed");
         } // while

         unlockComponent();
      } // if
   } // while
} // VisualLearner::runComponent

void VisualLearner::recogniseAttributes(VisualLearnerRecognitionTaskPtr _pTask)
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

   //FIX ME --- probably can optimise
   AttrObjectPtr pAttrObject = new AttrObject();
   //ProtoObjectPtr pProtoObject = (_pData->getData()); //*_pData->getData());

   //VL_recognise_attributes(*pAttrObject, _pProtoObj->getData());

   // Now write this back into working memory, this will manage the memory for us.
   //debug("::recogniseAttributes writing");
   //overwriteWorkingMemory<SOI>(_pProtoObj->getAddress(), _pProtoObj);
   //log("Added a new AttrObject.");
   //unlockEntry(_pProtoObj.address);
} // FeatureSupport::extractFeatures

} // namespace
