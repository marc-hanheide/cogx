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
   debug("::start");
   ManagedComponent::start();

   addChangeFilter(createLocalTypeFilter<VisualLearnerRecognitionTask>(cdl::ADD),
         new MemberFunctionChangeReceiver<VisualLearner>(this, &VisualLearner::onAddRecognitionTask));
}

void VisualLearner::stop()
{
   log("!!!STOP CALL!!!");
   ManagedComponent::stop();
}

void VisualLearner::onAddRecognitionTask(const cdl::WorkingMemoryChange & _wmc)
{
   debug("::onAddRecognitionTask");
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
               pTaskData = getMemoryEntry<VisualLearnerRecognitionTask>(*pwma);

               // TODO: check if pTaskData is valid
               // TODO: lock pTaskData
               recogniseAttributes(pTaskData); // TODO: if recogn... then overwr...
               overwriteWorkingMemory(*pwma, pTaskData);
               // TODO: unlock pTaskData
            }
            catch(cast::DoesNotExistOnWMException){
               log("VisualLearner: VisualLearnerRecognitionTask deleted while working...\n");
            };
            // TODO: catch other stuff from Matlab Proxy

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

   cdl::WorkingMemoryAddress addr;
   addr.subarchitecture = getSubarchitectureID();
   addr.id = _pTask->protoObjectId;
   ProtoObjectPtr pProtoObj = getMemoryEntry<ProtoObject>(addr);
   // TODO: lock pProtoObj

   // FIXME --- probably can optimise
   AttrObjectPtr pAttrObject = new AttrObject(); // will not be added to WM

   VL_recognise_attributes(*pAttrObject, *pProtoObj);

   // copy from pAttrObject to _pTask
   _pTask->colorLabel.clear();
   _pTask->colorDistr.clear();
   _pTask->colorLabel.push_back("demo");
   _pTask->colorDistr.push_back(2.0);
   vector<string>::const_iterator pstr;
   for( pstr = pAttrObject->colorLabel.begin(); pstr != pAttrObject->colorLabel.end(); pstr++) {
      _pTask->colorLabel.push_back(*pstr);
   }
   vector<double>::const_iterator pdbl;
   for( pdbl = pAttrObject->colorDistr.begin(); pdbl != pAttrObject->colorDistr.end(); pdbl++) {
      _pTask->colorDistr.push_back(*pdbl);
   }
   // delete pAttrObject;
   // TODO: unlock pProtoObj

   // Now write this back into working memory, this will manage the memory for us.
   //debug("::recogniseAttributes writing");
   //overwriteWorkingMemory<SOI>(_pProtoObj->getAddress(), _pProtoObj);
   //log("Added a new AttrObject.");
   //unlockEntry(_pProtoObj.address);
} // FeatureSupport::extractFeatures

} // namespace
