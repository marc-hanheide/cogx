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
         new MemberFunctionChangeReceiver<VisualLearner>(this, &VisualLearner::onAddRecognitionTask));
}

void VisualLearner::onAddRecognitionTask(const cdl::WorkingMemoryChange & _wmc)
{
   debug("::onAddRecognitionTask");
   string type(_wmc.type);
   string id(_wmc.address.id);

   log("Added: " + type + ", id: " + id);
   VisualLearnerRecognitionTaskPtr pTask;
   try {
      // get the data from working memory
      // pTask = getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
      m_RequestIdQueue.push_back(_wmc.address);
      log("Request addr pushed: %s", descAddr(_wmc.address).c_str());
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner.OnAdd: VisualLearnerRecognitionTask deleted while working...\n");
      return;
   };


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
         // lockComponent();

         // check (synchronised) joke queue
         WmAddressVector::iterator pwma = m_RequestIdQueue.begin();

         // see what's in there
         while (pwma != m_RequestIdQueue.end()) {
            cdl::WorkingMemoryAddress addr = *pwma;
            log("Request addr popped: %s", descAddr(addr).c_str());
            VisualLearnerRecognitionTaskPtr pTaskData;
            try{
               // get the data from working memory
               pTaskData = getMemoryEntry<VisualLearnerRecognitionTask>(addr);

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
               log("VisualLearner.run: VisualLearnerRecognitionTask deleted while working...\n");
            };
            // TODO: catch other stuff from Matlab Proxy

            // Erase and move to the next point in the list.
            pwma = m_RequestIdQueue.erase(pwma);
            log("VL_Recognition Task processed");
         } // while

         // unlockComponent();
      } // if
   } // while
} // VisualLearner::runComponent

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

   // AttrObjectPtr pAttrObject = new AttrObject(); // will not be added to WM

   VL_recognise_attributes(*pProtoObj, _pTask->colorLabel, _pTask->colorDistr);

   /*
   // copy from pAttrObject to _pTask
   _pTask->colorLabel.clear();
   _pTask->colorDistr.clear();
   _pTask->colorLabel.push_back("demo");
   _pTask->colorDistr.push_back(2.0);
   debug("Copy results from pAttrObject to _pTask");
   vector<string>::const_iterator pstr;
   for( pstr = pAttrObject->colorLabel.begin(); pstr != pAttrObject->colorLabel.end(); pstr++) {
      _pTask->colorLabel.push_back(string(*pstr));
   }
   vector<double>::const_iterator pdbl;
   for( pdbl = pAttrObject->colorDistr.begin(); pdbl != pAttrObject->colorDistr.end(); pdbl++) {
      _pTask->colorDistr.push_back(*pdbl);
   }
   // delete pAttrObject;
   debug("COPIED");
   */
   // TODO: unlock pProtoObj
   return true;

   // Now write this back into working memory, this will manage the memory for us.
   //debug("::recogniseAttributes writing");
   //overwriteWorkingMemory<SOI>(_pProtoObj->getAddress(), _pProtoObj);
   //log("Added a new AttrObject.");
   //unlockEntry(_pProtoObj.address);
} // FeatureSupport::extractFeatures

} // namespace
