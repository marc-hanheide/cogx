#include "VisualLearner.h"
#include "VisualLearnerProxy.h"
#include "AffordanceLearnerProxy.h"

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

#ifdef FEAT_VISUALIZATION
   m_display.configureDisplayClient(_config);
#endif
}

void VisualLearner::start()
{
   debug("::start");
   ManagedComponent::start();

   addChangeFilter(createGlobalTypeFilter<VisualLearnerRecognitionTask>(cdl::ADD),
         new MemberFunctionChangeReceiver<VisualLearner>(this, &VisualLearner::onAdd_RecognitionTask));

   addChangeFilter(createGlobalTypeFilter<VisualLearningTask>(cdl::ADD),
         new MemberFunctionChangeReceiver<VisualLearner>(this, &VisualLearner::onAdd_LearningTask));

   addChangeFilter(createGlobalTypeFilter<AffordanceRecognitionTask>(cdl::ADD),
         new MemberFunctionChangeReceiver<VisualLearner>(this, &VisualLearner::onAdd_AffordanceTask));

#ifdef FEAT_VISUALIZATION
   m_display.connectIceClient(*this);
   m_display.installEventReceiver();
   m_display.createViews();
#endif
}

#ifdef FEAT_VISUALIZATION
#define ID_HTML_OBJECT "VisLearner.htm.LoadModel"
#define IDCHUNK_MODEL_BUTTONS "model.buttons"

void VisualLearner::CMyDisplayClient::createViews()
{
   std::ostringstream ss;
   const int nModels = 3;
   const string models[nModels] = {
      string("mC1"),
      string("mC2"),
      string("mC3") };

   for (int i=0; i<nModels; i++) {
      ss << "<input type='button' value='Load model " << models[i] << "' "
         << "@@ONCLICK@@('" << models[i] << "');\" /><br>";
   }
   
   //ss << ": <select name='" << IDC_FORM_OBJECT_NAME << "' 
   //   << " onchange=\"CogxJsSendValue('@@FORMID@@','" << ID_CMD_OBJECT_RELOAD << "','"
   //   << IDC_FORM_OBJECT_NAME << "');\" >";
   //for(int i = 0; i < pSim->m_objectNames.size(); i++) {
   //   ss << "<option>" << pSim->m_objectNames[i] << "</option>";
   //}
   //ss << "</select>";

   setActiveHtml(ID_HTML_OBJECT, IDCHUNK_MODEL_BUTTONS, ss.str());
}

void VisualLearner::CMyDisplayClient::handleEvent(const Visualization::TEvent &event)
{
   if (event.type == Visualization::evHtmlOnClick) {
      log("evHtmlOnClick on " + event.sourceId + " (" + event.objectId + ":" + event.partId + ")");
      string fname = event.sourceId + ".mat";
      log("going to load model " + fname);
      matlab::VL_LoadAvModels_from_configured_dir(fname.c_str());
   }
}
#endif

void VisualLearner::onAdd_RecognitionTask(const cdl::WorkingMemoryChange & _wmc)
{
   debug("::onAdd_RecognitionTask");
   string type(_wmc.type);
   string id(_wmc.address.id);

   log("Added: " + type + ", id: " + id);
   //VisualLearnerRecognitionTaskPtr pTask;
   try {
      // get the data from working memory
      // pTask = getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
      {
         IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_RrqMonitor);
         m_RecogTaskId_Queue.push_back(_wmc.address);
         m_RrqMonitor.notify();
      }
      log("VL RecognitionTask addr pushed: %s", descAddr(_wmc.address).c_str());
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner.OnAdd: VL_RecognitionTask deleted while working...");
      return;
   };
}

void VisualLearner::onAdd_AffordanceTask(const cdl::WorkingMemoryChange & _wmc)
{
   debug("::onAdd_AffordanceTask");
   string type(_wmc.type);
   string id(_wmc.address.id);

   log("Added: " + type + ", id: " + id);
   //AffordanceRecognitionTaskPtr pTask;
   try {
      // get the data from working memory
      // pTask = getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
      {
         IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_RrqMonitor);
         m_AffordanceTaskId_Queue.push_back(_wmc.address);
         m_RrqMonitor.notify();
      }
      log("AffordanceTask addr pushed: %s", descAddr(_wmc.address).c_str());
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner.OnAdd: AffordanceTask deleted while working...");
      return;
   };
}

void VisualLearner::onAdd_LearningTask(const cdl::WorkingMemoryChange & _wmc)
{
   debug("::onAdd_LearningTask");
   string type(_wmc.type);
   string id(_wmc.address.id);

   log("Added: " + type + ", id: " + id);
   //VisualLearningTaskPtr pTask;
   try {
      // get the data from working memory
      // pTask = getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
      {
         IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_RrqMonitor);
         m_LearnTaskId_Queue.push_back(_wmc.address);
         m_RrqMonitor.notify();
      }
      log("VL LearningTask addr pushed: %s", descAddr(_wmc.address).c_str());
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner.OnAdd: VL_LearningTask deleted while working...");
      return;
   };
}

void VisualLearner::runComponent()
{
   debug("::runComponent");
   sleepComponent(1011);

   if (m_ClfConfigFile.size() > 0)
      matlab::VL_setClfStartConfig(m_ClfConfigFile.c_str());

   updateWmModelStatus();

   TWmAddressVector::iterator pwma;
   while (isRunning()) {

      TWmAddressVector newRecogRqs;
      TWmAddressVector newLearnRqs;
      TWmAddressVector newAffordanceRqs;
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
         newAffordanceRqs = m_AffordanceTaskId_Queue;
         m_AffordanceTaskId_Queue.clear();
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

            // XXX: check if pTaskData is valid
            // XXX: lock pTaskData in WM
            if (recogniseAttributes(pTaskData)) {
               debug("Will overwrite task: %s", descAddr(addr).c_str());
               // sleepComponent(100);
               try {
                  overwriteWorkingMemory(addr, pTaskData);
               }
               catch (cast::ConsistencyException) {
                  log("Learner/Recognition Task overwritten elsewhere: %s", descAddr(addr).c_str());
                  // The task was changed in WM, but we don't care
                  VisualLearnerRecognitionTaskPtr pData = getMemoryEntry<VisualLearnerRecognitionTask>(addr);
                  overwriteWorkingMemory(addr, pTaskData);
               }
            }
            // XXX: unlock pTaskData in WM
         }
         catch(cast::DoesNotExistOnWMException){
            log("run: VisualLearnerRecognitionTask deleted while working...");
         };
         // XXX: catch other stuff from Matlab Proxy

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

            // XXX: check if pTaskData is valid
            // XXX: lock pTaskData
            if (updateModel(pTaskData)) {
               debug("Will overwrite task: %s", descAddr(addr).c_str());
               // sleepComponent(100);
               try {
                  overwriteWorkingMemory(addr, pTaskData); // Not changed, but write to notify owner!
               }
               catch (cast::ConsistencyException) {
                  log("Learner/Learning Task overwritten elsewhere: %s", descAddr(addr).c_str());
                  // The task was changed in WM, but we don't care
                  VisualLearningTaskPtr pData = getMemoryEntry<VisualLearningTask>(addr);
                  overwriteWorkingMemory(addr, pTaskData);
               }
            }
            // XXX: unlock pTaskData
         }
         catch(cast::DoesNotExistOnWMException){
            log("VisualLearner.run: VisualLearnerLearningTask deleted while working...");
         };
         // XXX: catch other stuff from Matlab Proxy

         log("VL_Learning Task processed");
      }

      if (newAffordanceRqs.size() > 0) {
         pwma = newAffordanceRqs.begin();
         cdl::WorkingMemoryAddress addr = *pwma;
         newAffordanceRqs.erase(pwma);

         log("Affordance Request addr popped: %s", descAddr(addr).c_str());
         AffordanceRecognitionTaskPtr pTaskData;
         try{
            // get the data from working memory
            pTaskData = getMemoryEntry<AffordanceRecognitionTask>(addr);

            // XXX: check if pTaskData is valid
            // XXX: lock pTaskData in WM
            if (recogniseAffordance(pTaskData)) {
              debug("Will overwrite task: %s", descAddr(addr).c_str());
              // sleepComponent(100);
              try {
                 overwriteWorkingMemory(addr, pTaskData);
              }
              catch (cast::ConsistencyException) {
                 log("Affordance Task overwritten elsewhere: %s", descAddr(addr).c_str());
                 // The task was changed in WM, but we don't care
                 AffordanceRecognitionTaskPtr pData = getMemoryEntry<AffordanceRecognitionTask>(addr);
                 overwriteWorkingMemory(addr, pTaskData);
              }
            }
            // XXX: unlock pTaskData in WM
         }
         catch(cast::DoesNotExistOnWMException){
            log("run: AffordanceRecognitionTask deleted while working...");
         };
         // XXX: catch other stuff from Matlab Proxy

         log("VL_Recognition Task processed");
      } // while

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

   cdl::WorkingMemoryAddress addr = _pTask->protoObjectAddr;
   ProtoObjectPtr pProtoObj;
   try{
      pProtoObj = getMemoryEntry<ProtoObject>(addr);
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner.recogAttr: ProtoObject %s deleted while working...", descAddr(addr).c_str());
      return false;
   };
   // XXX: lock pProtoObj

   matlab::VL_recognise_attributes(*pProtoObj, _pTask->labels, _pTask->labelConcepts,
         _pTask->distribution, _pTask->gains);

   // XXX: unlock pProtoObj
   return true;

   // Now write this back into working memory, this will manage the memory for us.
   //debug("::recogniseAttributes writing");
   //overwriteWorkingMemory<SOI>(_pProtoObj->getAddress(), _pProtoObj);
   //log("Added a new AttrObject.");
   //unlockEntry(_pProtoObj.address);
}

bool VisualLearner::recogniseAffordance(AffordanceRecognitionTaskPtr _pTask)
{
   debug("::recogniseAffordance");
   //try{
   //   lockEntry(_pProtoObj.address, cdl::LOCKEDODR);
   //}
   //catch(cast::DoesNotExistOnWMException){
   //   printf("VisualLearner: ProtoObject deleted while working...\n");
   //   return;
   //};

   // get the SOI data nah: this is naughty, but as we're overwriting
   // the soi in wm anyway it's not too bad

   cdl::WorkingMemoryAddress addr = _pTask->protoObjectAddr;
   ProtoObjectPtr pProtoObj;
   try{
      pProtoObj = getMemoryEntry<ProtoObject>(addr);
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner.recogAffordance: ProtoObject %s deleted while working...", descAddr(addr).c_str());
      return false;
   };
   // XXX: lock pProtoObj

   matlab::AL_affordance_recognise(*pProtoObj, _pTask->affordance);

   // XXX: unlock pProtoObj
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

   // Introspect, write to VisualConceptModelStatus
   updateWmModelStatus();

   return true;
}


// The VisualConceptModelStatus for the VisualLearner is represented with 2 entries in WM.
// One entry is for colors, the other for shapes.
// The function overwrites both erntries every time it is called.
// If it fails to read the current status from WM, it creates a new status entry.
void VisualLearner::updateWmModelStatus()
{
   struct _l_ {
      // read or create
      static void getWmStatus(VisualLearner* pComponent,
            VisualConceptModelStatusPtr& pStatus, cdl::WorkingMemoryAddress& addr)
      {
         if (addr.id.length() != 0 && addr.subarchitecture.length() != 0) {
            try{
               pStatus = pComponent->getMemoryEntry<VisualConceptModelStatus>(addr);
               if (pStatus != NULL) {
                  pComponent->debug("VisualConceptModelStatus read ... %s", descAddr(addr).c_str());
                  return;
               }
            }
            catch(cast::DoesNotExistOnWMException){
               pComponent->log("VisualConceptModelStatus was deleted unexpectedly ...");
            };
         }

         addr.id = "";
         addr.subarchitecture = "";
         pStatus = new VisualConceptModelStatus();
      }
      // overwrite or add
      static void writeWmStatus(VisualLearner* pComponent,
            VisualConceptModelStatusPtr& pStatus, cdl::WorkingMemoryAddress& addr)
      {
         if (pStatus == NULL) {
            pComponent->log("Trying to write NULL VisualConceptModelStatusPtr to WM ... %s",
                  descAddr(addr).c_str());
            return;
         }

         //pComponent->log("Writing: VisualConceptModelStatus ...%s", descAddr(addr).c_str());
         if (addr.id.length() != 0 && addr.subarchitecture.length() != 0) {
            try{
               pComponent->overwriteWorkingMemory(addr, pStatus);
               pComponent->log("VisualConceptModelStatus overwritten ... %s", descAddr(addr).c_str());
               return;
            }
            catch(cast::DoesNotExistOnWMException){
               pComponent->log("VisualConceptModelStatus was deleted unexpectedly ...");
            };
         }

         addr.id = pComponent->newDataID();
         addr.subarchitecture = pComponent->getSubarchitectureID();
         pComponent->addToWorkingMemory(addr, pStatus);
         pComponent->log("VisualConceptModelStatus added ... %s", descAddr(addr).c_str());
      }
   };

   debug("::updateWmModelStatus");
   VisualConceptModelStatusPtr stColor, stShape;

   _l_::getWmStatus(this, stColor, m_addrColorStatus);
   stColor->concept = "color";
   _l_::getWmStatus(this, stShape, m_addrShapeStatus);
   stShape->concept = "shape";

   vector<string> labels;
   vector<int> labelConcepts;
   vector<double> gains;
   matlab::VL_introspect(labels, labelConcepts, gains);

   vector<string>::const_iterator plabel = labels.begin();
   vector<int>::const_iterator pconcpt = labelConcepts.begin();
   vector<double>::const_iterator pgain = gains.begin();
   for(; plabel != labels.end() && pconcpt != labelConcepts.end() && pgain != gains.end();
         plabel++, pconcpt++, pgain++)
   {
      // Concept mapping is done in Matlab
      if (*pconcpt == 1){ // color concept
         stColor->labels.push_back(*plabel);
         stColor->gains.push_back(*pgain);
      }
      else if (*pconcpt == 2) { // shape concept
         stShape->labels.push_back(*plabel);
         stShape->gains.push_back(*pgain);
      }
      else {
         println(" *** VisualLearner/updateWmModelStatus Invalid concept ID: %d", *pconcpt);
      }
   }

   _l_::writeWmStatus(this, stColor, m_addrColorStatus);
   _l_::writeWmStatus(this, stShape, m_addrShapeStatus);

}

} // namespace
