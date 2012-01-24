#include "VisualLearner.h"
#include "VisualLearnerProxy.h"
#include "AffordanceLearnerProxy.h"

#include "../../VisionUtils.h"

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
#if FEAT_FAKE_LEARNER
  ss << "<h1>Using a fake VisualLearner</h1>";
  ss << "To use the real VisualLearner it has to be rebuilt with CMake option:<br />";
  ss << "<pre style='margin-left: 1em;'>BUILD_SA_VISION_VISLEARNER_FAKE=OFF</pre>";
  setActiveHtml("VisLearner.FAKE", "info", ss.str());
#else
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
#endif

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
      class CCmd:
        public cogx::VisionCommandNotifier<VisualLearnerRecognitionTask, VisualLearnerRecognitionTaskPtr>
      {
        public:
          CCmd(cast::WorkingMemoryReaderComponent* pReader)
            : cogx::VisionCommandNotifier<VisualLearnerRecognitionTask, VisualLearnerRecognitionTaskPtr>(pReader) {}
        protected:
          virtual void doFail() { pcmd->status = VisionData::VCFAILED; }
          virtual void doSucceed() { pcmd->status = VisionData::VCSUCCEEDED; }
      } cmd(this);

      pwma = newRecogRqs.begin();
      cdl::WorkingMemoryAddress addr = *pwma;
      newRecogRqs.erase(pwma);

      if (cmd.read(addr)) {
        log("VL_Recognition Request addr popped: %s", descAddr(addr).c_str());
        try {
          if (recogniseAttributes(cmd.pcmd))
            cmd.succeed();
          else
            cmd.fail();
        }
        catch(...) {
          log("VL_Recognition: SOMETHING WENT WRONG");
          cmd.fail();
        }
        log("VL_Recognition Task processed");
      }
    }

    if (newLearnRqs.size() > 0) {
      class CCmd:
        public cogx::VisionCommandNotifier<VisualLearningTask, VisualLearningTaskPtr>
      {
        public:
          CCmd(cast::WorkingMemoryReaderComponent* pReader)
            : cogx::VisionCommandNotifier<VisualLearningTask, VisualLearningTaskPtr>(pReader) {}
        protected:
          virtual void doFail() { pcmd->status = VisionData::VCFAILED; }
          virtual void doSucceed() { pcmd->status = VisionData::VCSUCCEEDED; }
      } cmd(this);

      pwma = newLearnRqs.begin();
      cdl::WorkingMemoryAddress addr = *pwma;
      newLearnRqs.erase(pwma);

      if (cmd.read(addr)) {
        log("Learning Request addr popped: %s", descAddr(addr).c_str());
        try {
          if (updateModel(cmd.pcmd))
            cmd.succeed();
          else
            cmd.fail();
        }
        catch(...) {
          log("VL_Learning: SOMETHING WENT WRONG");
          cmd.fail();
        }
        log("VL_Learning Task processed");
      }
    }

    if (newAffordanceRqs.size() > 0) {
      class CCmd:
        public cogx::VisionCommandNotifier<AffordanceRecognitionTask, AffordanceRecognitionTaskPtr>
      {
        public:
          CCmd(cast::WorkingMemoryReaderComponent* pReader)
            : cogx::VisionCommandNotifier<AffordanceRecognitionTask, AffordanceRecognitionTaskPtr>(pReader) {}
        protected:
          virtual void doFail() { pcmd->status = VisionData::VCFAILED; }
          virtual void doSucceed() { pcmd->status = VisionData::VCSUCCEEDED; }
      } cmd(this);

      pwma = newAffordanceRqs.begin();
      cdl::WorkingMemoryAddress addr = *pwma;
      newAffordanceRqs.erase(pwma);

      if (cmd.read(addr)) {
        log("Affordance Request addr popped: %s", descAddr(addr).c_str());
        try {
          if (recogniseAffordance(cmd.pcmd))
            cmd.succeed();
          else
            cmd.fail();
        }
        catch(...) {
          log("VL_AffordanceRecognition: SOMETHING WENT WRONG");
          cmd.fail();
        }
        log("VL_AffordanceRecognition Task processed");
      }
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
  ProtoObjectPtr pProtoObj;
  try{
    cdl::WorkingMemoryPointerPtr pomp = _pTask->protoObjectAddr;
    addr = pomp->address;
    if (pomp->type == cast::typeName<ProtoObject>()) {
      pProtoObj = getMemoryEntry<ProtoObject>(pomp->address);
    }
    if (! pProtoObj.get()) {
      log("VisualLearner.recogAttr: No ProtoObject in task (protoObjectAddr not specified)");
      return false;
    }
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

  cdl::WorkingMemoryAddress addr;
  ProtoObjectPtr pProtoObj;
  try{
    cdl::WorkingMemoryPointerPtr pomp = _pTask->protoObjectAddr;
    addr = pomp->address;
    if (pomp->type == cast::typeName<ProtoObject>()) {
      pProtoObj = getMemoryEntry<ProtoObject>(pomp->address);
    }
    if (! pProtoObj.get()) {
      log("VisualLearner.recogAffordance: No ProtoObject in task (protoObjectAddr not specified)");
      return false;
    }
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
  VisualObjectPtr pVisObj;
  try{
    cdl::WorkingMemoryPointerPtr vomp = _pTask->visualObjectAddr;
    addr = vomp->address;
    if (vomp->type == cast::typeName<VisualObject>()) {
      pVisObj = getMemoryEntry<VisualObject>(vomp->address);
    }
    if (! pVisObj.get()) {
      log("VisualLearner.updateAttr: No VisualObject in task (visualObjectAddr not specified)");
      return false;
    }
  }
  catch(cast::DoesNotExistOnWMException){
    log("VisualLearner.updateAttr: VisualObject %s deleted while working...", descAddr(addr).c_str());
    return false;
  };

  ProtoObjectPtr pProtoObj;
  try{
    cdl::WorkingMemoryPointerPtr pomp = pVisObj->protoObject;
    addr = pomp->address;
    if (pomp->type == cast::typeName<ProtoObject>()) {
      pProtoObj = getMemoryEntry<ProtoObject>(pomp->address);
    }
    if (! pProtoObj.get()) {
      log("VisualLearner.updateAttr: VisualObject is not visible (protoObject not specified)");
      return false;
    }
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
          if (pStatus.get()) {
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
      if (! pStatus) {
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
// vim: set sw=2 ts=8 sts=4 et :vim
