#include "VisualLearner.h"
#include "VisualLearnerProxy.h"
//#include <vision/VisionGoals.h>
#include <VisionData.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::VisualLearner();
  }
}

VisualLearner::VisualLearner() :
      WorkingMemoryAttachedComponent(),
      ManagedComponent()
{
//   m_pProposedProcessing = new SOIMap();
   m_pSOIs = new SOIVector();
   
} // VisualLearner::VisualLearner

VisualLearner::~VisualLearner()
{
   delete m_pProposedProcessing;
   delete m_pSOIs;
   // delete this->bla_;
} // VisualLearner::~VisualLearner

/**
 * Some of the options:
 * -soisource ..Name of the soi source, the name of the segmentor component. Defaults to segmentor
 *              however, another common choice is soi.detector
 */
void VisualLearner::configure(map<string, string>& _config)
{
   // first let the base class configure itself
   ManagedComponent::configure(_config);

   //~ // **orig**: std::string matlabPath = "addpath('" + _config["-m"] + "')";
   //~ // **orig**: this->eval(matlabPath.c_str());
   //~ // **orig**: this->log_matlab();

   //~ // **orig**: matlabPath = "addPaths('" + _config["-m"] + "')";
   //~ // **orig**: this->eval(matlabPath.c_str());
   //~ // **orig**: this->log_matlab();

   //~ // **orig**: this->eval("p=path");
   //~ // **orig**: this->log_matlab();

//   cit=_config.find("-soisource");
//         if(cit!=_config.end()){
//  	   allowedSoiSource= cit->second;
//	 }else{
//	   allowedSoiSource=string("segmentor");
//	 }

} // VisualLearner::configure

void VisualLearner::start()
{
   ManagedComponent::start();
   debug("::start");

   addChangeFilter(createLocalTypeFilter<ProtoObject>(cdl::ADD),
         new MemberFunctionChangeReceiver<VisualLearner>(this, &VisualLearner::onNewProtoObject));

}

void VisualLearner::stop()
{
   ManagedComponent::stop();
   log("!!!STOP CALL!!!");
   TermFeatureLearningLib();
}

void VisualLearner::onNewProtoObject(const cdl::WorkingMemoryChange & _wmc)
{

  //HACK VisualLearner cannot deal with SOIs that were created by
  //anything but the segmentor
//  string soiSource(_wmc.src);
//  if(soiSource != allowedSoiSource) {
//    log("ignoring SOI created by: " + soiSource);
//    log("only accepting SOIs created by: segmentor");
//    return;
  }

   debug("::newProtoObject");
   string type(_wmc.type);

   log(type + " added");

   // get the id of the changed entry
   string id(_wmc.address.id);

   log("data id: " + id);
   ProtoObjectPtr pProtoObjectData;
   try{
      // get the data from working memory
      pProtoObjectData = getWorkingMemoryEntry<ProtoObject>(id);
   }
   catch(cast::DoesNotExistOnWMException){
      log("VisualLearner: ProtoObject deleted while working...\n");
      return;
   };

   if (pProtoObjectData != NULL) {

      log("done");

      // Get a new ID for the task.
      string taskID = this->newTaskID();
      string component = this->getComponentID();
/*
      // Store the data we want to process for later.
      (*m_pProposedProcessing)[taskID] = pSOIData;

      proposeTask(component, taskID, "s"); */
   } // if

} // VisualLearner::WorkingMemoryChange


void VisualLearner::taskAdopted(const string &_taskID)
{
   debug("::taskAdopted");
//   SOIMap::iterator i = m_pProposedProcessing->find(_taskID);
/*
   // If we have stored this goal earlier.
   if (i != m_pProposedProcessing->end()) {
      // Store the SOI to process later.
      m_pSOIs->push_back(i->second);
      // And take it out of the prosed list.
      m_pProposedProcessing->erase(i);
   } else {
      log("oh, this is my goal, but I have no data: "
            + _taskID);
   } // if - else

   // and now we're finished, tell the goal manager that the task is
   // over successfully (assuming it is... naughty!)
   taskComplete(_taskID, cast::cdl::ProcessingCompleteSuccess); */
} // VisualLearner::taskAdopted

void VisualLearner::taskRejected(const string &_taskID)
{
   debug("::taskRejected");
   println(":(");
//   SOIMap::iterator i = m_pProposedProcessing->find(_taskID);

   // if we have stored this goal earlier
   if (i != m_pProposedProcessing->end()) {
      // remove it
      m_pProposedProcessing->erase(i);
   } // if
} // VisualLearner::taskRejected

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
         SOIVector::iterator i = m_pProtoObjects->begin();

         // see what's in there
         while (i != m_pProtoObjects->end()) {
            log("Update another ProtoObject.");
            this->recogniseAttributes(*i);
            // Do something with SOI.
            //    quip(*i);

            // Erase and move to the next point in the list.
            i = m_pProtoObjects->erase(i);
            log("ProtoObject processed");
         } // while

         unlockComponent();
      } // if
   } // while
} // VisualLearner::runComponent

void VisualLearner::recogniseAttributes(ProtoObjectPtr _pData) //shared_ptr<const CASTData<SOI> > _pData)
{
   try{
      lockEntry(_pData->getID(), cdl::LOCKEDODR);
   }
   catch(cast::DoesNotExistOnWMException){
      printf("VisualLearner: ProtoObject deleted while working...\n");
      return;
   };
   debug("::recogniseAttributes");
  
   // get the SOI data nah: this is naughty, but as we're overwriting
   //the soi in wm anyway it's not too bad
  
   //FIX ME --- probably can optimise
   AttrObjectPtr pAttrObject = new AttrObject();
   //ProtoObjectPtr pProtoObject = (_pData->getData()); //*_pData->getData());
   
   VL_recognise_attributes(*pAttrObject, _pData->getData())
   //FE_extract_features(*pSOI);
   
   // Now write this back into working memory, this will manage the memory for us.
   debug("::recogniseAttributes writing");
   overwriteWorkingMemory<SOI>(_pData->getID(), pSOI);
   this->log("Added a new AttrObject.");
   unlockEntry(_pData->getID());
} // FeatureSupport::extractFeatures

//void VisualLearner::quip(CASTData<Vision::Joke> *_pData) {
//
// //get the joke data
// Vision::Joke *pJK = _pData->data();
// string setup(pJK->m_setup);
//
// // work out a smarty-pants punchline
// string punchline = generatePunchline(setup);
//
// // time it right
// println("*cough*");
// pJK->m_punchline = CORBA::string_dup(punchline.c_str());
//
// // now write this back into working memory, this will manage the
// // memory for us
// overwriteWorkingMemory(_pData->getID(),
//   VisionOntology::JOKE_TYPE, pJK);
//
// //delete data holder, but ensure it doesn't destroy the memory we've
// //just passed on
// _pData->data() = NULL;
// delete _pData;
//}
