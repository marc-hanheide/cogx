#include "FeatureExtractor.h"
#include "FeatureLearningProxy.h"
#include <vision/VisionGoals.h>
#include <vision/idl/Vision.hh>
#include <cast/architecture/ChangeFilterFactory.hpp>

extern "C"
{
   FrameworkProcess* newComponent(const string &_id) {
      InitFeatureLearningLib();
      return new FeatureExtractor(_id);
   }
}

FeatureExtractor::FeatureExtractor(const string &_id) :
      WorkingMemoryAttachedComponent(_id),
      ManagedProcess(_id)
{
   m_pProposedProcessing = new ROIMap();
   m_pROIs = new ROIVector();
} // FeatureExtractor::FeatureExtractor

FeatureExtractor::~FeatureExtractor()
{
   delete m_pProposedProcessing;
   delete m_pROIs;
   // delete this->bla_;
} // FeatureExtractor::~FeatureExtractor

void FeatureExtractor::configure(map<string, string>& _config)
{
   // first let the base class configure itself
   ManagedProcess::configure(_config);

   //~ // **orig**: std::string matlabPath = "addpath('" + _config["-m"] + "')";
   //~ // **orig**: this->eval(matlabPath.c_str());
   //~ // **orig**: this->log_matlab();

   //~ // **orig**: matlabPath = "addPaths('" + _config["-m"] + "')";
   //~ // **orig**: this->eval(matlabPath.c_str());
   //~ // **orig**: this->log_matlab();

   //~ // **orig**: this->eval("p=path");
   //~ // **orig**: this->log_matlab();
} // FeatureExtractor::configure

void FeatureExtractor::start()
{
   ManagedProcess::start();
   debug("::start");

   addChangeFilter(createLocalTypeFilter<ROI>(cdl::ADD),
         new MemberFunctionChangeReceiver<FeatureExtractor>(this, &FeatureExtractor::newROI));

}

void FeatureExtractor::stop()
{
   ManagedProcess::stop();
   log("!!!STOP CALL!!!");
   TermFeatureLearningLib();
}

void FeatureExtractor::newROI(
   const cdl::WorkingMemoryChange & _wmc)
{

  //HACK FeatureExtractor cannot deal with ROIs that were created by
  //anything but the segmentor
  string roiSource(_wmc.m_src);
  if(roiSource != "segmentor") {
    log("ignoring ROI created by: " + roiSource);
    log("only accepting ROIs created by: segmentor");
    return;
  }


   debug("::newROI");
   string type(_wmc.m_type);

   log(type + " added");

   // get the id of the changed entry
   string id(_wmc.m_address.m_id);

   log("data id: " + id);

   // get the data from working memory
   shared_ptr<const CASTData<ROI> > pROIData =
      getWorkingMemoryEntry<ROI>(id);

   if (pROIData != NULL) {

      log("done");

      // Get a new ID for the task.
      string taskID = this->newTaskID();

      // Store the data we want to process for later.
      (*m_pProposedProcessing)[taskID] = pROIData;

      proposeInformationProcessingTask(taskID,
            VisionGoals::EXTRACT_FEATURES_TASK);
   } // if

} // FeatureExtractor::WorkingMemoryChange


void FeatureExtractor::taskAdopted(const string &_taskID)
{
   debug("::taskAdopted");
   ROIMap::iterator i = m_pProposedProcessing->find(_taskID);

   // If we have stored this goal earlier.
   if (i != m_pProposedProcessing->end()) {
      // Store the ROI to process later.
      m_pROIs->push_back(i->second);
      // And take it out of the prosed list.
      m_pProposedProcessing->erase(i);
   } else {
      log("oh, this is my goal, but I have no data: "
            + _taskID);
   } // if - else

   // and now we're finished, tell the goal manager that the task is
   // over successfully (assuming it is... naughty!)
   taskComplete(_taskID, cdl::PROCESSING_COMPLETE_SUCCESS);
} // FeatureExtractor::taskAdopted

void FeatureExtractor::taskRejected(const string &_taskID)
{
   debug("::taskRejected");
   println(":(");
   ROIMap::iterator i = m_pProposedProcessing->find(_taskID);

   // if we have stored this goal earlier
   if (i != m_pProposedProcessing->end()) {
      // remove it
      m_pProposedProcessing->erase(i);
   } // if
} // FeatureExtractor::taskRejected

void FeatureExtractor::runComponent()
{
  debug("::runComponent");
  while (m_status == STATUS_RUN) {
      // do nothing for a while
      sleepProcess(20);

      // must check that we're still running after sleep
      if (m_status == STATUS_RUN) {
         //prevent external access
         lockProcess();

         // check (synchronised) joke queue
         ROIVector::iterator i = m_pROIs->begin();

         // see what's in there
         while (i != m_pROIs->end()) {
            log("Update another ROI.");
            this->extractFeatures(*i);
            // Do something with ROI.
            //    quip(*i);

            // Erase and move to the next point in the list.
            i = m_pROIs->erase(i);
            log("ROI updated");
         } // while

         unlockProcess();
      } // if
   } // while
} // FeatureExtractor::runComponent

void FeatureExtractor::extractFeatures(shared_ptr<const CASTData<ROI> > _pData)
{
  lockEntry(_pData->getID(), cdl::LOCKED_ODR);
  debug("::extractFeatures");
  
  // get the ROI data nah: this is naughty, but as we're overwriting
  //the roi in wm anyway it's not too bad
  
  //FIX ME --- probably can optimise
  ROI *pROI = new ROI(*_pData->getData());
  
  FE_extract_features(*pROI);
   
  // Now write this back into working memory, this will manage the memory for us.
  debug("::extractFeatures writing");
  overwriteWorkingMemory<ROI>(_pData->getID(), pROI, cdl::BLOCKING);
  this->log("Updated ROI features.");
  unlockEntry(_pData->getID());

} // FeatureSupport::extractFeatures

//void FeatureExtractor::quip(CASTData<Vision::Joke> *_pData) {
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
