#include "Recogniser.h"
#include "FeatureLearningProxy.h"
#include <vision/VisionGoals.h>
#include <vision/idl/Vision.hh>
#include <cast/architecture/ChangeFilterFactory.hpp>

extern "C"
{
   FrameworkProcess* newComponent(const string &_id) {
      InitFeatureLearningLib();
      return new Recogniser(_id);
   }
}

Recogniser::Recogniser(const string &_id) :
  WorkingMemoryAttachedComponent(_id), 
  ManagedProcess(_id),
  SceneObjectWriter(dynamic_cast<ManagedProcess&>(*this)) {

   this->objRoiIDs_ = new map<string, string>();
   this->recognitionTasks_ = new ROIMap();
   this->learnTasks_ = new LearnInstructionMap();

   // Get some knowledge ;).
   // this->eval("load mAVs4.mat");
} // Recogniser::Recogniser

Recogniser::~Recogniser()
{
   // TODO do the same for all maps!!!

   delete this->objRoiIDs_;
   delete this->recognitionTasks_;
   delete this->learnTasks_;
} // Recogniser::~Recogniser

void Recogniser::configure(map<string, string>& _config)
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

   map<string, string>::iterator cit = _config.find("--loadAV");
   if (cit != _config.end()) {
      m_AVModel = cit->second;
   } // if
   
   cit = _config.find("--matWin");
   if (cit != _config.end()) {
     if (cit->second == "true")
       m_matWindow = 1;
     else if (cit->second == "false")
       m_matWindow = 0;
   } 
   else {
     m_matWindow = 1;
   }// if
} // Recogniser::configure


void Recogniser::processROI(const cdl::WorkingMemoryChange& _wmChange)
{
   // processing.
   string roiID(_wmChange.m_address.m_id);

   shared_ptr<const CASTData<ROI> >pROIData =
      getWorkingMemoryEntry<ROI>(roiID);
   string objID(pROIData->getData()->m_objId);


   if (pROIData->getData()->m_features.data_.length() > 0
       && this->objRoiIDs_->find(objID) == this->objRoiIDs_->end()  ) {
      debug("Process ROI ID:" + roiID);

      (*this->objRoiIDs_)[objID] = roiID;

      // Propose new task.
      string taskID = this->newTaskID();

      // Store the ROI data for further processing (we need to wait for
      // task adoption before proceding).
      (*this->recognitionTasks_)[taskID] = pROIData;

      log("Propose recognition task for ROI ID:" + roiID);
      this->proposeInformationProcessingTask(taskID, VisionGoals::RECOGNISE_PROPERTIES_TASK);
   } else
      debug("ROI ID: " + roiID + " - Wait for features to be extracted");
}


void Recogniser::removeROI(const cdl::WorkingMemoryChange& _wmChange)
{

   string roiID(_wmChange.m_address.m_id);

   debug("Remove ROI ID:" + roiID);

   this->objRoiIDs_->erase(roiID);
}

void Recogniser::start()
{
   debug("::start");
   ManagedProcess::start();

   addChangeFilter(createLocalTypeFilter<ROI>(cdl::ADD),
         new MemberFunctionChangeReceiver<Recogniser>(this,
               &Recogniser::processROI));

   addChangeFilter(createLocalTypeFilter<ROI>(cdl::OVERWRITE),
         new MemberFunctionChangeReceiver<Recogniser>(this,
               &Recogniser::processROI));

   addChangeFilter(createLocalTypeFilter<ROI>(cdl::DELETE),
         new MemberFunctionChangeReceiver<Recogniser>(this,
               &Recogniser::removeROI));

   addChangeFilter(createLocalTypeFilter<LearnInstruction>(cdl::ADD),
         new MemberFunctionChangeReceiver<Recogniser>(this,
               &Recogniser::proposeLearnGoal));

} // Recogniser::start

void Recogniser::stop()
{
   ManagedProcess::stop();
   log("!!!STOP CALL!!!");
   TermFeatureLearningLib();
}

void Recogniser::taskAdopted(const string &_taskID)
{
   debug("::taskAdopted");
   ROIMap::iterator i = this->recognitionTasks_->find(_taskID);

   // If we have stored this goal earlier.
   if (i != this->recognitionTasks_->end()) {
      debug("Recognition task " + _taskID);
      this->recognise(i->second);
      this->recognitionTasks_->erase(i);

   } else {
      // Otherwise try to find the learning task.
      LearnInstructionMap::iterator iLI = this->learnTasks_->find(_taskID);

      if (iLI != this->learnTasks_->end()) {
         // Get the learn instruction.
         shared_ptr<const Vision::LearnInstruction> learnInstruction = iLI->second->getData();
         int val = learnInstruction->m_features[0].m_int;
         float conf = learnInstruction->m_features[0].m_confidence;

         this->println("Learning task < %s >: [ %i %f ]", _taskID.c_str(), val, conf);

         this->update(iLI->second);
         this->learnTasks_->erase(iLI);
      } // if - else
   } // if - else

   // and now we're finished, tell the goal manager that the task is
   // over successfully (assuming it is... naughty!)
   taskComplete(_taskID, cdl::PROCESSING_COMPLETE_SUCCESS);
} // Recogniser::taskAdopted

void Recogniser::taskRejected(const string &_taskID)
{
   // TODO add the ID of the ROI that was just rejected to the list of IDs
   // again.
} // Recogniser::taskRejected

void Recogniser::runComponent()
{
   debug("::runComponent");
   R_RunComponent(m_matWindow);
   this->loadAVModels();

   while (m_status == STATUS_RUN) {
      //sleep for a random amount of time
      sleep(1);

      // must check that we're still running after sleep
      if (m_status == STATUS_RUN) {
         //prevent external access
         lockProcess();

         unlockProcess();
      } // if
   } // while
} // Recogniser::runComponent


void Recogniser::recognise(shared_ptr<const CASTData<Vision::ROI> > _roiData)
{
   debug("::recognise");
   shared_ptr<const ROI> roi = _roiData->getData();
   debug("Recognise attributes from ROI ID: " + _roiData->getID());

   // Setup global variables.
   // Read the recogniser constants.
   // Add the features to the matlab engine.

   // Recognise atributes.
   RecognisedAttributes* attr = R_Recognise(*roi);
   attr->m_targetAddress = CORBA::string_dup(_roiData->getID().c_str());

   //load obj
   string objID(roi->m_objId);
   loadSceneObject(objID);

   // Add recognised attributes to the working memory.
//  this->addToWorkingMemory(this->newDataID(),
//      VisionOntology::RECOGNISED_ATTRIBUTES_TYPE, attr);

// TODO: RecognisedAttributes do not go to WM currently, they are resolved localy. Remove them from IDL or keep them for possible future use?

   int colour(getColour(attr));
   if (colour != -1) {
     setObjectColour(colour,1.0);     
   }

   int shape(getShape(attr));
   if (shape != -1) {
     setObjectShape(shape,1.0);     
   }

   int size(getSize(attr));
   if (size != -1) {
     setObjectSize(size,1.0);     
   }

   //write obj to working memory
   writeObject();

   // Remove the features from the matlab engine.
   //~ // **orig**: this->eval("clear f answ");
} // Recogniser::recognise

long Recogniser::getColour(Vision::RecognisedAttributes* _attr)
{
   for (unsigned i = 0; i < _attr->m_attributes.data_.length(); i++)
      if (_attr->m_attributes.data_[i] >= 1 && _attr->m_attributes.data_[i] <= 4)
         return (long) _attr->m_attributes.data_[i];

   return -1;
}

long Recogniser::getShape(Vision::RecognisedAttributes* _attr)
{
   debug("::getShape");
   for (unsigned i = 0; i < _attr->m_attributes.data_.length(); i++)
      if (_attr->m_attributes.data_[i] >= 7 && _attr->m_attributes.data_[i] <= 10)
         return (long) _attr->m_attributes.data_[i];

   return -1;
}

long Recogniser::getSize(Vision::RecognisedAttributes* _attr)
{
   debug("::getSize");
   for (unsigned i = 0; i < _attr->m_attributes.data_.length(); i++)
      if (_attr->m_attributes.data_[i] >= 5 && _attr->m_attributes.data_[i] <= 7)
         return (long) _attr->m_attributes.data_[i];

   return -1;
}


void Recogniser::proposeLearnGoal(const cdl::WorkingMemoryChange& _wmChange)
{
   debug("::proposeLearnGoal");
   log("Learning instruction received");

   string id(_wmChange.m_address.m_id);

   shared_ptr<const CASTData<Vision::LearnInstruction> > pLearnData =
      this->getWorkingMemoryEntry<Vision::LearnInstruction>(id);

   if (pLearnData) {

      // Now propose new task.
      string taskID = this->newTaskID();

      (*this->learnTasks_)[taskID] = pLearnData;

      this->proposeInformationProcessingTask(taskID,
            VisionGoals::LEARN_TASK);

   // TODO: Should probably delete LearnInstruction here.
   } // if
} // Recogniser::proposeLearnGoal


void Recogniser::update(shared_ptr<const CASTData<Vision::LearnInstruction> > _liData)
{
   debug("::update");
   // Get the learn instruction.
   shared_ptr<const Vision::LearnInstruction> learnInstruction = _liData->getData();
   
    // Get the ROI in question.
   string objID(learnInstruction->m_targetAddress);
   string objType(learnInstruction->m_type);

   debug("Object Type :" + objType);

   if (objType == typeName<SceneObject>()) {
      debug("Updating knowledge from information on object ID: " + objID);
      debug("which belongs to ROI ID: " + (*objRoiIDs_)[objID]);

      shared_ptr<const CASTTypedData<ROI> > pROI =
         getWorkingMemoryEntry<ROI>((*objRoiIDs_)[objID]);

      // Make sure not to update with non-existent ROI! This might occur when the
      // object is removed from the scene before the update takes place.
      if (pROI) R_Update(*learnInstruction, *(pROI->getData()));

      debug("Updated knowledge from object ID: " + objID);
      
      // Now propose new tasks for recognition of attributes in WM (only if the
      // learn instruction was initiated by the tutor).
      if (learnInstruction->m_tutorInitiated) {
         this->onRepresentationChange();
      } // if
   } // if
   else {
      log("Unknown object type: " + objID);
      return;
   }
} // Recogniser::update


void Recogniser::onRepresentationChange()
{
   debug("::onRepresentationChange");
   // Get the list of current ROIs in the working memory.
   std::vector<shared_ptr<const CASTData<ROI> > > roisInWM;
   this->getWorkingMemoryEntries<ROI>(0, roisInWM);

   // Add IDs of all ROIs to the list of those we need to re-recognise.
   for (std::vector<shared_ptr<const CASTData<Vision::ROI> > >::const_iterator
        wit = roisInWM.begin(), wit_e = roisInWM.end();
        wit != wit_e; wit++) {
      (*this->objRoiIDs_)[string((*wit)->getData()->m_objId)] = (*wit)->getID();
   } // for
} // Recogniser::onRepresentationChange

void Recogniser::loadAVModels()
{
   debug("::loadAVModels");
   if (!m_AVModel.empty()) {
      R_LoadAvModels(m_AVModel.c_str());
   } // if
} // Recogniser::loadAVModels
