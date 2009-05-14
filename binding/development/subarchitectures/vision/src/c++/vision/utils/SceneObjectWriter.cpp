#include "SceneObjectWriter.hpp"

//default useful namespaces, fix to reflect your own code
using namespace std;
using namespace Vision;
using namespace cast;
using namespace boost;


SceneObjectWriter::SceneObjectWriter(ManagedProcess& _component) :
  m_soCache(_component),
  m_component(_component),
  m_pCurrentObject(NULL),
  m_currentObjectID(""),
  m_potentiallyChanged(false) {

}



string
SceneObjectWriter::newSceneObject() 
  throw(SubarchitectureProcessException) {

  checkUnloaded();

  //no id
  m_currentObjectID = "";
  m_potentiallyChanged = false;

  m_pCurrentObject = new SceneObject();

  m_pCurrentObject->m_time = BALTTimer::getBALTTime();
  m_pCurrentObject->m_label.m_string = CORBA::string_dup("");
  m_pCurrentObject->m_label.m_confidence = 0.0;
  
  m_pCurrentObject->m_surfaces.length(0);

  m_pCurrentObject->m_ROIsMemoryIDs.length(0);
  
  m_pCurrentObject->m_color.m_int = Vision::UNKNOWN;
  m_pCurrentObject->m_color.m_confidence = 0.0;
  
  m_pCurrentObject->m_shape.m_int = Vision::UNKNOWN;
  m_pCurrentObject->m_shape.m_confidence = 0.0;
  
  m_pCurrentObject->m_size.m_int = Vision::UNKNOWN;
  m_pCurrentObject->m_size.m_confidence = 0.0;
  
  m_pCurrentObject->m_objtype.m_int= Vision::UNKNOWN;
  m_pCurrentObject->m_objtype.m_confidence= 0.0;
  
  m_pCurrentObject->m_pgshape.m_int=Vision::UNKNOWN;
  m_pCurrentObject->m_pgshape.m_confidence=0.0;

  m_pCurrentObject->m_recognisedClass.m_int=Vision::UNKNOWN;
  m_pCurrentObject->m_recognisedClass.m_confidence=0.0;

  m_pCurrentObject->m_objtype.m_int= Vision::UNDEF;
  m_pCurrentObject->m_objtype.m_confidence=0.0;

  m_pCurrentObject->m_activity.m_int=Vision::UNDEF;
  m_pCurrentObject->m_activity.m_confidence=0.0;

  m_pCurrentObject->m_game.m_int=Vision::UNDEF;
  m_pCurrentObject->m_game.m_confidence=0.0;

  m_pCurrentObject->m_managingProcess=CORBA::string_dup(m_component.getProcessIdentifier().c_str());

  m_currentObjectID = m_component.newDataID();

  m_isNew = true;

  return m_currentObjectID;
}

/**
 * Load existing object from wm and set as current object.
 */
void 
SceneObjectWriter::loadSceneObject(const std::string & _soID,
				   const cast::cdl::WorkingMemoryPermissions & _permissions) 
  throw(SubarchitectureProcessException) {

  checkUnloaded();

  m_loadedPermissions = _permissions;
  if(m_loadedPermissions != cast::cdl::UNLOCKED) {
    m_component.lockEntry(_soID,m_loadedPermissions);
  }


  //create new so from storage, inefficient but easy
  m_pCurrentObject = new SceneObject(m_soCache[_soID]);

  //set current id
  m_currentObjectID = _soID;

  m_isNew = false;
}

  
void 
SceneObjectWriter::addObjectROI(const string & _roiID)  
  throw(SubarchitectureProcessException) {

  checkLoaded();

  //create new list
  unsigned int newLength(m_pCurrentObject->m_ROIsMemoryIDs.length() + 1);
  cdl::WorkingMemoryIDList newList;
  newList.length(newLength);

  //populate it
  for(unsigned int i = 0; i < m_pCurrentObject->m_ROIsMemoryIDs.length(); ++i) {
    newList[i] = m_pCurrentObject->m_ROIsMemoryIDs[i];
  }

  //add new
  newList[newList.length()-1] = CORBA::string_dup(_roiID.c_str());

  //and store
  m_pCurrentObject->m_ROIsMemoryIDs = newList;  
  
  //double check
  assert(m_pCurrentObject->m_ROIsMemoryIDs.length() == newLength);
  assert(strcmp(m_pCurrentObject->m_ROIsMemoryIDs[m_pCurrentObject->m_ROIsMemoryIDs.length()-1],_roiID.c_str()) == 0);
}


void 
SceneObjectWriter::setObjectColour(const int & _i, const float & _confidence) 
  throw(SubarchitectureProcessException)  {
  checkLoaded();
  m_pCurrentObject->m_color.m_int = _i;
  m_pCurrentObject->m_color.m_confidence = _confidence;
  objectColourUpdated();
}

void 
SceneObjectWriter::setObjectShape(const int & _i, const float & _confidence) 
  throw(SubarchitectureProcessException)  {
  checkLoaded();
  m_pCurrentObject->m_shape.m_int = _i;
  m_pCurrentObject->m_shape.m_confidence = _confidence;
  objectShapeUpdated();
}

void 
SceneObjectWriter::setObjectSize(const int & _i, const float & _confidence) 
  throw(SubarchitectureProcessException)  {
  checkLoaded();
  m_pCurrentObject->m_size.m_int = _i;
  m_pCurrentObject->m_size.m_confidence = _confidence;
  objectSizeUpdated();
}

void 
SceneObjectWriter::setObjectClass(const int & _i, const float & _confidence) 
  throw(SubarchitectureProcessException)  {
  checkLoaded();
  m_pCurrentObject->m_recognisedClass.m_int = _i; 
  m_pCurrentObject->m_recognisedClass.m_confidence = _confidence;
  objectClassUpdated();
}


string
SceneObjectWriter::writeObject()  
  throw(SubarchitectureProcessException) {

  checkLoaded();

  //if current id is "" then we're adding
  if(isNewObject()) {
    m_component.log("Adding WM SceneObject entry ID: %s", 
		    m_currentObjectID.c_str());
    m_component.addToWorkingMemory(m_currentObjectID, 
				   m_pCurrentObject, 
				   cdl::BLOCKING); 
  }
  else {
    if(objectPropertiesUpdated() || m_potentiallyChanged) {
      m_component.log("Overwriting WM SceneObject entry ID: %s", 
		      m_currentObjectID.c_str());
      m_component.overwriteWorkingMemory(m_currentObjectID, 
					 m_pCurrentObject, 
					 cdl::BLOCKING);
    }
    else {
      m_component.log("Not overwriting unchanged WM SceneObject entry ID: %s", 
		      m_currentObjectID.c_str());
    }

    if(m_loadedPermissions != cast::cdl::UNLOCKED) {
      m_component.unlockEntry(m_currentObjectID);
      m_loadedPermissions = cast::cdl::UNLOCKED;
    }
  }

  m_pCurrentObject = NULL;

  if(objectPropertiesUpdated() || m_potentiallyChanged) {
    m_component.log("Writing update struct");
    SceneObjectUpdate * update = new SceneObjectUpdate();
    update->m_soID = CORBA::string_dup(m_currentObjectID.c_str());
    update->m_updates = m_updated;
    //also write update struct
    m_component.addToWorkingMemory(m_component.newDataID(),
				   update);
  }

  resetUpdates();

  return m_currentObjectID;

}
