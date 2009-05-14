#ifndef SCENE_OBJECT_WRITER_HPP
#define SCENE_OBJECT_WRITER_HPP

#include <vision/idl/Vision.hh>
#include <vision/utils/VideoClientProcess.h>

#include <cast/core/CASTDataCache.hpp>
#include <cast/architecture/ManagedProcess.hpp>


/**
 * A class for handling writing and overwriting SceneObjects
 */
class SceneObjectWriter {

public :
  SceneObjectWriter(cast::ManagedProcess& _component);


  /**
   * Create a new scene object with default values and set as current
   * object. Returns the id it will be written to.
   */
  std::string newSceneObject() throw( cast::SubarchitectureProcessException);

  /**
   * Load existing object from wm and set as current object.
   */
  void loadSceneObject(const std::string & _soID, 
		       const cast::cdl::WorkingMemoryPermissions & _permissions = cast::cdl::LOCKED_ODR) 
    throw( cast::SubarchitectureProcessException);

  
  Vision::SceneObject & currentObject()  throw( cast::SubarchitectureProcessException) {
    checkLoaded();
    //no longer safe to ignore overwrite
    m_potentiallyChanged = true;
    return *m_pCurrentObject;
  }


  void setObjectColour(const int & _i, const float & _confidence) throw( cast::SubarchitectureProcessException);
  void setObjectShape(const int & _i, const float & _confidence) throw( cast::SubarchitectureProcessException);
  void setObjectSize(const int & _i, const float & _confidence) throw( cast::SubarchitectureProcessException);
  void setObjectClass(const int & _i, const float & _confidence) throw( cast::SubarchitectureProcessException);


  void objectPositionUpdated() throw( cast::SubarchitectureProcessException) {
    checkLoaded();
    m_updated.m_position = true;
  }

  void objectShapeUpdated() throw( cast::SubarchitectureProcessException) {
    checkLoaded();
    m_updated.m_shape = true;
  }

  void objectSizeUpdated() throw( cast::SubarchitectureProcessException) {
    checkLoaded();
    m_updated.m_size = true;
  }

  void objectColourUpdated() throw( cast::SubarchitectureProcessException) {
    checkLoaded();
    m_updated.m_colour = true;
  }

  void objectClassUpdated() throw( cast::SubarchitectureProcessException) {
    checkLoaded();
    m_updated.m_class = true;
  }

  void addObjectROI(const std::string & _roiID) throw( cast::SubarchitectureProcessException);

  std::string writeObject() throw( cast::SubarchitectureProcessException);

  bool objectPropertiesUpdated() {
    return m_updated.m_position ||
      m_updated.m_shape ||
      m_updated.m_size ||
      m_updated.m_colour ||
      m_updated.m_class;    
  }

  bool isNewObject() const {
    return m_isNew;
  }

private:  

  /**
   * Check that there is no current proxy.
   */
  void checkUnloaded() throw( cast::SubarchitectureProcessException) {
    if(m_pCurrentObject != NULL) {
      throw( cast::SubarchitectureProcessException(__HERE__,"SceneObject in use. Cancel or store it before working on a new one."));
    }
  }

  /**
   * Check that there is a current proxy.
   */  
  void checkLoaded() throw( cast::SubarchitectureProcessException) {
    if(m_pCurrentObject == NULL) {
      throw( cast::SubarchitectureProcessException(__HERE__,"No current SceneObject. Use either newSceneObject or loadSceneObject before setting values"));
    }
  }
  
  void resetUpdates() {
    m_updated.m_position = false;
    m_updated.m_shape = false;
    m_updated.m_size = false;
    m_updated.m_colour = false;
    m_updated.m_class = false;
  }

  /**
   * Cache for the scene objects.
   */
  cast::CASTDataCache<Vision::SceneObject> m_soCache;

  cast::ManagedProcess& m_component;
  Vision::ObjectPropertyUpdates m_updated;
  Vision::SceneObject * m_pCurrentObject;
  std::string m_currentObjectID;
  bool m_potentiallyChanged;
  bool m_isNew;
  cast::cdl::WorkingMemoryPermissions m_loadedPermissions;
};

#endif
