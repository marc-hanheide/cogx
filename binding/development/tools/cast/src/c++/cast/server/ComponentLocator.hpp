#include <Ice/Ice.h> 
#include <cast/slice/CDL.hpp> 
#include <cast/core/CASTUtils.hpp> 
#include <cast/core/CASTComponent.hpp> 
#include <cast/core/StringMap.hpp> 
#include <dlfcn.h>

namespace cast {

class DynamicComponentCreator {

  ///Handle on the  the dynamically loaded library the component comes from
  void * m_libHandle;

  ///pointer to the function for creating the component
  cast::CASTComponentPtr (*m_newComponent)();
    

 public:

  DynamicComponentCreator(void * _libHandle, 
			  cast::CASTComponentPtr (*_newComponent)()) {
    m_libHandle = _libHandle;
    m_newComponent = _newComponent;
  }

  ///Closes the library that was opened to create this 
  virtual ~DynamicComponentCreator() {
    dlclose(m_libHandle);
  }

  virtual 
  cast::CASTComponentPtr  
  createNewComponent(const std::string &_procName) const {
    
    cast::CASTComponentPtr  proc = m_newComponent();
    proc->setID(_procName, ::Ice::Current());
    if(!proc) {
      throw cast::CASTException(exceptionMessage(__HERE__, 
						 "failed to load component %s", 
						 _procName.c_str()));
    }

    return proc;
  };


};


class ComponentLocator : public virtual Ice::ServantLocator { 
  
  typedef StringMap<DynamicComponentCreator *>::map ComponentCreatorMap;
  
public: 
  
  ComponentLocator();
  virtual ~ComponentLocator();
  
  virtual Ice::ObjectPtr locate(const Ice::Current&  c, 
				Ice::LocalObjectPtr& cookie) 
    throw (Ice::ObjectNotExistException); 
  
  virtual void finished(const Ice::Current&        c, 
			const Ice::ObjectPtr&      servant, 
			const Ice::LocalObjectPtr& cookie);
  
  virtual void deactivate(const std::string& category);

private:
  

  /**
   * Load a shared library and return handle.
   */
  void* loadLibrary(const std::string &_baseName) const;

  DynamicComponentCreator * 
  createComponentCreator(const std::string &_baseName)
  throw (CASTException);
  
  DynamicComponentCreator * 
  getComponentCreator(const std::string &_baseName) 
  throw (CASTException);
  
  IceUtil::Mutex m_mutex; 

  
  ComponentCreatorMap m_creators;

}; 

}; //namespace cast
