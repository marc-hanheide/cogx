#ifndef COMPONENT_CREATOR_HPP_
#define COMPONENT_CREATOR_HPP_

#include <Ice/Ice.h> 
#include <CDL.hpp> 
#include <CASTUtils.hpp> 
#include <CASTComponent.hpp> 
#include <StringMap.hpp> 
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

  DynamicComponentCreator * createComponentCreator(const std::string &_baseName) throw (CASTException);

  void* loadLibrary(const std::string & _baseName);

}; //namespace cast


#endif
