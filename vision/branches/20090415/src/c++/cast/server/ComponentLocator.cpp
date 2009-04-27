#include "ComponentLocator.hpp"

#include <iostream>

#ifndef __APPLE__
#include <linux/limits.h>
#endif

using namespace std;
using namespace Ice;
using namespace IceUtil;


namespace cast {

using namespace cdl;

ComponentLocator::ComponentLocator() : Ice::ServantLocator() {}

ComponentLocator::~ComponentLocator() {
  //destroy creators
  for(ComponentCreatorMap::iterator i = m_creators.begin();
      i != m_creators.end(); ++i) {
    delete i->second;
    i->second = NULL;    
  }
}


  DynamicComponentCreator * 
  ComponentLocator::createComponentCreator(const std::string &_baseName) 
  throw (CASTException)  {
    
    //load the library
    void *libHandle = loadLibrary(_baseName);

    //pointer to the function for creating the component
    cast::CASTComponentPtr (*newComponent)();

    //void pointer to the method
    void *nc = dlsym(libHandle, "newComponent");

    //newComponent = (cast::CASTComponentPtr (*)(const string &))(nc);
    newComponent = (cast::CASTComponentPtr (*)())(nc);

    if(newComponent == 0) {
      throw CASTException(exceptionMessage(__HERE__,
					   "no newComponent function defined in library lib%s.so", 
					   _baseName.c_str()));
    }

    //create a new dynamic creator object using the dynamically loaded
    //library function pointer 
    return new DynamicComponentCreator(libHandle,
				       newComponent);
  }

  DynamicComponentCreator * 
  ComponentLocator::getComponentCreator(const std::string &_baseName) 
  throw (CASTException) { 
    ComponentCreatorMap::iterator i = m_creators.find(_baseName);
    //if we don't have it yet
    if(i == m_creators.end()) {
      DynamicComponentCreator * dcc = createComponentCreator(_baseName);
      m_creators[_baseName] = dcc;
      return dcc;
    }
    else {
      return i->second;
    }
    
  }



Ice::ObjectPtr ComponentLocator::locate(const Ice::Current&  _current, 
					Ice::LocalObjectPtr& cookie) throw (ObjectNotExistException) {

  Mutex::Lock lock(m_mutex); 
	
  ObjectPtr servant = _current.adapter->find(_current.id); 
	
  if (!servant) {     // We don't have a servant already 
		
//     cout<<"name: "<< _current.id.name<<endl;
//     cout<<"category: "<<_current.id.category<<endl;	

    //TODO handle exceptions

    cast::CASTComponentPtr component;
    try {
      DynamicComponentCreator * dcc = getComponentCreator(_current.id.category);
      component = dcc->createNewComponent(_current.id.name);
      component->setObjectAdapter(_current.adapter);
      component->setIceIdentity(_current.id);

      //now generalise again so it doesn't assume it's only a castcomponent
      servant = component;

    }
    catch(CASTException & e) {
      //this will leave servant as null
      cout<<e.message<<endl;
      ObjectNotExistException e(__FILE__,__LINE__);
      e.id = _current.id;
    }
	
    

    if(servant) {
      _current.adapter->add(servant, _current.id); 
    }
    else {
      cerr<<"No component for:"<<endl;
      cerr<<"name: "<< _current.id.name<<endl;
      cerr<<"category: "<<_current.id.category<<endl;	
		  
    }
  } 
	
  return servant; 
	
}  


/**
 * Load a shared library and return handle.
 */
void* ComponentLocator::loadLibrary(const string & _baseName) const {
  char libName[PATH_MAX];
  void *libHandle;

#ifdef __APPLE__
  snprintf(libName, PATH_MAX, "lib%s.dylib", _baseName.c_str());
#else
  snprintf(libName, PATH_MAX, "lib%s.so", _baseName.c_str());
#endif

  // note: multiple calls of dlopen() on the same library are ok and just
  // return the same handle
  // IMPORTANT: RTLD_LOCAL is crucial!
  // If RTLD_GLOBAL is used here, symbols of the loaded library (= component)
  // are made available globally. If two libraries contain the same symbols
  // (e.g.  Matrix::mult(), Image::Image()) one overwrites the other and one
  // library is left with a mess. The result is usually a very odd segfault
  // wich is very hard to find.
  

  libHandle = dlopen(libName, RTLD_NOW | RTLD_LOCAL);

  //nah: Changed to GLOBAL because symbols are not shared when opened via JNI
  //http://gcc.gnu.org/bugzilla/show_bug.cgi?id=11223
  //http://gcc.gnu.org/bugzilla/show_bug.cgi?id=4993
  //libHandle = dlopen(libName, RTLD_NOW | RTLD_GLOBAL);

  //cout<<"about to dlopen: "<<libName<<endl;
  
  //libHandle = dlopen(libName, RTLD_LAZY | RTLD_GLOBAL);

  //nah: bug fix #34
  //nah: changed back.... anything else seems to result in a hang

  //this causes the cast crash
  //libHandle = dlopen(libName, RTLD_NOW | RTLD_LOCAL);

  //this causes the freeze
  //libHandle = dlopen(libName, RTLD_NOW | RTLD_GLOBAL);
  
  //this causes the cast crash
  //libHandle = dlopen(libName, RTLD_LAZY | RTLD_LOCAL);

  //this causes the freeze
  //libHandle = dlopen(libName, RTLD_LAZY | RTLD_GLOBAL);

  //this causes the cast crash
  //libHandle = dlopen(libName, RTLD_LAZY);
  //libHandle = dlopen(libName, RTLD_NOW);

  if(!libHandle) {
    throw CASTException(exceptionMessage(__HERE__, dlerror()));
  }

  return libHandle;

}


void ComponentLocator::finished(const Ice::Current& c, 
				const Ice::ObjectPtr&      servant, 
				const Ice::LocalObjectPtr& cookie) {				
}


void ComponentLocator::deactivate(const std::string& category) {

}


}; //namespace cast
