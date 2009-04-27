#include "ComponentCreator.hpp"

#include <iostream>

#ifndef __APPLE__
#include <linux/limits.h>
#endif

using namespace std;
using namespace Ice;
using namespace IceUtil;


namespace cast {

using namespace cdl;



/**
 * Load a shared library and return handle.
 */
void* loadLibrary(const string & _baseName)  {
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

 
  DynamicComponentCreator * 
  createComponentCreator(const std::string &_baseName) 
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
 



}; //namespace cast
