/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 *
 * Copyright (C) 2006-2007 Nick Hawes, Michael Zillich
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include "FrameworkProcessCreatorManager.hpp"

#include <dlfcn.h>

using namespace std;

ProcessCreatorMap * FrameworkProcessCreatorManager::m_pCreators = new ProcessCreatorMap();

FrameworkProcessCreatorManager::FrameworkProcessCreatorManager() {

}




FrameworkProcessCreatorManager::~FrameworkProcessCreatorManager() {
  delete m_pCreators;
}


void FrameworkProcessCreatorManager::addClass(const string &_datatype, 
					      FrameworkProcessCreator * _pCreator) {
  
  string lowerData = _datatype;
  int (*pf)(int)=tolower; 
  transform (lowerData.begin(),lowerData.end(), lowerData.begin(), pf);

  (*m_pCreators)[lowerData] = _pCreator;
}


/**
 * Loads library libBASENAME.so.
 * Libraries are searched in BASEDIR/lib. It is assumed that current directory
 * is BASEDIR, which is the directory from where bin/zworkd is started.
 * Returns library handle.
 *
 * Taken from zwork.
 */
void* FrameworkProcessCreatorManager::loadLibrary(const string &baseName) {
  char libName[PATH_MAX];
  void *libHandle;

#ifdef __APPLE__
  snprintf(libName, PATH_MAX, "lib%s.dylib", baseName.c_str());
#else
  snprintf(libName, PATH_MAX, "lib%s.so", baseName.c_str());
#endif

  // note: multiple calls of dlopen() on the same library are ok and just
  // return the same handle
  // IMPORTANT: RTLD_LOCAL is crucial!
  // If RTLD_GLOBAL is used here, symbols of the loaded library (= component)
  // are made available globally. If two libraries contain the same symbols
  // (e.g.  Matrix::mult(), Image::Image()) one overwrites the other and one
  // library is left with a mess. The result is usually a very odd segfault
  // wich is very hard to find.
  

  //libHandle = dlopen(libName, RTLD_NOW | RTLD_LOCAL);

  //nah: Changed to GLOBAL because symbols are not shared when opened via JNI
  //http://gcc.gnu.org/bugzilla/show_bug.cgi?id=11223
  //http://gcc.gnu.org/bugzilla/show_bug.cgi?id=4993
  //libHandle = dlopen(libName, RTLD_NOW | RTLD_GLOBAL);

  //cout<<"about to dlopen: "<<libName<<endl;
  
  libHandle = dlopen(libName, RTLD_LAZY | RTLD_GLOBAL);

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
    throw BALTException(__HERE__, dlerror());
  }
  else {
    //   cout<<" ... OK "<<endl;
  }

  //

  return libHandle;
}


const FrameworkProcessCreator * FrameworkProcessCreatorManager::processCreator(const string &_className) {

  string lowerName = _className;
  int (*pf)(int)=tolower; 
  transform (lowerName.begin(),lowerName.end(), lowerName.begin(), pf);

  ProcessCreatorMap::iterator i = m_pCreators->find(lowerName);  
  if(i != m_pCreators->end()) {
    return i->second;
  } 
  else {

    //if a process creator has not been found, try to create one by
    //dynamically loading a library

    void *libHandle = loadLibrary(_className);

    //pointer to the function for creating the component
    FrameworkProcess* (*newComponent)(const string &);

    //void pointer to the method
    void *nc = dlsym(libHandle, "newComponent");


    //why not?
    //CnewComponent = static_cast<FrameworkProcess* (*)(const string &)>(nc);

    newComponent = (FrameworkProcess* (*)(const string &))(nc);



    if(newComponent == 0) {
      throw BALTException(__HERE__,
			  "no newComponent function defined in library lib%s.so", _className.c_str());
    }

    //create a new dynamic creator object using the dynamically loaded
    //library function pointer 
    FrameworkProcessCreator *fpc = new DynamicProcessCreator(libHandle,
							     newComponent);

    //add it to the manager for later operations
    addClass(_className,fpc);

    //and return the creator
    return fpc;

  }
  
}


