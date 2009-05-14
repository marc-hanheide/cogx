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

#ifndef FRAMEWORK_PROCESS_CREATOR_H_
#define FRAMEWORK_PROCESS_CREATOR_H_

#include "FrameworkProcess.hpp"
#include "BALTException.hpp"

#include <dlfcn.h>
#include <stdexcept>


class FrameworkProcessCreator {
  
 public:
  FrameworkProcessCreator(){};
  virtual ~FrameworkProcessCreator(){};
  virtual FrameworkProcess * createNewProcess(const std::string &_procName) const = 0;

};


template <class T>
class GenericProcessCreator : public FrameworkProcessCreator {
 public:
  virtual T * createNewProcess(const std::string &_procName) const {
    return new T(_procName);
  };
};

class DynamicProcessCreator : public FrameworkProcessCreator {

  ///Handle on the  the dynamically loaded library the process comes from
  void * m_libHandle;

  ///pointer to the function for creating the component
  FrameworkProcess* (*m_newComponent)(const std::string &);
    

 public:

  DynamicProcessCreator(void * _libHandle, 
			FrameworkProcess* (*_newComponent)(const std::string &)) {
    m_libHandle = _libHandle;
    m_newComponent = _newComponent;
  }

  ///Closes the library that was opened to create this 
  ~DynamicProcessCreator() {
    dlclose(m_libHandle);
  }

  virtual FrameworkProcess * createNewProcess(const std::string &_procName) const {

    FrameworkProcess * proc = m_newComponent(_procName);

    if(!proc) {
      throw BALTException(__HERE__, "failed to load component %s", _procName.c_str());
    }

    return proc;
  };


};




#endif
