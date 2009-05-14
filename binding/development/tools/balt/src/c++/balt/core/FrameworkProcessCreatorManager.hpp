/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 *
 * Copyright (C) 2006-2007 Nick Hawes
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

#ifndef FRAMEWORK_PROCESS_CREATOR_MANAGER_H_
#define FRAMEWORK_PROCESS_CREATOR_MANAGER_H_


#include "includes.hpp"
#include "FrameworkProcess.hpp"
#include "FrameworkProcessCreator.hpp"
#include "BALTException.hpp"

typedef cast::StringMap<FrameworkProcessCreator *>::map ProcessCreatorMap;

class FrameworkProcessCreatorManager {

 public:
  FrameworkProcessCreatorManager();
  ~FrameworkProcessCreatorManager();
  
  template <class T>
  static void addClass(const std::string &_datatype)  {
      std::string lowerData = _datatype;
    int (*pf)(int)=tolower; 
    transform (lowerData.begin(),lowerData.end(), lowerData.begin(), pf);
    
    (*m_pCreators)[lowerData] = new GenericProcessCreator<T>();
  }

  
  static void addClass(const std::string &_datatype, FrameworkProcessCreator * _pCreator);

  static const FrameworkProcessCreator * processCreator(const std::string &_className);



 private:
  static void* loadLibrary(const std::string &baseName);

  static ProcessCreatorMap * m_pCreators;

};


#endif
