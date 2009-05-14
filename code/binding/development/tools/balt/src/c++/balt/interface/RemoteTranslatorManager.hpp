/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 * 
 * Copyright (C) 2006-2007 Nick Hawes, Gregor Berginc, Henrik
 * Jacobsson
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

#ifndef REMOTE_TRANSLATOR_MANAGER
#define REMOTE_TRANSLATOR_MANAGER

/**
   The job of this class is to provide static methods for mapping from a
   name of a datatype to particular objects.
 
*/

#include <balt/core/BALTCore.hpp>

#include "RemoteDataTranslator.hpp"


#include "includes.hpp"


typedef cast::StringMap<AbstractRemoteDataTranslator * >::map RemoteTranslatorMap;

class RemoteTranslatorManager {

public:
  RemoteTranslatorManager();
  ~RemoteTranslatorManager();

 
  static void addDatatype(const std::string &_datatype, 
			  AbstractRemoteDataTranslator * _pTranslator);
    
    
  template <class T>
  static void addObjectDatatype() {
    std::string _datatype = BALTType::typeName<T>();
    addDatatype(_datatype,
		new GenericConsumingTranslator<T>());
      
  }
    
  template <class T>
  static void addSequenceDatatype() {
    std::string _datatype = BALTType::typeName<T>();
    addDatatype(_datatype,
		new GenericRemoteSequenceTranslator<T>());

  }


  static const AbstractRemoteDataTranslator * abstractTranslator(const std::string &_datatype);
    

  template <class T>
  static const RemoteDataTranslator<T> * 
  translator() {
    
    
    if(!m_bInit) {
      init();
    }
    
    std::string _datatype = BALTType::typeName<T>();
    
    RemoteTranslatorMap::iterator i = m_pTranslators->find(_datatype);
    
    if(i != m_pTranslators->end()) {
      AbstractRemoteDataTranslator * abstr = i->second;
      
      //RemoteDataTranslator<T> * trans = dynamic_cast<RemoteDataTranslator<T>*>(abstr);
      RemoteDataTranslator<T> * trans = static_cast<RemoteDataTranslator<T>*>(abstr);
      
      if(trans) {
	return trans;
      }
      else {
	
	std::ostringstream outStream;
	outStream<<"ERROR: Template type doesn't match stored datatype: "
		 <<_datatype<<std::endl;
	
	throw BALTException(__HERE__,
			    outStream.str().c_str());
	
      }
    }
    else {
      //std::cout<<"RemoteTranslatorManager::adding and recursing"<<std::endl;
      //try adding it as an object
      addObjectDatatype<T>();
      //and recurse
      return translator<T>();
    }
    
//     throw BALTException(__HERE__,
// 			"ERROR: RemoteTranslatorMap unknown datatype: %s",_datatype.c_str());
    
    
  }
  
  

private:
  static void init();
  static void builtInTypes();
  static RemoteTranslatorMap * m_pTranslators;
  static bool m_bInit;


};


#endif
