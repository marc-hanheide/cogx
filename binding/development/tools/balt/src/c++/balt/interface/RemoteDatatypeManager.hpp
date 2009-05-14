/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 * 
 * Copyright (C) 2006-2007 Nick Hawes, Gregor Berginc
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

#ifndef REMOTE_DATATYPE_MANAGER
#define REMOTE_DATATYPE_MANAGER

/**
The job of this class is to provide static methods for mapping from a
name of a datatype to particular objects.
*/

#include <balt/core/BALTCore.hpp>
#include "RemoteConnectionCreator.hpp"

#include "GenericRemoteConnectionCreator.hpp"



#include "RemoteTranslatorManager.hpp"

#include "includes.hpp"


template <class T> class GenericRemoteConnectionCreator;
template <class T> class GenericRemoteDataTranslator;
template <class T> class GenericRemoteSequenceTranslator;

typedef cast::StringMap<RemoteConnectionCreator*>::map RemoteConnectionCreatorMap;

class RemoteDatatypeManager {

public:
  RemoteDatatypeManager();
  ~RemoteDatatypeManager();
  
  static void addDatatype(const std::string &_datatype, RemoteConnectionCreator * _pCreator);
  
  template <class T>
  static void addObjectDatatype() {    
    std::string datatype = BALTType::typeName<T>();
    RemoteDataTranslator<T>* pTrans = new GenericConsumingTranslator<T>();
    addDatatype(datatype,new GenericRemoteConnectionCreator<T>(pTrans));
    RemoteTranslatorManager::addDatatype(datatype,pTrans);
    //std::cout<<"added: \""<<datatype<<"\""<<std::endl;
  }

  template <class T>
  static void addEnumDatatype() {
    std::string datatype = BALTType::typeName<T>();
    RemoteDataTranslator<T>* pTrans = new GenericRemoteDataTranslator<T>();
    addDatatype(datatype,new GenericRemoteConnectionCreator<T>(pTrans));
    RemoteTranslatorManager::addDatatype(datatype,pTrans);
  }
  
  template <class T>
  static void addSequenceDatatype() {
    std::string datatype = BALTType::typeName<T>();
    RemoteDataTranslator<T>* pTrans = new GenericRemoteSequenceTranslator<T>();
    addDatatype(datatype,
		new GenericRemoteConnectionCreator<T>(pTrans));
    RemoteTranslatorManager::addDatatype(datatype,pTrans);    
  }
  
  static const RemoteConnectionCreator * connectionCreator(const std::string &_datatype);
  

private:
  static void init();
  static void builtInTypes();
  static RemoteConnectionCreatorMap * m_pCreators;

  static bool m_bInit;
  

};


#endif
