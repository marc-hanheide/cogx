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

#include "RemoteTranslatorManager.hpp"

using namespace std;

RemoteTranslatorMap * RemoteTranslatorManager::m_pTranslators = new RemoteTranslatorMap();
bool RemoteTranslatorManager::m_bInit = false;

RemoteTranslatorManager::RemoteTranslatorManager() {
}

void RemoteTranslatorManager::init() {

  addDatatype("string",
	      new GenericRemoteDataTranslator<string>());
//   addDatatype("int",
// 	      new GenericRemoteDataTranslator<int>());
  addDatatype("long",
	      new GenericRemoteDataTranslator<CORBA::LongLong>());
  addDatatype("short",
	      new GenericRemoteDataTranslator<CORBA::Short>());
  addDatatype("double",
	      new GenericRemoteDataTranslator<CORBA::Double>());
  addDatatype("float",
	      new GenericRemoteDataTranslator<CORBA::Float>());
  addDatatype("char",
	      new GenericRemoteDataTranslator<CORBA::Char>());
  addDatatype("byte",
	      new GenericRemoteDataTranslator<CORBA::Octet>());
  addDatatype("bool",
	      new GenericRemoteDataTranslator<CORBA::Boolean>());
  addDatatype("stringseq",
	      new GenericRemoteSequenceTranslator<CORBA::StringSeq>());
  addDatatype("intseq",
	      new GenericRemoteSequenceTranslator<CORBA::LongSeq>());
  addDatatype("longseq",
	      new GenericRemoteSequenceTranslator<CORBA::LongLongSeq>());
  addDatatype("shortseq",
	      new GenericRemoteSequenceTranslator<CORBA::ShortSeq>());
  addDatatype("doubleseq",
	      new GenericRemoteSequenceTranslator<CORBA::DoubleSeq>());
  addDatatype("floatseq",
	      new GenericRemoteSequenceTranslator<CORBA::FloatSeq>());
  addDatatype("charseq",
	      new GenericRemoteSequenceTranslator<CORBA::CharSeq>());
  addDatatype("byteseq",
	      new GenericRemoteSequenceTranslator<CORBA::OctetSeq>());
  addDatatype("boolseq",
	      new GenericRemoteSequenceTranslator<CORBA::BooleanSeq>());

  m_bInit = true;
}


RemoteTranslatorManager::~RemoteTranslatorManager() {
    delete m_pTranslators;
}


void RemoteTranslatorManager::addDatatype(const string &_datatype, 
					  AbstractRemoteDataTranslator * _pTranslator) {


  RemoteTranslatorMap::iterator i = m_pTranslators->find(_datatype);

  //if it doesn't exist
  if(i == m_pTranslators->end()) {
    (*m_pTranslators)[_datatype] = _pTranslator;
    //cout<<"Datatype added: "<<_datatype<<endl;
  } 
  else {
    //cout<<"Datatype already added: "<<_datatype<<endl;
    delete _pTranslator;
  }
  
}

const AbstractRemoteDataTranslator * RemoteTranslatorManager::abstractTranslator(const string &_datatype) {

    if(!m_bInit) {
        init();
    }


    RemoteTranslatorMap::iterator i = m_pTranslators->find(_datatype);

    if(i != m_pTranslators->end()) {
      return  i->second;
    } else {
        cerr<<"RemoteTranslatorManager ERROR: Unknown datatype: "<<_datatype<<endl;
        return NULL;
    }

}
