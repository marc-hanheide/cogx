
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

#include "LocalDatatypeManager.hpp"

#include "BALTException.hpp"

using namespace std;

ConnectionCreatorMap * LocalDatatypeManager::m_pCreators = new ConnectionCreatorMap();
bool LocalDatatypeManager::m_bInit = false;

LocalDatatypeManager::LocalDatatypeManager() {

}


void LocalDatatypeManager::builtInTypes() {
  
  //addDatatype("string",new GenericConnectionCreator<std::string>());
  //addDatatype("int",new GenericConnectionCreator<int>());
//   addDatatype("long",new GenericConnectionCreator<long>());
//   addDatatype("short",new GenericConnectionCreator<short>());
//   addDatatype("double",new GenericConnectionCreator<double>());
//   addDatatype("float",new GenericConnectionCreator<float>());
//   addDatatype("char",new GenericConnectionCreator<char>());
//   addDatatype("byte",new GenericConnectionCreator<unsigned char>());
//   addDatatype("bool",new GenericConnectionCreator<bool>());
  
//   addDatatype("stringseq",new GenericConnectionCreator<CORBA::StringSeq>());  
//   addDatatype("intseq",new GenericConnectionCreator<CORBA::LongSeq>());  
//   addDatatype("longseq",new GenericConnectionCreator<CORBA::LongLongSeq>());  
//   addDatatype("shortseq",new GenericConnectionCreator<CORBA::ShortSeq>());  
//   addDatatype("doubleseq",new GenericConnectionCreator<CORBA::DoubleSeq>()); 
//   addDatatype("floatseq",new GenericConnectionCreator<CORBA::FloatSeq>());  
//   addDatatype("charseq",new GenericConnectionCreator<CORBA::CharSeq>());  
//   addDatatype("byteseq",new GenericConnectionCreator<CORBA::OctetSeq>());  
//   addDatatype("boolseq",new GenericConnectionCreator<CORBA::BooleanSeq>());  

}

void LocalDatatypeManager::init() {
  
  
  builtInTypes();
  
  m_bInit = true;
}


LocalDatatypeManager::~LocalDatatypeManager() {
  delete m_pCreators;
}




void LocalDatatypeManager::addDatatype(const string &_datatype, LocalConnectionCreator * _pCreator) {
  
  //cout<<"LocalDatatypeManager adding: "<<_datatype<<endl;

//   string lowerData = _datatype;
//   int (*pf)(int)=tolower; 
//   transform (lowerData.begin(),lowerData.end(), lowerData.begin(), pf);

  ConnectionCreatorMap::iterator i = m_pCreators->find(_datatype);

  if(i == m_pCreators->end()) {
    (*m_pCreators)[_datatype] = _pCreator;
  }
  else {
    //cout<<"LocalDatatypeManager already added: "<<_datatype<<endl;
    delete _pCreator;
  }
}


const LocalConnectionCreator * LocalDatatypeManager::connectionCreator(const string &_datatype) {

  if(!m_bInit) {
    init();
  }

//   string lowerData = _datatype;
//   int (*pf)(int)=tolower; 
//   transform (lowerData.begin(),lowerData.end(), lowerData.begin(), pf);

  ConnectionCreatorMap::iterator i = m_pCreators->find(_datatype);
  
  if(i != m_pCreators->end()) {
    return i->second;
  } else {
    throw(BALTException(__HERE__,"LocalDatatypeManager ERROR: Unknown datatype: %s",_datatype.c_str()));    
  }
  
}
