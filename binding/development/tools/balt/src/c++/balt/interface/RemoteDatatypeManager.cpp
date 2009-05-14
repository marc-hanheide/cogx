/*
o * BALT - The Boxes and Lines Toolkit for component communication.
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
 
#include "RemoteDatatypeManager.hpp"
#include "GenericRemoteConnectionCreator.hpp"


//#include <iostream>
//#include <stringstream>

using namespace std;


RemoteConnectionCreatorMap * RemoteDatatypeManager::m_pCreators = new RemoteConnectionCreatorMap();
bool RemoteDatatypeManager::m_bInit = false;

RemoteDatatypeManager::RemoteDatatypeManager() {
}


void RemoteDatatypeManager::builtInTypes() {

  //addDatatype("string",
  //      new GenericRemoteConnectionCreator<string>(new GenericRemoteDataTranslator<string>()));

//   addDatatype("int",
// 	      new GenericRemoteConnectionCreator<CORBA::Long>(new GenericRemoteDataTranslator<CORBA::Long>()));

//   addDatatype("int",
// 	      new GenericRemoteConnectionCreator<int>(new GenericRemoteDataTranslator<int>()));

//   addDatatype("long",
// 	      new GenericRemoteConnectionCreator<CORBA::LongLong>(new GenericRemoteDataTranslator<CORBA::LongLong>()));
//   addDatatype("short",
// 	      new GenericRemoteConnectionCreator<CORBA::Short>(new GenericRemoteDataTranslator<CORBA::Short>()));
//   addDatatype("double",
// 	      new GenericRemoteConnectionCreator<CORBA::Double>(new GenericRemoteDataTranslator<CORBA::Double>()));
//   addDatatype("float",
// 	      new GenericRemoteConnectionCreator<CORBA::Float>(new GenericRemoteDataTranslator<CORBA::Float>()));
//   addDatatype("char",
// 	      new GenericRemoteConnectionCreator<CORBA::Char>(new GenericRemoteDataTranslator<CORBA::Char>()));
//   addDatatype("byte",
// 	      new GenericRemoteConnectionCreator<CORBA::Octet>(new GenericRemoteDataTranslator<CORBA::Octet>()));
//   addDatatype("bool",
// 	      new GenericRemoteConnectionCreator<CORBA::Boolean>(new GenericRemoteDataTranslator<CORBA::Boolean>()));

//   addDatatype("stringseq",
// 	      new GenericRemoteConnectionCreator<CORBA::StringSeq>(new GenericRemoteSequenceTranslator<CORBA::StringSeq>()));
//   addDatatype("intseq",
// 	      new GenericRemoteConnectionCreator<CORBA::LongSeq>(new GenericRemoteSequenceTranslator<CORBA::LongSeq>()));
//   addDatatype("longseq",
// 	      new GenericRemoteConnectionCreator<CORBA::LongLongSeq>(new GenericRemoteSequenceTranslator<CORBA::LongLongSeq>()));
//   addDatatype("shortseq",
// 	      new GenericRemoteConnectionCreator<CORBA::ShortSeq>(new GenericRemoteSequenceTranslator<CORBA::ShortSeq>()));
//   addDatatype("doubleseq",
// 	      new GenericRemoteConnectionCreator<CORBA::DoubleSeq>(new GenericRemoteSequenceTranslator<CORBA::DoubleSeq>()));
//   addDatatype("floatseq",
// 	      new GenericRemoteConnectionCreator<CORBA::FloatSeq>(new GenericRemoteSequenceTranslator<CORBA::FloatSeq>()));
//   addDatatype("charseq",
// 	      new GenericRemoteConnectionCreator<CORBA::CharSeq>(new GenericRemoteSequenceTranslator<CORBA::CharSeq>()));
//   addDatatype("byteseq",
// 	      new GenericRemoteConnectionCreator<CORBA::OctetSeq>(new GenericRemoteSequenceTranslator<CORBA::OctetSeq>()));
//   addDatatype("boolseq",
// 	      new GenericRemoteConnectionCreator<CORBA::BooleanSeq>(new GenericRemoteSequenceTranslator<CORBA::BooleanSeq>()));


}

void RemoteDatatypeManager::init() {

  builtInTypes();
  m_bInit = true;
}




RemoteDatatypeManager::~RemoteDatatypeManager() {
    delete m_pCreators;
}


void RemoteDatatypeManager::addDatatype(const string &_datatype, RemoteConnectionCreator * _pCreator) {

    string lowerData = _datatype;
    int (*pf)(int)=tolower;
    transform (lowerData.begin(),lowerData.end(), lowerData.begin(), pf);

    (*m_pCreators)[lowerData] = _pCreator;
}



// void RemoteDatatypeManager::addSequenceDatatype(const string &_datatype, RemoteConnectionCreator * _pCreator) {

//     string lowerData = _datatype;
//     int (*pf)(int)=tolower;
//     transform (lowerData.begin(),lowerData.end(), lowerData.begin(), pf);

//     (*m_pCreators)[lowerData] = _pCreator;
// }


const RemoteConnectionCreator * RemoteDatatypeManager::connectionCreator(const string &_datatype) {

    if(!m_bInit) {
        init();
    }

    string lowerData = _datatype;
    int (*pf)(int)=tolower;
    transform (lowerData.begin(),lowerData.end(), lowerData.begin(), pf);


    RemoteConnectionCreatorMap::iterator i = m_pCreators->find(lowerData);

    if(i != m_pCreators->end()) {
        return i->second;
    } else {
        cerr<<"RemoteDatatypeManager ERROR: Unknown datatype: "<<_datatype<<endl;
        return NULL;
    }

}


// const RemoteDataTranslator * RemoteDatatypeManager::translator(const string &_datatype) {

//   if(!m_bInit) {
//     init();
//   }

//   string lowerData = _datatype;
//   int (*pf)(int)=tolower;
//   transform (lowerData.begin(),lowerData.end(), lowerData.begin(), pf);
  
  
//   RemoteConnectionCreatorMap::iterator i = m_pCreators->find(lowerData);
  
//   if(i != m_pCreators->end()) {
//     return i->second;
//   } else {
//     cerr<<"ERROR: Unknown datatype: "<<_datatype<<endl;
//     return NULL;
//   }
  
// }
