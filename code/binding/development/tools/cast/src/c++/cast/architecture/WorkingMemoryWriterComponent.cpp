/*
  * CAST - The CoSy Architecture Schema Toolkit
  *
  * Copyright (C) 2006-2009 Nick Hawes
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

#include "WorkingMemoryWriterComponent.hpp"
#include <sstream>

using namespace std;

namespace cast {

  WorkingMemoryWriterComponent::WorkingMemoryWriterComponent() 
    : m_dataCount(0) {
  }


  WorkingMemoryWriterComponent::~WorkingMemoryWriterComponent() {

  }

  const unsigned char ID_TABLE_SIZE = 62;
  const char id_table[ID_TABLE_SIZE + 1] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  // a bit safer perhaps:
  //  const unsigned char ID_TABLE_SIZE = 37;
  //  const char id_table[ID_TABLE_SIZE + 1] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

  // generates short unique strings from ints
  inline
  string stringify(int _n) {
    
    if(_n == 0) 
      return string(1,id_table[0]);
    
    string ret;
    
    int i = 0;
    while(_n != 0) {
      ret.resize(i+1);
      unsigned char m = static_cast<unsigned char>(_n % ID_TABLE_SIZE);
      ret[i++] = id_table[m];
      _n -= m;
      assert(_n % ID_TABLE_SIZE == 0);
      _n /= ID_TABLE_SIZE;
    }
    return ret;
  }


  void 
  WorkingMemoryWriterComponent::configureInternal(const map<string,string> & _config) {

    WorkingMemoryAttachedComponent::configureInternal(_config);

    map<string,string>::const_iterator i = _config.find(cdl::COMPONENTNUMBERKEY);

    assert(i != _config.end());

    istringstream configStream(i->second);
    configStream >> m_componentNumber;
    m_componentNumberString = stringify(m_componentNumber);
    
  }

  inline
  string WorkingMemoryWriterComponent::newDataID() {
  
    std::ostringstream o;
        
    //o << m_dataCount << ":";
    o << stringify(m_dataCount) << ":";
    if(m_bDebugOutput) {
      o << getComponentID();
      o << ":data";
    }
    else {
      o << m_componentNumberString;
    }
    
    m_dataCount++;

    return o.str();
    
  }





  void 
  WorkingMemoryWriterComponent::deleteFromWorkingMemory(const string &_id,
							const string &_subarch) 
    throw (DoesNotExistOnWMException,PermissionException, UnknownSubarchitectureException) {
    
    assert(!_id.empty());//id must not be empty
    assert(!_subarch.empty());//subarch must not be empty
    
    //UPGRADE still needs thought
    //checkPrivileges(_subarch);

    // do the check here as it may save a lot of hassle later        
    if (!existsOnWorkingMemory(_id, _subarch)) {
      throw DoesNotExistOnWMException(exceptionMessage(__HERE__,
						       "Entry does not exist for deleting. Was looking for id %s in sa %s",
						       _id.c_str(), _subarch.c_str()),
				      makeWorkingMemoryAddress(_subarch, _id));
    }

    // if we don't currently hold a lock on this item
    if (!holdsDeleteLock(_id, _subarch)) {
      // check that we can delete it
      if (!isDeletable(_id, _subarch)) {
	  throw PermissionException(exceptionMessage(__HERE__,
						     "Delete not allowed on locked item: %s:%s",
						     _id.c_str(), _subarch.c_str()),
				    makeWorkingMemoryAddress(_subarch,_id));
      }
    }


    //logMemoryDelete(_id, _subarch);

    //always keep versioning... 
    //stopVersioning(_id);

    m_workingMemory->deleteFromWorkingMemory(_id,_subarch,getComponentID());
  }


} //namespace cast
