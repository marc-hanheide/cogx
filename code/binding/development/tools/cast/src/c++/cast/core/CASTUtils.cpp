/*
 * CAST - The CoSy Architecture Schema Toolkit
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

#include "CASTUtils.hpp"
#include <cast/core/CASTException.hpp>
#include <cast/core/CASTDatatypeManager.hpp>
#include <cast/core/CASTWorkingMemoryItem.hpp>
#include <cast/cdl/CAST.hh>
#include <cast/cdl/guitypes.hh>
#include <balt/core/BALTLocalUtils.hpp>
#include <sstream>
using namespace std;

namespace cast {

  using namespace cdl;

  bool operator== (const cast::cdl::WorkingMemoryAddress & _wmaA, 
		   const cast::cdl::WorkingMemoryAddress & _wmaB) {

    //cout<<"overidden equals"<<endl;
    return (0 == strcmp(_wmaA.m_subarchitecture, _wmaB.m_subarchitecture)) &&
      (0 == strcmp(_wmaA.m_id,_wmaB.m_id));

  }

  bool operator== (const cast::cdl::WorkingMemoryChange & _wmcA, 
		   const cast::cdl::WorkingMemoryChange & _wmcB) {

    //cout<<"overidden equals"<<endl;
    return _wmcA.m_address == _wmcB.m_address &&
      _wmcA.m_operation == _wmcB.m_operation &&
      (0 == strcmp(_wmcA.m_src, _wmcB.m_src)) &&
      (0 == strcmp(_wmcA.m_type,_wmcB.m_type));
  
  }

  bool operator== (const cast::cdl::testing::CASTTestStruct & _ctsA, 
		   const cast::cdl::testing::CASTTestStruct & _ctsB) {

    //cout<<"overidden equals"<<endl;
    return _ctsA.m_count == _ctsB.m_count &&
      _ctsA.m_change == _ctsB.m_change;
    
  }



  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::WorkingMemoryAddress &_wma) {

    _stream<<"[WMA "
	   <<_wma.m_subarchitecture
	   <<":"
	   <<_wma.m_id
	   <<"]";
    return _stream;
  }

  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::WorkingMemoryOperation &_op) {

    switch(_op) {
    case cast::cdl::ADD:
      _stream<<"ADD";
      break;
    case cast::cdl::OVERWRITE:
      _stream<<"OVERWRITE";
      break;
    case cast::cdl::DELETE:
      _stream<<"DELETE";
      break;  
    case cast::cdl::GET:
      _stream<<"GET";
      break;     
    case cast::cdl::WILDCARD:
      _stream<<"WILDCARD";
      break;  
    }
  
    return _stream;
  }

  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::FilterRestriction &_restriction) {

    switch(_restriction) {
    case cast::cdl::LOCAL_SA:
      _stream<<"LOCAL_SA";
      break;
    case cast::cdl::ALL_SA:
      _stream<<"ALL_SA";
      break;
    }
  
    return _stream;

  }

  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::WorkingMemoryChangeFilter &_wmcf) {


    _stream<<"[WMCF: "
	   <<_wmcf.m_operation
	   <<" "
	   <<_wmcf.m_src
	   <<" "
	   <<_wmcf.m_address
	   <<" "
	   <<_wmcf.m_type
	   <<" "
	   <<_wmcf.m_restriction
	   <<" ("
	   <<_wmcf.m_origin
	   <<")]";

    return _stream;
  }

  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::WorkingMemoryChange &_wmc) {


    _stream<<"[WMC: "
	   <<_wmc.m_src
	   <<" "
	   <<_wmc.m_type
	   <<" "
	   <<_wmc.m_address
	   <<" "
	   <<_wmc.m_operation
	   <<"]";

    return _stream;
  }









  std::string lockQueryString(const std::string &  _id, 
			      const std::string &  _subarch,
			      const cdl::WorkingMemoryPermissions &  _permissions, 
			      const cdl::OperationMode &  _op) {
    ostringstream buf;
    buf<<_id<<'\\'<<_subarch<<'\\'<<toEnum(_permissions, _op)<<'\\';
    return buf.str();
  }



  std::string unlockQueryString(const std::string &  _id, 
				const std::string &  _subarch) {
    ostringstream buf;
    buf<<_id <<'\\' <<_subarch<<'\\'<<REQUEST_UNLOCK<<'\\';
    return buf.str();
  }

  std::string statusQueryString(const std::string &  _id, 
				const std::string &  _subarch) {
    ostringstream buf;
    buf<<_id<<'\\'<<_subarch<<'\\'<<REQUEST_STATUS<<'\\';
    return buf.str();
  }


  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::WorkingMemoryPermissions &_perm) {
    switch (_perm) {
    case LOCKED_O:
      _stream<< "LOCKED_O";
      break;
    case LOCKED_OD:
      _stream<< "LOCKED_OD";
      break;
    case LOCKED_ODR:
      _stream<< "LOCKED_ODR";
      break;
    case UNLOCKED:
      _stream<< "UNLOCKED";
      break;
    case DOES_NOT_EXIST:
      _stream<< "DOES_NOT_EXIST";
      break;
    default:
      _stream<<"unknown enum";
      break;
    }
    return _stream;
  }



  cdl::WorkingMemoryPermissions toPermissions(const cdl::WorkingMemoryLockRequest & _request) {

    if (REQUEST_LOCK_O == _request || REQUEST_TRY_LOCK_O == _request) {
      return LOCKED_O;
    } else if (REQUEST_LOCK_OD == _request || REQUEST_TRY_LOCK_OD == _request) {
      return LOCKED_OD;
    } else if (REQUEST_LOCK_ODR == _request || REQUEST_TRY_LOCK_ODR == _request) {
      return LOCKED_ODR;
    } else {
      return UNLOCKED;
    }
  }

  cdl::WorkingMemoryLockRequest toEnum(const cdl::WorkingMemoryPermissions &  _permissions, 
				       const cdl::OperationMode &  _op) {
    if (_permissions == LOCKED_O) {
      if (_op == BLOCKING) {
	return REQUEST_LOCK_O;
      } else if (_op == NON_BLOCKING) {
	return REQUEST_TRY_LOCK_O;
      } 
    } else if (_permissions == LOCKED_OD) {
      if (_op == BLOCKING) {
	return REQUEST_LOCK_OD;
      } else if (_op == NON_BLOCKING) {
	return REQUEST_TRY_LOCK_OD;
      } 
    } else if (_permissions == LOCKED_ODR) {
      if (_op == BLOCKING) {
	return REQUEST_LOCK_ODR;
      } else if (_op == NON_BLOCKING) {
	return REQUEST_TRY_LOCK_ODR;
      } 
    }

    throw CASTException(__HERE__,
			"invalid lock request: %d %d", _permissions, _op);
  }


  bool readAllowed(const cdl::WorkingMemoryPermissions &_permissions) {
    return _permissions == UNLOCKED
      || _permissions == LOCKED_O
      || _permissions == LOCKED_OD;
  }

  /**
   * Checks input enum for requisite permissions.
   * 
   * @param _permissions
   * @return
   */
  bool deleteAllowed(const cdl::WorkingMemoryPermissions &_permissions) {
    return _permissions == UNLOCKED
      || _permissions == LOCKED_O;
  }

  /**
   * Checks input enum for requisite permissions.
   * 
   * @param _permissions
   * @return
   */
  bool overwriteAllowed(const cdl::WorkingMemoryPermissions &_permissions) {
    return _permissions == UNLOCKED;
  }


  void initCASTDatatypes() {
    
    //cout<<"Process Server: CAST-specific initialisation code...";
    
    CASTDatatypeManager::addObjectDatatype<guitypes::DrawBatch>();
    
    addLocalDatatype<CASTWorkingMemoryEntry>();
    //HACK: local sequence types are not aligned
    addLocalDatatype<vector<CASTWorkingMemoryEntry*> >("cast::CASTWorkingMemoryEntryList");
    
    addLocalDatatype<std::string>();
    RemoteDatatypeManager::addObjectDatatype<std::string>();
    
    addLocalDatatype<int>();
    RemoteDatatypeManager::addObjectDatatype<int>();
    
    addLocalDatatype<bool>();
    RemoteDatatypeManager::addObjectDatatype<bool>();
    
    //working memory datatypes
    CASTDatatypeManager::addObjectDatatype<cdl::WorkingMemoryEntry>();
    CASTDatatypeManager::addObjectDatatype<cdl::WorkingMemoryChangeFilter>();
    CASTDatatypeManager::addObjectDatatype<cdl::WorkingMemoryChange>();
    CASTDatatypeManager::addSequenceDatatype<cdl::WorkingMemoryEntryList>();
    CASTDatatypeManager::addEnumDatatype<cdl::WorkingMemoryPermissions>();
    //CASTDatatypeManager::addSequenceDatatype<cdl::WorkingMemoryChangeList>();
    
    //goal management types
    CASTDatatypeManager::addObjectDatatype<cdl::InformationProcessingTask>();
    CASTDatatypeManager::addObjectDatatype<cdl::TaskManagementResult>();
    CASTDatatypeManager::addObjectDatatype<cdl::TaskResult>();
    CASTDatatypeManager::addSequenceDatatype<cdl::TaskDescriptionList>();
    
    //Arch UI types
    CASTDatatypeManager::addObjectDatatype<cdl::ui::TextOutput>();
    CASTDatatypeManager::addObjectDatatype<cdl::ui::ComponentStatus>();
    CASTDatatypeManager::addObjectDatatype<cdl::ui::ComponentEvent>();
    
    //cout<<" done"<<endl;
    
  }

  bool WorkingMemoryPointerComparator::operator () (const cdl::WorkingMemoryPointer & _f1, 
						    const cdl::WorkingMemoryPointer & _f2) const {
      
      //used to try to keep mapping with identical Java code
      static const bool BEFORE = true;
      static const bool EQUAL = false;
      static const bool AFTER = false;

      // same object
      if (&_f1 == &_f2) {
	return EQUAL;
      }

      // change id
      int comparison = strcmp(_f1.m_address.m_id,
			      _f2.m_address.m_id);
      if (comparison < 0) {
	return BEFORE;
      }
      if (comparison > 0) {
	return AFTER;
      }
  
      // change sa
      comparison = strcmp(_f1.m_address.m_subarchitecture, 
			  _f2.m_address.m_subarchitecture);
      if (comparison < 0) {
	return BEFORE;
      }
      if (comparison > 0) {
	return AFTER;
      }

      // ontological type
      comparison = strcmp(_f1.m_type,_f2.m_type);
      if (comparison < 0) {
	return BEFORE;
      }
      if (comparison > 0) {
	return AFTER;
      }

  
      return EQUAL;
      
  }

  
}
