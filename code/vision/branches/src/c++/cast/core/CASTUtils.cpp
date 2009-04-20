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

// #include <sstream>
#include <cstdarg>
using namespace std;

namespace cast {

  using namespace cdl;
  
  std::string exceptionMessage(const char *file, 
			       const char *function, 
			       int line,
			       const char *format, ...)  {
    static char what[1024];
    static char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(what, 1024, format, arg_list);
    va_end(arg_list);
    snprintf(msg, 1024, "%s:%s:%d: %s", file, function, line, what);
    return msg;
  }

  cdl::WorkingMemoryAddress makeWorkingMemoryAddress(const std::string & _id,
						     const std::string & _subarch) {
    WorkingMemoryAddress wma;
    wma.id = _id;
    wma.subarchitecture = _subarch;
    return wma;
  }
  

   /**
    * Stolen from http://www.linuxselfhelp.com/HOWTO/C++Programming-HOWTO-7.html
    */
   void 
   tokenizeString(const string & _str,
		  vector < string > &_tokens,
		  const string & _delimiters) {
    // Skip delimiters at beginning.
    string::size_type lastPos = _str.find_first_not_of(_delimiters, 0);
    // Find first "non-delimiter".
    string::size_type pos = _str.find_first_of(_delimiters, lastPos);
  
    while(string::npos != pos || string::npos != lastPos) {
      // Found a token, add it to the vector.
      _tokens.push_back(_str.substr(lastPos, pos - lastPos));
      // Skip delimiters.  Note the "not_of"
      lastPos = _str.find_first_not_of(_delimiters, pos);
      // Find next "non-delimiter"
      pos = _str.find_first_of(_delimiters, lastPos);
    }
  }

  bool
  operator<(const cdl::CASTTime & _ct1,
	    const cdl::CASTTime & _ct2) {
    if(_ct1.s < _ct2.s) {
      return true;
    }
    else if(_ct1.s == _ct2.s) {
      return (_ct1.us < _ct2.us);
    }
    else {
      return false;
    }
  }

  cdl::CASTTime
  operator-(const cdl::CASTTime & _start,
	    const cdl::CASTTime & _end) {

//     if(_end > _start) {
//       return _end - _start;
//     }

    CASTTime t, startTime = _start;

    /* start the carry for the later subtraction by updating y. */
    if (_end.us < startTime.us) {
      int nsec = (startTime.us - _end.us) / 1000000 + 1;
      startTime.us -= 1000000 * nsec;
      startTime.s += nsec;
    }
    
    if (_end.us - startTime.us > 1000000) {
      int nsec = (_end.us - startTime.us) / 1000000;
      startTime.us += 1000000 * nsec;
      startTime.s -= nsec;
    }
    
    /* Compute the time remaining to wait.
       us is certainly positive. */
    t.s = _end.s - startTime.s;
    t.us = _end.us - startTime.us;
    
    return t;        
  }
	    

  cdl::CASTTime
    castTimeSeconds(const double & _seconds) {
    CASTTime ct;
    ct.s = _seconds;
    ct.us = ((_seconds - ct.s) * 1000000);
    return ct;
  }

  cdl::CASTTime
  castTimeMillis(const long & _millis) {
    static const long MILLIS_PER_MICROSECOND = 1000;
    static const long MICROS_PER_SECOND = 1000000;
    
    long asMicros = _millis * MILLIS_PER_MICROSECOND;
    CASTTime ct;
    ct.s = asMicros / MICROS_PER_SECOND;
    ct.us = asMicros % MICROS_PER_SECOND;
    return ct;
  }

  cdl::CASTTime
  castTimeMicros(const long & _micros) {
    static const long MICROS_PER_SECOND = 1000000;
    
    CASTTime ct;
    ct.s = _micros / MICROS_PER_SECOND;
    ct.us = _micros % MICROS_PER_SECOND;
    return ct;
  }


//   bool operator== (const cast::cdl::WorkingMemoryChange & _wmcA, 
// 		   const cast::cdl::WorkingMemoryChange & _wmcB) {

//     //cout<<"overidden equals"<<endl;
//     return _wmcA.address == _wmcB.address &&
//       _wmcA.operation == _wmcB.operation &&
//       (0 == strcmp(_wmcA.src, _wmcB.src)) &&
//       (0 == strcmp(_wmcA.type,_wmcB.type));
  
//   }

//   bool operator== (const cast::cdl::testing::CASTTestStruct & _ctsA, 
// 		   const cast::cdl::testing::CASTTestStruct & _ctsB) {

//     //cout<<"overidden equals"<<endl;
//     return _ctsA.count == _ctsB.count &&
//       _ctsA.change == _ctsB.change;
    
//   }


  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::CASTTime &_ct) {
    _stream<<"[CASTTime: "<<_ct.s<<":"<<_ct.us<<"]";
    return _stream;
  }

  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::WorkingMemoryAddress &_wma) {

    _stream<<"[WMA: "
	   <<_wma.subarchitecture
	   <<":"
	   <<_wma.id
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
    case cast::cdl::LOCALSA:
      _stream<<"LOCAL_SA";
      break;
    case cast::cdl::ALLSA:
      _stream<<"ALL_SA";
      break;
    }
  
    return _stream;

  }

  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::WorkingMemoryChangeFilter &_wmcf) {


    _stream<<"[WMCF: "
	   <<_wmcf.operation
	   <<" "
	   <<_wmcf.src
	   <<" "
	   <<_wmcf.address
	   <<" "
	   <<_wmcf.type
	   <<" "
	   <<_wmcf.restriction
	   <<" ("
	   <<_wmcf.origin
	   <<")]";

    return _stream;
  }
  
  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::WorkingMemoryChange &_wmc) {
    _stream<<"[WMC: "
	   <<_wmc.src
	   <<" "
	   <<_wmc.type
	   <<" "
	   <<_wmc.address
	   <<" "
	   <<_wmc.operation
	   <<"]";

    return _stream;
  }









//   std::string lockQueryString(const std::string &  _id, 
// 			      const std::string &  _subarch,
// 			      const cdl::WorkingMemoryPermissions &  _permissions, 
// 			      const cdl::OperationMode &  _op) {
//     ostringstream buf;
//     buf<<_id<<'\\'<<_subarch<<'\\'<<toEnum(_permissions, _op)<<'\\';
//     return buf.str();
//   }



//   std::string unlockQueryString(const std::string &  _id, 
// 				const std::string &  _subarch) {
//     ostringstream buf;
//     buf<<_id <<'\\' <<_subarch<<'\\'<<REQUEST_UNLOCK<<'\\';
//     return buf.str();
//   }

//   std::string statusQueryString(const std::string &  _id, 
// 				const std::string &  _subarch) {
//     ostringstream buf;
//     buf<<_id<<'\\'<<_subarch<<'\\'<<REQUEST_STATUS<<'\\';
//     return buf.str();
//   }


//   std::ostream & 
//   operator<<(std::ostream &_stream, 
// 	     const cast::cdl::WorkingMemoryPermissions &_perm) {
//     switch (_perm) {
//     case LOCKEDO:
//       _stream<< "LOCKEDO";
//       break;
//     case LOCKEDOD:
//       _stream<< "LOCKEDOD";
//       break;
//     case LOCKEDODR:
//       _stream<< "LOCKEDODR";
//       break;
//     case UNLOCKED:
//       _stream<< "UNLOCKED";
//       break;
//     case DOES_NOT_EXIST:
//       _stream<< "DOES_NOT_EXIST";
//       break;
//     default:
//       _stream<<"unknown enum";
//       break;
//     }
//     return _stream;
//   }



//   cdl::WorkingMemoryPermissions toPermissions(const cdl::WorkingMemoryLockRequest & _request) {

//     if (REQUEST_LOCK_O == _request || REQUEST_TRY_LOCK_O == _request) {
//       return LOCKEDO;
//     } else if (REQUEST_LOCK_OD == _request || REQUEST_TRY_LOCK_OD == _request) {
//       return LOCKEDOD;
//     } else if (REQUEST_LOCK_ODR == _request || REQUEST_TRY_LOCK_ODR == _request) {
//       return LOCKEDODR;
//     } else {
//       return UNLOCKED;
//     }
//   }

//   cdl::WorkingMemoryLockRequest toEnum(const cdl::WorkingMemoryPermissions &  _permissions, 
// 				       const cdl::OperationMode &  _op) {
//     if (_permissions == LOCKEDO) {
//       if (_op == BLOCKING) {
// 	return REQUEST_LOCK_O;
//       } else if (_op == NON_BLOCKING) {
// 	return REQUEST_TRY_LOCK_O;
//       } 
//     } else if (_permissions == LOCKEDOD) {
//       if (_op == BLOCKING) {
// 	return REQUEST_LOCK_OD;
//       } else if (_op == NON_BLOCKING) {
// 	return REQUEST_TRY_LOCK_OD;
//       } 
//     } else if (_permissions == LOCKEDODR) {
//       if (_op == BLOCKING) {
// 	return REQUEST_LOCK_ODR;
//       } else if (_op == NON_BLOCKING) {
// 	return REQUEST_TRY_LOCK_ODR;
//       } 
//     }

//     throw CASTException(__HERE__,
// 			"invalid lock request: %d %d", _permissions, _op);
//   }


  bool readAllowed(const cdl::WorkingMemoryPermissions &_permissions) {
    return _permissions == UNLOCKED
      || _permissions == LOCKEDO
      || _permissions == LOCKEDOD;
  }

  /**
   * Checks input enum for requisite permissions.
   * 
   * @param _permissions
   * @return
   */
  bool deleteAllowed(const cdl::WorkingMemoryPermissions &_permissions) {
    return _permissions == UNLOCKED
      || _permissions == LOCKEDO;
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


//   void initCASTDatatypes() {
    
//     //cout<<"Process Server: CAST-specific initialisation code...";
    
//     CASTDatatypeManager::addObjectDatatype<guitypes::DrawBatch>();
    
//     addLocalDatatype<CASTWorkingMemoryEntry>();
//     //HACK: local sequence types are not aligned
//     addLocalDatatype<vector<CASTWorkingMemoryEntry*> >("cast::CASTWorkingMemoryEntryList");
    
//     addLocalDatatype<std::string>();
//     RemoteDatatypeManager::addObjectDatatype<std::string>();
    
//     addLocalDatatype<int>();
//     RemoteDatatypeManager::addObjectDatatype<int>();
    
//     addLocalDatatype<bool>();
//     RemoteDatatypeManager::addObjectDatatype<bool>();
    
//     //working memory datatypes
//     CASTDatatypeManager::addObjectDatatype<cdl::WorkingMemoryEntry>();
//     CASTDatatypeManager::addObjectDatatype<cdl::WorkingMemoryChangeFilter>();
//     CASTDatatypeManager::addObjectDatatype<cdl::WorkingMemoryChange>();
//     CASTDatatypeManager::addSequenceDatatype<cdl::WorkingMemoryEntryList>();
//     CASTDatatypeManager::addEnumDatatype<cdl::WorkingMemoryPermissions>();
//     //CASTDatatypeManager::addSequenceDatatype<cdl::WorkingMemoryChangeList>();
    
//     //goal management types
//     CASTDatatypeManager::addObjectDatatype<cdl::InformationProcessingTask>();
//     CASTDatatypeManager::addObjectDatatype<cdl::TaskManagementResult>();
//     CASTDatatypeManager::addObjectDatatype<cdl::TaskResult>();
//     CASTDatatypeManager::addSequenceDatatype<cdl::TaskDescriptionList>();
    
//     //Arch UI types
//     CASTDatatypeManager::addObjectDatatype<cdl::ui::TextOutput>();
//     CASTDatatypeManager::addObjectDatatype<cdl::ui::ComponentStatus>();
//     CASTDatatypeManager::addObjectDatatype<cdl::ui::ComponentEvent>();
    
//     //cout<<" done"<<endl;
    
//   }

//   bool WorkingMemoryPointerComparator::operator () (const cdl::WorkingMemoryPointer & _f1, 
// 						    const cdl::WorkingMemoryPointer & _f2) const {
      
//       //used to try to keep mapping with identical Java code
//       static const bool BEFORE = true;
//       static const bool EQUAL = false;
//       static const bool AFTER = false;

//       // same object
//       if (&_f1 == &_f2) {
// 	return EQUAL;
//       }

//       // change id
//       int comparison = strcmp(_f1.address.id,
// 			      _f2.address.id);
//       if (comparison < 0) {
// 	return BEFORE;
//       }
//       if (comparison > 0) {
// 	return AFTER;
//       }
  
//       // change sa
//       comparison = strcmp(_f1.address.subarchitecture, 
// 			  _f2.address.subarchitecture);
//       if (comparison < 0) {
// 	return BEFORE;
//       }
//       if (comparison > 0) {
// 	return AFTER;
//       }

//       // ontological type
//       comparison = strcmp(_f1.type,_f2.type);
//       if (comparison < 0) {
// 	return BEFORE;
//       }
//       if (comparison > 0) {
// 	return AFTER;
//       }

  
//       return EQUAL;
      
//   }

}
