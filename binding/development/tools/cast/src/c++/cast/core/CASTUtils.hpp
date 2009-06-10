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

#ifndef CAST_UTILS_HPP_
#define CAST_UTILS_HPP_

#include <cast/slice/CDL.hpp>


#include <Ice/ObjectAdapter.h>

#include <iostream>
#include <vector>
#include <set>

/**
 * A slightly hacky way to only get the pure file name without the whole path
 * from __FILE__
 */
#define __THIS_FILE__ ((strrchr(__FILE__, '/') ?: __FILE__ - 1) + 1)

/**
 * A convenience macro for keeping function calls short.
 */
#define __HERE__   __THIS_FILE__, __FUNCTION__, __LINE__


namespace cast {

  /**
   * Get the current communicator pointer
   */
  inline Ice::CommunicatorPtr getCommunicator(Ice::Current _crt = Ice::Current()) {
    return _crt.adapter->getCommunicator(); 
  }
  

  std::string exceptionMessage(const char *file, 
			       const char *function, 
			       int line,
			       const char *format, ...);

  cdl::WorkingMemoryAddress makeWorkingMemoryAddress(const std::string & _id,
						     const std::string & _subarch);


  void tokenizeString(const std::string & _str,
		      std::vector < std::string > &_tokens,
		      const std::string & _delimiters  = " ");


  cdl::CASTTime
  operator-(const cdl::CASTTime & _start,
	    const cdl::CASTTime & _end);



  bool
  operator<(const cdl::CASTTime & _ct1,
	    const cdl::CASTTime & _ct2);


  cdl::CASTTime
  castTimeSeconds(const double & _seconds);
	    

  cdl::CASTTime
  castTimeMillis(const long & _millis);

	    
  cdl::CASTTime
  castTimeMicros(const long & _micros);

	    

  /**
   *   Provide the id static id of the type
   */  
  template <class IceClass>
  const std::string& typeName() {
    return IceClass::ice_staticId();
  }


//   bool 
//   operator== (const cast::cdl::testing::CASTTestStruct& _ctsA, 
// 	      const cast::cdl::testing::CASTTestStruct & _ctsB);
  
//   bool 
//   operator== (const cast::cdl::WorkingMemoryChange& _wmcA, 
// 	      const cast::cdl::WorkingMemoryChange & _wmcB);
  
//   bool 
//   operator== (const cast::cdl::WorkingMemoryAddress& _wmaA, 
// 	      const cast::cdl::WorkingMemoryAddress & _wmaB);

  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::CASTTime &_ct);

  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::WorkingMemoryAddress &_wma);


  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::WorkingMemoryOperation &_op);

  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::FilterRestriction &_restriction);

  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::WorkingMemoryChangeFilter &_wmcf);

  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::WorkingMemoryChange &_wmcf);

  std::ostream & 
  operator<<(std::ostream &_stream, 
	     const cast::cdl::WorkingMemoryPermissions &_perm);
  

//   std::string lockQueryString(const std::string &  _id, 
// 			      const std::string &  _subarch,
// 			      const cdl::WorkingMemoryPermissions &  _permissions, 
// 			      const cdl::OperationMode &  _op);
  
//   std::string unlockQueryString(const std::string &  _id, 
// 				const std::string &  _subarch);

//   std::string statusQueryString(const std::string &  _id, 
// 				const std::string &  _subarch);
  
  // cdl::WorkingMemoryPermissions toPermissions(const cdl::WorkingMemoryLockRequest & _request);

//   cdl::WorkingMemoryLockRequest toEnum(const cdl::WorkingMemoryPermissions &  _permissions, 
// 				       const cdl::OperationMode &  _op);


  /**
   * Checks input enum for requisite permissions.
   * 
   * @param _permissions
   * @return
   */
  bool readAllowed(const cdl::WorkingMemoryPermissions &_permissions);

  /**
   * Checks input enum for requisite permissions.
   * 
   * @param _permissions
   * @return
   */
  bool deleteAllowed(const cdl::WorkingMemoryPermissions &_permissions);

  /**
   * Checks input enum for requisite permissions.
   * 
   * @param _permissions
   * @return
   */
  bool overwriteAllowed(const cdl::WorkingMemoryPermissions &_permissions);

//   /**
//    * Convenience function to instantiate pointer
//    */
//   template <class Type>
//   void workingMemoryPointer(const std::string & _id,
// 			    const std::string & _subarch,
// 			    cast::cdl::WorkingMemoryPointer & _wmp) {
//     _wmp.m_type = CORBA::string_dup(typeName<Type>().c_str());
//     _wmp.m_address.m_id = CORBA::string_dup(_id.c_str());
//     _wmp.m_address.m_subarchitecture = CORBA::string_dup(_subarch.c_str());
//   }

  
//   struct WorkingMemoryPointerComparator {    
//     bool operator () (const cdl::WorkingMemoryPointer & _f1, 
// 		      const cdl::WorkingMemoryPointer & _f2) const;
//   };

//   typedef std::set<cdl::WorkingMemoryPointer, WorkingMemoryPointerComparator> WorkingMemoryPointerSet;

}
#endif
