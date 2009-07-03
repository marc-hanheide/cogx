/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes, Henrik Jacobsson
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


#ifndef CAST_CHANGE_FILTER_FACTORY_H_
#define CAST_CHANGE_FILTER_FACTORY_H_

#include <cast/slice/CDL.hpp>
#include <cast/core/CASTUtils.hpp>


/**
 * 
 * Helper functions to create change filters for passing to {@link
 * WorkingMemoryReaderProcess}.addChangerFilter. If you primarily care
 * about the type of the data look at the *TypeFilter* methods. If you
 * primarily care about the operation being performed look at the
 * createOperationFilter methods. For the id or address of the data,
 * look at the createID and createAddress methods. And for the
 * component that changed the data, look at createSourceFilter.
 * 
 * @author nah
 * 
 */

namespace cast {

  /**
   * Create an arbitrary change filter
   */
  inline
  cdl::WorkingMemoryChangeFilter
  createChangeFilter(const std::string & _type,
		     const cdl::WorkingMemoryOperation & _operation, 
		     const std::string & _src, 
		     const std::string & _changeID,
		     const std::string & _changeSA, 
		     const cdl::FilterRestriction & _restriction) {
    
    cdl::WorkingMemoryChangeFilter filter;
    filter.operation = _operation;
    filter.src = _src;
    filter.address.id = _changeID;
    filter.address.subarchitecture = _changeSA;
    filter.type = _type;
    filter.restriction = _restriction;
    return filter;
  }



  /**
   * Create an arbitrary change filter.
   */
  template <class Type>
  cdl::WorkingMemoryChangeFilter
  createChangeFilter(const cdl::WorkingMemoryOperation & _operation, 
		     const std::string & _src, 
		     const std::string & _changeID,
		     const std::string & _changeSA, 
		     const cdl::FilterRestriction & _restriction) {
    
    return createChangeFilter(typeName<Type>(),_operation, _src,_changeID, _changeSA, _restriction);
  }
 
  /**
   * Create a filter that matches the given type using the given
   * operation and restriction
   */
  template <class Type>
  cdl::WorkingMemoryChangeFilter 
  createTypeFilter(const cdl::WorkingMemoryOperation & _operation,
		   const cdl::FilterRestriction & _restriction) { 

    return createChangeFilter<Type>(_operation, 
				    "", 
				    "",
				    "", 
				    _restriction);
  }
    
 
  /**
   * Create a filter that matches all operations on the given type
   * across the architecture
   */
  template <class Type>
  cdl::WorkingMemoryChangeFilter 
  createGlobalTypeFilter(const cdl::WorkingMemoryOperation & _operation = cdl::WILDCARD) {
    return createTypeFilter<Type>(_operation, cdl::ALLSA);
  }

  /**
   * Create a filter that matches all operations on the given type
   * in the component's subarchitecture.
   */
  template <class Type>
  cdl::WorkingMemoryChangeFilter 
  createLocalTypeFilter(const cdl::WorkingMemoryOperation & _operation = cdl::WILDCARD) {
    return createTypeFilter<Type>(_operation, cdl::LOCALSA);
  }

  /**
   * Create a filter that matches all operations on the given type
   * using the given restriction.
   */
  template <class Type>
  cdl::WorkingMemoryChangeFilter 
  createTypeFilter(const cdl::FilterRestriction & _restriction) {
    return createTypeFilter<Type>(cdl::WILDCARD, _restriction);
  }

  /**
   * Create a filter that matches the given type using the given
   * operation.
   */
  template <class Type>
  cdl::WorkingMemoryChangeFilter 
  createTypeFilter(const cdl::WorkingMemoryOperation & _operation) {
    return createTypeFilter<Type>(_operation, cdl::ALLSA);
  }

  inline
  cdl::WorkingMemoryChangeFilter 
  createAddressFilter(const std::string & _id,
		      const std::string & _subarch,
		      const cdl::WorkingMemoryOperation & _operation = cdl::WILDCARD) {
    return createChangeFilter("",
			      _operation, 
			      "", 
			      _id,
			      _subarch, 
			      //this can be refined
			      //later in the actual
			      //component actually
			      //subarch local	
			      cdl::ALLSA);     
  }

   
  /**
   * Create a filter that listens to changes to the given id in the
   * local subarchitecture. Optional parameter provides the
   * operation to listen for.
   */
  inline
  cdl::WorkingMemoryChangeFilter 
  createIDFilter(const std::string & _id,
		 const cdl::WorkingMemoryOperation & _operation = cdl::WILDCARD) {
    return createAddressFilter(_id, "", _operation);
  }

  /**
   * Create a filter that listens to changes to the given id in the
   * local subarchitecture. Optional parameter provides the
   * operation to listen for.
   */
  inline
  cdl::WorkingMemoryChangeFilter 
  createAddressFilter(const cdl::WorkingMemoryAddress & _wma,
		      const cdl::WorkingMemoryOperation & _operation = cdl::WILDCARD) {
    return createAddressFilter(std::string(_wma.id), std::string(_wma.subarchitecture), _operation);
  }

 


  /**
   * Create a filter that listens to changes of a type made by a particular
   * component. Optional parameter provides the operation to listen
   * for.
   */

  template <class Type>
  cdl::WorkingMemoryChangeFilter 
  createSourceFilter(const std::string & _component,
		     const cdl::WorkingMemoryOperation & _operation = cdl::WILDCARD) {
    return createChangeFilter<Type>(_operation, 
				    _component, 
				    "",
				    "", 
				    cdl::ALLSA);
  }

  /**
   * Create a filter that listens to changes of made by a particular
   * component. Optional parameter provides the operation to listen
   * for.
   */    
  inline
  cdl::WorkingMemoryChangeFilter 
  createSourceFilter(const std::string & _component,
		     const cdl::WorkingMemoryOperation & _operation = cdl::WILDCARD) {
    return createChangeFilter("",
			      _operation, 
			      _component, 
			      "",
			      "", 
			      cdl::ALLSA);      
  }

    
  /**
   * Create a filter that listens to all untyped operations in the
   * local sa, with an option to extend this restriction.
   */    
  inline
  cdl::WorkingMemoryChangeFilter 
  createOperationFilter(const cdl::WorkingMemoryOperation & _operation,
			const cdl::FilterRestriction & _restriction = cdl::LOCALSA) {
    return createChangeFilter("",
			      _operation, 
			      "", 
			      "",
			      "", 
			      _restriction);      
  }


  

}//namespace cast


#endif
