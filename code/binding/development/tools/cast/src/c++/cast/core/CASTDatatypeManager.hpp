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

#ifndef CAST_CAST_DATATYPE_MANAGER
#define CAST_CAST_DATATYPE_MANAGER

#include <balt/interface/RemoteDatatypeManager.hpp>
#include <balt/core/LocalDatatypeManager.hpp>
#include <balt/core/BALTLocalUtils.hpp>

namespace cast {

/**
 * Manages the datatypes that the CAST framework can use. Only
 * datatypes added to this class are able to be transmitted across
 * machines and languages unless they are primitive types or arrays of
 * primitive types.
 * 
 * @author nah
 */
class CASTDatatypeManager {

 public:

  /**
   * Add object class T to CAST. This then uses the underlying
   * framework to automatically generate the translation code.
   * 
   * @param _datatype
   *            The name that will be used to refer to type T
   */
  template <class T>
    static void addObjectDatatype() {
    RemoteDatatypeManager::addObjectDatatype<T>();
    addLocalDatatype<T>();
  }

  /**
   * Add enum class T to CAST. This then uses the underlying
   * framework to automatically generate the translation code.
   * 
   * @param _datatype
   *            The name that will be used to refer to type T
   */
  template <class T>
    static void addEnumDatatype() {
    RemoteDatatypeManager::addEnumDatatype<T>();
    addLocalDatatype<T>();
  }


  /**
   * Add sequence type T to CAST. This then uses the underlying
   * framework to automatically generate the translation code.
   * 
   * @param _datatype
   *            The name that will be used to refer to type T
   */
  template <class T>
    static void addSequenceDatatype() {
    RemoteDatatypeManager::addSequenceDatatype<T>();
    addLocalDatatype<T>();
  }
  

};

} //namespace cast

#endif
